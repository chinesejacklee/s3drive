#include "fs.h"
#include <string.h>
#include "esp_partition.h"
#include "esp_rom_crc.h"

#define SEC 4096u
#define BLK 16u  // sectors per 64 KB erase block
#define HALF (32u * SEC)
#define DATA0 (2 * HALF / SEC)  // first data sector (after the two log halves)
#define MAX_NAME 200
#define MAX_EXT 64
#define HDR_MAGIC 0x53463353u  // "S3FS"
#define REC_MAGIC 0x5AA5
enum { REC_ADD = 1, REC_DEL = 2 };
enum : uint8_t { USED, ERASED, DIRTY, UNKNOWN };

struct __attribute__((packed)) HalfHdr { uint32_t magic, gen, crc, rsv; };
struct __attribute__((packed)) RecHdr { uint16_t magic; uint8_t type, rsv; uint16_t len, rsv2; uint32_t crc; };

static const esp_partition_t *part;
static const uint8_t *map;  // whole partition memory-mapped (reads bypass the cache-disabling flash API)
static uint16_t nsec;
static uint8_t *st;  // per-sector state
static std::vector<FEntry> files;
static int active;
static uint32_t gen, log_off, other_erase_pos;
static bool other_erased, need_compact;
static uint16_t alloc_cursor = DATA0, bg_cursor = DATA0;

// in-progress upload
static std::vector<uint16_t> pend;
static std::string put_name;
static uint32_t put_size, put_pos, put_crc, put_mtime;
static bool put_active;

static inline uint32_t align4(uint32_t n) { return (n + 3) & ~3u; }
static inline uint32_t half_base(int h) { return h * HALF; }
static inline bool is_free(uint16_t s) { return st[s] != USED; }

static int rd(uint32_t addr, void *dst, size_t n) {
  if (map) { memcpy(dst, map + addr, n); return ST_OK; }
  return esp_partition_read(part, addr, dst, n) == ESP_OK ? ST_OK : ST_IO;
}
static int wr(uint32_t addr, const void *src, size_t n) { return esp_partition_write(part, addr, src, n) == ESP_OK ? ST_OK : ST_IO; }
static int er(uint32_t addr, size_t n) { return esp_partition_erase_range(part, addr, n) == ESP_OK ? ST_OK : ST_IO; }

static bool sector_blank(uint16_t s) {
  static uint32_t tmp[SEC / 4];
  if (rd(s * SEC, tmp, SEC)) return false;
  for (uint32_t w : tmp)
    if (w != 0xFFFFFFFF) return false;
  return true;
}

// ---- metadata records ----
static uint32_t rec_size(const FEntry &e) { return align4(sizeof(RecHdr) + 15 + e.name.size() + 4 * e.ext.size()); }

static uint16_t encode_add(const FEntry &e, uint8_t *p) {
  uint16_t next = e.ext.size();
  uint8_t nlen = e.name.size();
  memcpy(p, &e.size, 4);
  memcpy(p + 4, &e.crc, 4);
  memcpy(p + 8, &e.mtime, 4);
  memcpy(p + 12, &next, 2);
  p[14] = nlen;
  memcpy(p + 15, e.name.data(), nlen);
  memcpy(p + 15 + nlen, e.ext.data(), 4 * next);
  return 15 + nlen + 4 * next;
}

static bool decode_add(const uint8_t *p, uint16_t len, FEntry &e) {
  if (len < 15) return false;
  uint16_t next;
  memcpy(&e.size, p, 4);
  memcpy(&e.crc, p + 4, 4);
  memcpy(&e.mtime, p + 8, 4);
  memcpy(&next, p + 12, 2);
  uint8_t nlen = p[14];
  if (len != 15 + nlen + 4 * next) return false;
  e.name.assign((const char *)p + 15, nlen);
  e.ext.resize(next);
  memcpy(e.ext.data(), p + 15 + nlen, 4 * next);
  uint32_t secs = 0;
  for (auto &x : e.ext) {
    if (x.start < DATA0 || x.start + x.count > nsec) return false;
    secs += x.count;
  }
  return secs == (e.size + SEC - 1) / SEC;
}

static int find_idx(const std::string &name) {
  for (size_t i = 0; i < files.size(); i++)
    if (files[i].name == name) return i;
  return -1;
}

static void mark(const FEntry &e, uint8_t state) {
  for (auto &x : e.ext)
    for (uint16_t s = x.start; s < x.start + x.count; s++) st[s] = state;
}

static int write_rec(int h, uint32_t off, uint8_t type, const uint8_t *payload, uint16_t len) {
  static uint8_t buf[sizeof(RecHdr) + 15 + 255 + 4 * MAX_EXT + 4];
  RecHdr r = {REC_MAGIC, type, 0, len, 0, 0};
  r.crc = esp_rom_crc32_le(esp_rom_crc32_le(0, &type, 1), payload, len);
  memcpy(buf, &r, sizeof r);
  memcpy(buf + sizeof r, payload, len);
  uint32_t total = align4(sizeof r + len);
  memset(buf + sizeof r + len, 0xFF, total - sizeof r - len);
  return wr(half_base(h) + off, buf, total);
}

static int write_hdr(int h, uint32_t g) {
  HalfHdr hd = {HDR_MAGIC, g, 0, 0};
  hd.crc = esp_rom_crc32_le(0, (const uint8_t *)&hd, 8);
  return wr(half_base(h), &hd, sizeof hd);
}

static int erase_other_now() {
  if (other_erased) return ST_OK;
  int r = er(half_base(active ^ 1), HALF);
  other_erased = r == ST_OK;
  other_erase_pos = 0;
  return r;
}

// Rewrite all live entries into the other half, then switch to it.
static int compact() {
  int o = active ^ 1, r = erase_other_now();
  if (r) return r;
  other_erased = false;  // from here on it holds data (or a failed attempt)
  other_erase_pos = 0;
  uint32_t off = sizeof(HalfHdr);
  static uint8_t pl[15 + 255 + 4 * MAX_EXT];
  for (auto &e : files) {
    if (off + rec_size(e) > HALF) return ST_FULL;
    uint16_t n = encode_add(e, pl);
    if ((r = write_rec(o, off, REC_ADD, pl, n))) return r;
    off += rec_size(e);
  }
  if ((r = write_hdr(o, gen + 1))) return r;
  active = o;  // the old half is now the spare and still dirty (other_erased == false)
  gen++;
  log_off = off;
  need_compact = false;
  return ST_OK;
}

static int log_append(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint32_t total = align4(sizeof(RecHdr) + len);
  if (need_compact || log_off + total > HALF) {
    int r = compact();
    if (r) return r;
    if (log_off + total > HALF) return ST_FULL;
  }
  int r = write_rec(active, log_off, type, payload, len);
  if (r == ST_OK) log_off += total;
  else need_compact = true;  // the tail may be half-programmed: never append there again
  return r;
}

static uint32_t meta_bytes() {
  uint32_t n = sizeof(HalfHdr);
  for (auto &e : files) n += rec_size(e);
  return n;
}

// ---- mount / format ----
static bool read_hdr(int h, uint32_t &g) {
  HalfHdr hd;
  if (rd(half_base(h), &hd, sizeof hd)) return false;
  if (hd.magic != HDR_MAGIC || hd.crc != esp_rom_crc32_le(0, (const uint8_t *)&hd, 8)) return false;
  g = hd.gen;
  return true;
}

static void replay() {
  static uint8_t pl[15 + 255 + 4 * MAX_EXT];
  uint32_t off = sizeof(HalfHdr);
  for (;;) {
    RecHdr r;
    if (off + sizeof r > HALF || rd(half_base(active) + off, &r, sizeof r)) break;
    if (r.magic == 0xFFFF) {  // clean end of log only if the header area is fully blank
      const uint32_t *w = (const uint32_t *)&r;
      if ((w[0] & w[1] & w[2]) != 0xFFFFFFFF) need_compact = true;
      break;
    }
    if (r.magic != REC_MAGIC || r.len > sizeof pl || off + align4(sizeof r + r.len) > HALF ||
        rd(half_base(active) + off + sizeof r, pl, r.len) ||
        r.crc != esp_rom_crc32_le(esp_rom_crc32_le(0, &r.type, 1), pl, r.len)) {
      need_compact = true;  // torn write at the tail (power loss): recover by compacting
      break;
    }
    FEntry e;
    if (r.type == REC_ADD && decode_add(pl, r.len, e)) {
      int i = find_idx(e.name);
      if (i >= 0) files[i] = std::move(e);
      else files.push_back(std::move(e));
    } else if (r.type == REC_DEL && r.len >= 1 && r.len == 1 + pl[0]) {
      int i = find_idx(std::string((const char *)pl + 1, pl[0]));
      if (i >= 0) files.erase(files.begin() + i);
    }
    off += align4(sizeof r + r.len);
  }
  log_off = off;
}

int fs_format() {
  int r;
  files.clear();
  if ((r = er(0, 2 * HALF)) || (r = write_hdr(0, 1))) return r;
  active = 0;
  gen = 1;
  log_off = sizeof(HalfHdr);
  other_erased = true;
  need_compact = false;
  for (uint16_t s = DATA0; s < nsec; s++) st[s] = UNKNOWN;
  return ST_OK;
}

bool fs_mount() {
  part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
  if (!part) return false;
  nsec = part->size / SEC;
  st = (uint8_t *)malloc(nsec);
  esp_partition_mmap_handle_t h;
  const void *p;
  if (esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA, &p, &h) == ESP_OK) map = (const uint8_t *)p;
  uint32_t g0 = 0, g1 = 0;
  bool v0 = read_hdr(0, g0), v1 = read_hdr(1, g1);
  if (!v0 && !v1) return fs_format() == ST_OK;
  active = (v0 && (!v1 || g0 > g1)) ? 0 : 1;
  gen = active ? g1 : g0;
  replay();
  other_erased = false;
  other_erase_pos = 0;
  for (uint16_t s = DATA0; s < nsec; s++) st[s] = UNKNOWN;
  for (auto &e : files) mark(e, USED);
  return true;
}

// ---- queries ----
const std::vector<FEntry> &fs_list() { return files; }
FEntry *fs_find(const std::string &name) {
  int i = find_idx(name);
  return i < 0 ? nullptr : &files[i];
}
uint32_t fs_total() { return (nsec - DATA0) * SEC; }
static uint32_t count(bool (*f)(uint8_t)) {
  uint32_t n = 0;
  for (uint16_t s = DATA0; s < nsec; s++) n += f(st[s]);
  return n * SEC;
}
uint32_t fs_free() { return count([](uint8_t v) { return v != USED; }); }
uint32_t fs_ready() { return count([](uint8_t v) { return v == ERASED; }); }

// ---- upload ----
// Collect `need` free sectors starting at alloc_cursor (next-fit, spreads wear).
// First try only already-erased sectors; if that is not enough, any free sector.
static bool alloc(uint32_t need, bool erased_only) {
  pend.clear();
  uint16_t nd = nsec - DATA0, runs = 0;
  bool in_run = false;
  for (uint16_t k = 0; k < nd && pend.size() < need; k++) {
    uint16_t s = DATA0 + (alloc_cursor - DATA0 + k) % nd;
    bool ok = erased_only ? st[s] == ERASED : is_free(s);
    if (ok && (!in_run || s != pend.back() + 1)) runs++;
    in_run = ok;
    if (ok) pend.push_back(s);
  }
  return pend.size() == need && runs <= MAX_EXT;
}

int fs_put_begin(const std::string &name, uint32_t size, uint32_t mtime) {
  if (name.empty() || name.size() > MAX_NAME) return ST_BADREQ;
  uint32_t need = (size + SEC - 1) / SEC;
  if (need * SEC > fs_free()) return ST_NOSPACE;
  FEntry probe{name, 0, 0, 0, std::vector<Ext>(MAX_EXT)};
  if (meta_bytes() + rec_size(probe) > HALF - SEC) return ST_FULL;
  if (!alloc(need, true) && !alloc(need, false)) return ST_FULL;  // too fragmented
  put_name = name;
  put_size = size;
  put_mtime = mtime;
  put_pos = put_crc = 0;
  put_active = true;
  return ST_OK;
}

// Make pend[i] writable; erase a whole 64 KB block when the next 16 pending sectors cover it.
static int prep(size_t i) {
  uint16_t s = pend[i];
  if (st[s] == ERASED) return ST_OK;
  bool block = s % BLK == 0 && i + BLK <= pend.size();
  for (size_t k = 1; block && k < BLK; k++) block = pend[i + k] == s + k;
  int r = er(s * SEC, block ? BLK * SEC : SEC);
  if (r == ST_OK)
    for (uint16_t k = 0; k < (block ? BLK : 1); k++) st[s + k] = ERASED;
  return r;
}

int fs_put_write(const uint8_t *data, size_t len) {
  if (!put_active || put_pos + len > put_size) return ST_BADREQ;
  while (len) {
    size_t i = put_pos / SEC, off = put_pos % SEC, n = SEC - off;
    if (n > len) n = len;
    int r;
    if (off == 0 && (r = prep(i))) return r;
    st[pend[i]] = DIRTY;  // programmed but not committed: reclaimed if the upload fails
    if ((r = wr(pend[i] * SEC + off, data, n))) return r;
    put_crc = esp_rom_crc32_le(put_crc, data, n);
    put_pos += n;
    data += n;
    len -= n;
  }
  return ST_OK;
}

int fs_put_commit(uint32_t crc) {
  if (!put_active) return ST_BADREQ;
  put_active = false;
  if (put_pos != put_size) return ST_BADREQ;
  if (put_crc != crc) return ST_CRC;
  FEntry e{put_name, put_size, crc, put_mtime, {}};
  for (uint16_t s : pend) {
    if (!e.ext.empty() && e.ext.back().start + e.ext.back().count == s) e.ext.back().count++;
    else e.ext.push_back({s, 1});
  }
  static uint8_t pl[15 + 255 + 4 * MAX_EXT];
  int r = log_append(REC_ADD, pl, encode_add(e, pl));
  if (r) return r;
  int i = find_idx(put_name);
  if (i >= 0) {
    mark(files[i], DIRTY);
    files[i] = std::move(e);
    mark(files[i], USED);
  } else {
    files.push_back(std::move(e));
    mark(files.back(), USED);
  }
  if (!pend.empty()) alloc_cursor = pend.back() + 1 < nsec ? pend.back() + 1 : DATA0;
  return ST_OK;
}

void fs_put_abort() { put_active = false; }

// ---- read / delete ----
int fs_read(const FEntry &e, uint32_t off, uint8_t *dst, size_t len) {
  uint32_t base = 0;
  for (auto &x : e.ext) {
    uint32_t xl = x.count * SEC;
    while (len && off < base + xl) {
      size_t n = base + xl - off;
      if (n > len) n = len;
      int r = rd(x.start * SEC + (off - base), dst, n);
      if (r) return r;
      off += n;
      dst += n;
      len -= n;
    }
    base += xl;
  }
  return len ? ST_BADREQ : ST_OK;
}

int fs_delete(const std::string &name) {
  int i = find_idx(name);
  if (i < 0) return ST_NOTFOUND;
  uint8_t pl[1 + 255];
  pl[0] = name.size();
  memcpy(pl + 1, name.data(), name.size());
  int r = log_append(REC_DEL, pl, 1 + name.size());
  if (r) return r;
  mark(files[i], DIRTY);
  files.erase(files.begin() + i);
  return ST_OK;
}

// ---- background: erase the spare log half, classify unknown sectors, erase dirty ones ----
bool fs_bg_step() {
  if (!other_erased) {
    uint32_t base = half_base(active ^ 1) + other_erase_pos;
    if (other_erase_pos == 0) {  // skip the erase if the whole half is already blank
      bool blank = true;
      for (uint32_t o = 0; blank && o < HALF; o += SEC) blank = sector_blank((base + o) / SEC);
      if (blank) { other_erased = true; return true; }
    }
    er(base, BLK * SEC);
    other_erase_pos += BLK * SEC;
    if (other_erase_pos >= HALF) other_erased = true;
    return true;
  }
  uint16_t nd = nsec - DATA0;
  for (uint16_t k = 0; k < nd; k++) {
    uint16_t s = DATA0 + (bg_cursor - DATA0 + k) % nd;
    if (st[s] == UNKNOWN) {
      st[s] = sector_blank(s) ? ERASED : DIRTY;
      bg_cursor = s;
      return true;
    }
    if (st[s] == DIRTY) {
      uint16_t b = s - s % BLK;
      bool whole = true;
      for (uint16_t t = b; whole && t < b + BLK; t++) whole = is_free(t);
      if (whole) {
        for (uint16_t t = b; t < b + BLK; t++)  // classify first: skip the erase if already blank
          if (st[t] == UNKNOWN) st[t] = sector_blank(t) ? ERASED : DIRTY;
        if (er(b * SEC, BLK * SEC) == ST_OK)
          for (uint16_t t = b; t < b + BLK; t++) st[t] = ERASED;
      } else if (er(s * SEC, SEC) == ST_OK) {
        st[s] = ERASED;
      }
      bg_cursor = s;
      return true;
    }
  }
  return false;
}
