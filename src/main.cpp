// S3Drive firmware: ESP32-S3 flash as fast USB file storage (host app: s3drive.py).
//
// Wire protocol over the WinUSB bulk pipe (little-endian):
//   request : Req (16 B) + name (nlen B) [+ file data for PUT]
//   response: Resp (8 B) + len payload bytes
//   'I' info    -> total, free, ready, nfiles, version (5 x u32)
//   'L' list    -> per file: size, crc, mtime (u32), nlen (u8), name
//   'P' put     -> Resp(status); if OK host streams `size` bytes -> Resp(status after CRC check)
//   'G' get     -> Resp(len = 4 + size): crc (u32) + data
//   'D' delete, 'F' format -> Resp(status)
#include <Arduino.h>
#include "fs.h"
#include "usbio.h"

#define FW_VERSION 1
#define IOB 32768

struct __attribute__((packed)) Req { uint8_t op, nlen; uint16_t rsv; uint32_t size, crc, mtime; };
struct __attribute__((packed)) Resp { uint8_t status, rsv[3]; uint32_t len; };

static SemaphoreHandle_t fs_lock;
static volatile uint32_t last_io;
static uint8_t *iobuf;

static void reply(uint8_t status, const void *p = nullptr, uint32_t len = 0) {
  Resp r = {status, {0, 0, 0}, len};
  usbio_write(&r, sizeof r);
  if (len) usbio_write(p, len);
  usbio_flush();
}

static void do_info() {
  uint32_t v[5] = {fs_total(), fs_free(), fs_ready(), (uint32_t)fs_list().size(), FW_VERSION};
  reply(ST_OK, v, sizeof v);
}

static void do_list() {
  std::string out;
  for (auto &e : fs_list()) {
    uint32_t h[3] = {e.size, e.crc, e.mtime};
    out.append((const char *)h, sizeof h);
    out.push_back((char)e.name.size());
    out += e.name;
  }
  reply(ST_OK, out.data(), out.size());
}

static void do_put(const Req &q, const std::string &name) {
  int st = fs_put_begin(name, q.size, q.mtime);
  reply(st);
  if (st) return;
  for (uint32_t left = q.size; left;) {
    uint32_t k = left < IOB ? left : IOB;
    if (usbio_read(iobuf, k, 5000) != k) {  // host vanished or aborted: drop the upload
      fs_put_abort();
      return;
    }
    if (st == ST_OK) st = fs_put_write(iobuf, k);  // on error keep draining so the host isn't stuck
    left -= k;
  }
  if (st == ST_OK) st = fs_put_commit(q.crc);
  else fs_put_abort();
  reply(st);
}

static void do_get(const std::string &name) {
  FEntry *e = fs_find(name);
  if (!e) return reply(ST_NOTFOUND);
  Resp r = {ST_OK, {0, 0, 0}, 4 + e->size};
  usbio_write(&r, sizeof r);
  usbio_write(&e->crc, 4);
  for (uint32_t off = 0; off < e->size;) {
    uint32_t k = e->size - off < IOB ? e->size - off : IOB;
    fs_read(*e, off, iobuf, k);  // a read error shows up as a CRC mismatch on the host
    if (usbio_write(iobuf, k) != k) return;
    off += k;
  }
  usbio_flush();
}

static void proto_task(void *) {
  for (;;) {
    Req q;
    char name[256];
    if (usbio_read(&q, sizeof q, 500) != sizeof q || (q.nlen && usbio_read(name, q.nlen, 500) != q.nlen)) {
      if (usbio_aborted()) usbio_reset();
      continue;
    }
    last_io = millis();
    std::string nm(name, q.nlen);
    xSemaphoreTake(fs_lock, portMAX_DELAY);
    switch (q.op) {
      case 'I': do_info(); break;
      case 'L': do_list(); break;
      case 'P': do_put(q, nm); break;
      case 'G': do_get(nm); break;
      case 'D': reply(fs_delete(nm)); break;
      case 'F': reply(fs_format()); break;
      default: reply(ST_BADREQ);
    }
    xSemaphoreGive(fs_lock);
    last_io = millis();
  }
}

// Pre-erase free space while the link is idle, so uploads only pay the program time.
static void bg_task(void *) {
  for (;;) {
    bool more = false;
    if (millis() - last_io > 300 && xSemaphoreTake(fs_lock, 0) == pdTRUE) {
      more = fs_bg_step();
      xSemaphoreGive(fs_lock);
    }
    vTaskDelay(pdMS_TO_TICKS(more ? 1 : 100));
  }
}

void setup() {
  Serial.begin(115200);
  iobuf = (uint8_t *)heap_caps_malloc(IOB, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  fs_lock = xSemaphoreCreateMutex();
  bool ok = fs_mount();
  Serial.printf("S3Drive: mount %s, %u files, %lu/%lu KB free\n", ok ? "ok" : "FAILED", (unsigned)fs_list().size(),
                (unsigned long)(fs_free() / 1024), (unsigned long)(fs_total() / 1024));
  usbio_begin(0x303A, 0x4DD0, "S3Drive");
  xTaskCreatePinnedToCore(proto_task, "proto", 8192, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(bg_task, "fsbg", 4096, nullptr, 1, nullptr, 0);
}

void loop() { vTaskDelay(portMAX_DELAY); }
