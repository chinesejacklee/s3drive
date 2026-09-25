// S3FS: minimal extent-based file store tuned for write speed.
// - Files live in 4 KB sectors grouped into extents; data sectors are erased in the
//   background while idle, so uploads only pay the flash *program* time.
// - Metadata is an append-only log (CRC per record) in two 128 KB halves; when a half
//   fills, live entries are compacted into the other one. Power-loss safe: a file only
//   exists once its record is committed, after its data and CRC are verified.
// Not thread-safe: callers serialise access (one lock in main.cpp).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

enum { ST_OK, ST_NOTFOUND, ST_NOSPACE, ST_CRC, ST_IO, ST_BADREQ, ST_FULL };

struct Ext { uint16_t start, count; };  // in sectors
struct FEntry {
  std::string name;
  uint32_t size, crc, mtime;
  std::vector<Ext> ext;
};

bool fs_mount();
int fs_format();
const std::vector<FEntry> &fs_list();
FEntry *fs_find(const std::string &name);
uint32_t fs_total();
uint32_t fs_free();   // bytes not used by files
uint32_t fs_ready();  // free bytes already erased (fast to write)

int fs_put_begin(const std::string &name, uint32_t size, uint32_t mtime);
int fs_put_write(const uint8_t *data, size_t len);
int fs_put_commit(uint32_t crc);  // crc of the whole file as computed by the host
void fs_put_abort();

int fs_read(const FEntry &e, uint32_t off, uint8_t *dst, size_t len);
int fs_delete(const std::string &name);
bool fs_bg_step();  // one unit of background erase work; false when nothing left
