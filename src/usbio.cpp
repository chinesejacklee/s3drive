// The stock vendor class moves one 64-byte packet per re-arm (its FIFO is compiled at
// 64 bytes), so every following OUT packet is NAKed and retried (~500 KB/s). Instead we
// register our own class driver (app drivers are matched before the built-in ones) and
// arm ~8 KB multi-packet transfers straight into a ring of buffers (~0.9-1 MB/s).
// Back-pressure is plain USB flow control: with no free buffer, OUT is simply not armed.
//
// Buffer ownership: rx/tx index queues; all endpoint state is touched only in the
// TinyUSB task (driver callbacks + usbd_defer_func), the protocol task only uses queues.
#include "usbio.h"
#include <Arduino.h>
#include "USB.h"
#include "esp32-hal-tinyusb.h"
#include "tusb.h"
#include "device/usbd_pvt.h"

#define XF (127 * 64)  // bytes per transfer: ESP32-S3 OTG packet counter is 7 bits (GHWCFG3)
#define NRX 8
#define NTX 4
#define EP_SZ 64
#define NOBUF 0xFF
#define REQ_ABORT 0x41
#define REQ_MSOS 0x02  // VENDOR_REQUEST_MICROSOFT in the core's BOS descriptor

struct Xfer { uint8_t idx, epoch; uint16_t len; };

static uint8_t *rxb[NRX], *txb[NTX];
static QueueHandle_t rx_full, rx_free, tx_full, tx_free;
static uint8_t ep_out, ep_in, rx_idx, tx_idx;
static volatile uint8_t epoch;  // bumped on abort/reset: stale transfers are dropped
static volatile bool rx_armed, tx_busy, aborted;
static uint8_t rx_epoch;

// ---------------- TinyUSB task context ----------------
static void arm_rx(void * = nullptr) {
  if (rx_armed || !ep_out || xQueueReceive(rx_free, &rx_idx, 0) != pdTRUE) return;
  rx_epoch = epoch;
  rx_armed = usbd_edpt_xfer(0, ep_out, rxb[rx_idx], XF, false);
  if (!rx_armed) xQueueSendToFront(rx_free, &rx_idx, 0);
}

static void kick_tx(void * = nullptr) {
  Xfer x;
  if (tx_busy || !ep_in || xQueueReceive(tx_full, &x, 0) != pdTRUE) return;
  tx_idx = x.idx;
  tx_busy = usbd_edpt_xfer(0, ep_in, x.idx == NOBUF ? nullptr : txb[x.idx], x.len, false);
  if (!tx_busy && x.idx != NOBUF) xQueueSend(tx_free, &x.idx, 0);
}

static void drop_io(bool bus_reset) {
  Xfer x = {NOBUF, epoch, 0};
  epoch = epoch + 1;
  aborted = true;
  xQueueSend(rx_full, &x, 0);  // wake a blocked reader
  while (xQueueReceive(tx_full, &x, 0) == pdTRUE)
    if (x.idx != NOBUF) xQueueSend(tx_free, &x.idx, 0);
  if (bus_reset) {  // endpoints are gone: in-flight buffers will never complete
    if (rx_armed) xQueueSend(rx_free, &rx_idx, 0);
    if (tx_busy && tx_idx != NOBUF) xQueueSend(tx_free, &tx_idx, 0);
    rx_armed = false;
    tx_busy = false;
    ep_in = ep_out = 0;
  }
}

static void drv_init(void) {}
static bool drv_deinit(void) { return true; }
static void drv_reset(uint8_t) { drop_io(true); }

static uint16_t drv_open(uint8_t rhport, tusb_desc_interface_t const *d, uint16_t max_len) {
  TU_VERIFY(d->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC, 0);
  uint16_t len = sizeof(tusb_desc_interface_t);
  uint8_t const *p = tu_desc_next(d);
  for (uint8_t n = 0; n < d->bNumEndpoints && len < max_len; p = tu_desc_next(p)) {
    if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
      auto ep = (tusb_desc_endpoint_t const *)p;
      TU_ASSERT(usbd_edpt_open(rhport, ep), 0);
      (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN ? ep_in : ep_out) = ep->bEndpointAddress;
      n++;
    }
    len += tu_desc_len(p);
  }
  arm_rx();
  return len;
}

static bool drv_control(uint8_t, uint8_t, tusb_control_request_t const *) { return false; }

static bool drv_xfer_cb(uint8_t, uint8_t ep, xfer_result_t, uint32_t n) {
  if (ep == ep_out) {
    Xfer x = {rx_idx, rx_epoch, (uint16_t)n};
    rx_armed = false;
    xQueueSend(rx_full, &x, 0);
    arm_rx();
  } else if (ep == ep_in) {
    tx_busy = false;
    if (tx_idx != NOBUF) xQueueSend(tx_free, &tx_idx, 0);
    kick_tx();
  }
  return true;
}

static const usbd_class_driver_t s3_driver = {
  "S3DRIVE", drv_init, drv_deinit, drv_reset, drv_open, drv_control, drv_xfer_cb, nullptr, nullptr,
};
extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *count) {
  *count = 1;
  return &s3_driver;
}

// The core always serves a BOS advertising an MS OS 2.0 set (vendor code 2, 0xB2 bytes),
// but its set uses function-subset headers, which Windows ignores on a single-interface
// device. With webUSB off the core forwards code 2 to us, so we answer with a plain
// (subset-free) set of the same length -> Windows auto-binds WinUSB, no driver install.
#define MSOS_LEN 0xB2
static const uint8_t msos_desc[] = {
  U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MSOS_LEN),
  U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  U16_TO_U8S_LE(0x0084), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
  'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
  'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
  U16_TO_U8S_LE(0x0050),
  '{', 0, '5', 0, '3', 0, '3', 0, 'D', 0, '5', 0, '2', 0, '1', 0, '4', 0, '-', 0, '5', 0, '3', 0, '0', 0, '4', 0, '-', 0,
  '4', 0, 'D', 0, '5', 0, '2', 0, '-', 0, '9', 0, 'E', 0, '7', 0, '1', 0, '-', 0, '5', 0, '3', 0, '3', 0, 'D', 0, '5', 0,
  '2', 0, '1', 0, '4', 0, '0', 0, '0', 0, '0', 0, '1', 0, '}', 0, 0, 0, 0, 0,
  // 16-byte filler to match the core's advertised length: REG_BINARY "X" = {0,0}
  U16_TO_U8S_LE(0x0010), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), U16_TO_U8S_LE(0x0003), U16_TO_U8S_LE(0x0004),
  'X', 0, 0, 0, U16_TO_U8S_LE(0x0002), 0, 0,
};
static_assert(sizeof msos_desc == MSOS_LEN, "MS OS 2.0 descriptor length");

extern "C" bool tinyusb_vendor_control_request_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
  if (req->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) return false;
  if (stage != CONTROL_STAGE_SETUP) return true;
  if (req->bRequest == REQ_MSOS && req->wIndex == 7) return tud_control_xfer(rhport, req, (void *)msos_desc, MSOS_LEN);
  if (req->bRequest == REQ_ABORT) {
    drop_io(false);
    return tud_control_status(rhport, req);
  }
  return false;
}

static uint16_t load_desc(uint8_t *dst, uint8_t *itf) {
  uint8_t str = tinyusb_add_string_descriptor("S3Drive");
  uint8_t ep = tinyusb_get_free_duplex_endpoint();
  TU_VERIFY(ep != 0);
  uint8_t d[TUD_VENDOR_DESC_LEN] = {TUD_VENDOR_DESCRIPTOR(*itf, str, ep, (uint8_t)(0x80 | ep), EP_SZ)};
  *itf += 1;
  memcpy(dst, d, sizeof d);
  return sizeof d;
}

// ---------------- protocol task context ----------------
static Xfer rd;
static uint16_t rd_off;
static bool rd_have, wr_have;
static uint8_t wr_idx;
static uint16_t wr_len;
static bool msg_open, last_full;  // last submitted transfer was a multiple of EP_SZ

void usbio_begin(uint16_t vid, uint16_t pid, const char *product) {
  rx_full = xQueueCreate(NRX + 2, sizeof(Xfer));
  tx_full = xQueueCreate(NTX + 2, sizeof(Xfer));
  rx_free = xQueueCreate(NRX, 1);
  tx_free = xQueueCreate(NTX, 1);
  for (uint8_t i = 0; i < NRX; i++) {
    rxb[i] = (uint8_t *)heap_caps_malloc(XF, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    xQueueSend(rx_free, &i, 0);
  }
  for (uint8_t i = 0; i < NTX; i++) {
    txb[i] = (uint8_t *)heap_caps_malloc(XF, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    xQueueSend(tx_free, &i, 0);
  }
  tinyusb_enable_interface(USB_INTERFACE_VENDOR, TUD_VENDOR_DESC_LEN, load_desc);
  USB.VID(vid);
  USB.PID(pid);
  USB.productName(product);
  USB.manufacturerName("S3Drive");
  USB.usbClass(0);              // single-function device, class from the interface (0xFF)
  USB.usbVersion(0x0210);       // >= 2.1 so Windows asks for the BOS descriptor
  USB.firmwareVersion(0x0105);  // bump if descriptors change: Windows caches MS OS info per bcdDevice
  USB.begin();
}

bool usbio_aborted() { return aborted || !tud_mounted(); }

static void release_rx() {
  if (rd.idx != NOBUF) {
    xQueueSend(rx_free, &rd.idx, 0);
    usbd_defer_func(arm_rx, nullptr, false);
  }
  rd_have = false;
}

size_t usbio_read(void *buf, size_t len, uint32_t timeout_ms) {
  uint8_t *p = (uint8_t *)buf;
  size_t got = 0;
  while (got < len && !aborted) {
    if (!rd_have) {
      if (xQueueReceive(rx_full, &rd, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) break;
      rd_have = true;
      rd_off = 0;
      if (rd.epoch != epoch || rd.len == 0) { release_rx(); continue; }
    }
    size_t k = rd.len - rd_off;
    if (k > len - got) k = len - got;
    memcpy(p + got, rxb[rd.idx] + rd_off, k);
    got += k;
    rd_off += k;
    if (rd_off >= rd.len) release_rx();
  }
  return got;
}

static void submit(uint8_t idx, uint16_t len) {
  Xfer x = {idx, 0, len};
  xQueueSend(tx_full, &x, portMAX_DELAY);
  usbd_defer_func(kick_tx, nullptr, false);
  last_full = len % EP_SZ == 0;
}

size_t usbio_write(const void *buf, size_t len, uint32_t timeout_ms) {
  const uint8_t *p = (const uint8_t *)buf;
  size_t done = 0;
  msg_open = true;
  while (done < len && !aborted) {
    if (!wr_have) {
      if (xQueueReceive(tx_free, &wr_idx, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) break;
      wr_have = true;
      wr_len = 0;
    }
    size_t k = XF - wr_len;
    if (k > len - done) k = len - done;
    memcpy(txb[wr_idx] + wr_len, p + done, k);
    wr_len += k;
    done += k;
    if (wr_len == XF) {
      submit(wr_idx, wr_len);
      wr_have = false;
    }
  }
  return done;
}

void usbio_flush() {
  if (!msg_open || aborted) return;
  if (wr_have && wr_len) {
    submit(wr_idx, wr_len);
    wr_have = false;
  }
  if (last_full) submit(NOBUF, 0);  // ZLP terminates the host's read
  msg_open = false;
}

void usbio_reset() {
  if (rd_have) release_rx();
  if (wr_have) xQueueSend(tx_free, &wr_idx, 0);
  wr_have = msg_open = false;
  aborted = false;
}
