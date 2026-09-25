// Raw WinUSB bulk pipe: custom TinyUSB class driver doing ~8 KB multi-packet transfers.
#pragma once
#include <stddef.h>
#include <stdint.h>

void usbio_begin(uint16_t vid, uint16_t pid, const char *product);
// Read exactly len bytes; returns fewer on timeout or abort.
size_t usbio_read(void *buf, size_t len, uint32_t timeout_ms);
// Buffer len bytes for the host (sent in ~8 KB transfers); returns fewer on timeout or abort.
size_t usbio_write(const void *buf, size_t len, uint32_t timeout_ms = 3000);
// End of a response: send the partial buffer and a ZLP if needed, so the host's read returns.
void usbio_flush();
// True after the host's ABORT request, a bus reset or unplug; cleared by usbio_reset().
bool usbio_aborted();
void usbio_reset();
