# S3Drive

Use an ESP32-S3's flash as fast USB file storage from Windows: firmware for the board plus a small tkinter app with upload/download progress.

- Download ~990 KB/s, upload ~410 KB/s (full-speed USB, 16 MB flash board → ~13.7 MB usable)
- No driver install: the board enumerates as a WinUSB device (MS OS 2.0 descriptors)
- CRC-checked transfers; power-loss-safe metadata; files only appear once fully received

## Use (any Windows 8.1+ PC)

```
pip install pyusb libusb-package
python s3drive.py
```

Plug in the board's native **USB** port. The window connects automatically.

## Build and flash the firmware

Tested on an ESP32-S3 N16R8 (16 MB flash, 8 MB PSRAM) with PlatformIO. Flash through the **UART** port (set `upload_port` in `platformio.ini`):

```
pio run -t upload
```

## Layout

| File | Purpose |
|---|---|
| `s3drive.py` | Windows app and protocol client (`S3Drive` class) |
| `src/usbio.cpp` | Custom TinyUSB class driver: multi-packet bulk transfers, WinUSB descriptors |
| `src/fs.cpp` | Extent-based file store: background pre-erase, append-only metadata log |
| `src/main.cpp` | Wire protocol and background erase task |
| `partitions.csv` | 2 MB app, 14 MB storage |

## Notes

- The ESP32-S3 OTG packet counter is 7 bits, so bulk transfers are capped at 127 × 64 bytes.
- The stock Arduino `USBVendor` class moves one 64-byte packet per re-arm (~500 KB/s); a custom class driver roughly doubles that.
- Uploads are fastest after the board has been idle briefly; free space is pre-erased in the background.
