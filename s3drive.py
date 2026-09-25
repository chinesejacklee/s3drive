"""S3Drive - use an ESP32-S3's flash as fast USB file storage.

Plug the board's native "USB" port into the PC; the window connects automatically
(Windows binds WinUSB by itself - no driver install). Requires:
    pip install pyusb libusb-package
"""
import os
import queue
import struct
import threading
import time
import zlib
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import libusb_package
import usb.core
import usb.util

VID, PID = 0x303A, 0x4DD0
REQ_ABORT = 0x41
WCHUNK = 1 << 16  # bytes per USB write (also the progress-bar step)
RCHUNK = 1 << 16  # bytes per USB read (multiple of 64)
ERRORS = {1: "not found", 2: "not enough space", 3: "CRC mismatch", 4: "flash I/O error",
          5: "bad request", 6: "too many files or too fragmented"}


class DriveError(Exception):
    pass


class S3Drive:
    """Protocol client; see src/main.cpp for the wire format. Not thread-safe."""

    def __init__(self):
        # Pinned libusb DLL: this PC has several conflicting libusb builds on PATH.
        self.dev = usb.core.find(idVendor=VID, idProduct=PID, backend=libusb_package.get_libusb1_backend())
        if self.dev is None:
            raise DriveError("not connected")
        self.dev.set_configuration()
        intf = self.dev.get_active_configuration()[(0, 0)]
        eps = {usb.util.endpoint_direction(e.bEndpointAddress): e.bEndpointAddress for e in intf}
        self.ep_out, self.ep_in = eps[usb.util.ENDPOINT_OUT], eps[usb.util.ENDPOINT_IN]
        self.rtmp = usb.util.create_buffer(RCHUNK)
        self.rbuf = bytearray()
        self.resync()

    def close(self):
        usb.util.dispose_resources(self.dev)

    def resync(self):
        """Abort whatever the device was doing and drop stale data in both directions."""
        self.dev.ctrl_transfer(0x40, REQ_ABORT, 0, 0, None, 1000)
        self.dev.write(self.ep_out, b"", 1000)  # completes a half-filled device transfer
        while True:
            try:
                self.dev.read(self.ep_in, self.rtmp, 150)
            except usb.core.USBTimeoutError:
                break
        self.rbuf.clear()
        self.info()

    # -- framing: every message that ends on a 64-byte boundary is closed by a ZLP --
    def _send(self, data, timeout=5000):
        self.dev.write(self.ep_out, data, timeout)
        if len(data) % 64 == 0:
            self.dev.write(self.ep_out, b"", timeout)

    def _fill(self, timeout):
        n = self.dev.read(self.ep_in, self.rtmp, timeout)
        self.rbuf += memoryview(self.rtmp)[:n]

    def _read(self, n, timeout=5000):
        while len(self.rbuf) < n:
            self._fill(timeout)
        out = bytes(self.rbuf[:n])
        del self.rbuf[:n]
        return out

    def _request(self, op, name="", size=0, crc=0, mtime=0, timeout=5000):
        nb = name.encode("utf-8")
        self._send(struct.pack("<cBHIII", op, len(nb), 0, size, crc, mtime) + nb)
        return self._response(timeout)

    def _response(self, timeout=5000):
        status, length = struct.unpack("<B3xI", self._read(8, timeout))
        if status:
            raise DriveError(ERRORS.get(status, f"error {status}"))
        return length

    # -- API --
    def info(self):
        total, free, ready, nfiles, ver = struct.unpack("<5I", self._read(self._request(b"I")))
        return dict(total=total, free=free, ready=ready, files=nfiles, version=ver)

    def list(self):
        data, out, i = self._read(self._request(b"L")), [], 0
        while i < len(data):
            size, crc, mtime, nlen = struct.unpack_from("<3IB", data, i)
            i += 13
            out.append((data[i:i + nlen].decode("utf-8", "replace"), size, mtime))
            i += nlen
        return out

    def put(self, path, name, progress=lambda n: None):
        size, mtime, crc = os.path.getsize(path), int(os.path.getmtime(path)), 0
        with open(path, "rb") as f:
            for block in iter(lambda: f.read(1 << 22), b""):
                crc = zlib.crc32(block, crc)
            self._request(b"P", name, size, crc, mtime)
            f.seek(0)
            sent = 0
            while sent < size:
                block = f.read(WCHUNK)
                self.dev.write(self.ep_out, block, 30000)
                sent += len(block)
                progress(len(block))
        if size % 64 == 0:
            self.dev.write(self.ep_out, b"", 5000)
        self._response(60000)  # CRC verified on the device before the file is committed

    def get(self, name, dest, mtime=None, progress=lambda n: None):
        size = self._request(b"G", name) - 4
        want, crc, left = struct.unpack("<I", self._read(4))[0], 0, size
        tmp = dest + ".part"
        try:
            with open(tmp, "wb") as f:
                while left:
                    if not self.rbuf:
                        self._fill(10000)
                    k = min(left, len(self.rbuf))
                    chunk = self.rbuf[:k]
                    del self.rbuf[:k]
                    f.write(chunk)
                    crc = zlib.crc32(chunk, crc)
                    left -= k
                    progress(k)
            if crc != want:
                raise DriveError("CRC mismatch on download")
            os.replace(tmp, dest)
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)
        if mtime:
            os.utime(dest, (mtime, mtime))

    def delete(self, name):
        self._request(b"D", name)

    def format(self):
        self._request(b"F", timeout=15000)


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024


class App:
    def __init__(self, root):
        self.root, self.drive, self.files = root, None, {}
        self.jobs, self.ui = queue.Queue(), queue.Queue()
        root.title("S3Drive")
        root.geometry("680x440")
        root.minsize(480, 260)

        bar = ttk.Frame(root, padding=(6, 6, 6, 0))
        bar.pack(fill="x")
        self.buttons = [ttk.Button(bar, text=t, command=c) for t, c in (
            ("Upload…", self.on_upload), ("Download…", self.on_download), ("Delete", self.on_delete),
            ("Refresh", lambda: self.jobs.put(self.job_refresh)), ("Format…", self.on_format))]
        for b in self.buttons:
            b.pack(side="left", padx=(0, 4))
        self.conn = ttk.Label(bar, text="Waiting for device…", foreground="#a60")
        self.conn.pack(side="right")

        body = ttk.Frame(root, padding=6)
        body.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(body, columns=("name", "size", "date"), show="headings", selectmode="extended")
        self.tree.heading("name", text="Name", anchor="w")
        self.tree.heading("size", text="Size", anchor="e")
        self.tree.heading("date", text="Modified", anchor="w")
        self.tree.column("name", width=360)
        self.tree.column("size", width=90, anchor="e", stretch=False)
        self.tree.column("date", width=140, stretch=False)
        sb = ttk.Scrollbar(body, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.tree.bind("<Double-1>", lambda e: self.on_download())

        foot = ttk.Frame(root, padding=(6, 0, 6, 6))
        foot.pack(fill="x")
        self.progress = ttk.Progressbar(foot, mode="determinate", maximum=1)
        self.progress.pack(fill="x")
        row = ttk.Frame(foot)
        row.pack(fill="x")
        self.status = ttk.Label(row, text="")
        self.status.pack(side="left")
        self.space = ttk.Label(row, text="")
        self.space.pack(side="right")

        threading.Thread(target=self.worker, daemon=True).start()
        self.pump()

    # -- thread plumbing: device I/O only on the worker thread, Tk only on the main thread --
    def post(self, fn, *args):
        self.ui.put((fn, args))

    def pump(self):
        try:
            while True:
                fn, args = self.ui.get_nowait()
                fn(*args)
        except queue.Empty:
            pass
        self.root.after(40, self.pump)

    def worker(self):
        last_ping = 0
        while True:
            if self.drive is None:
                try:
                    self.drive = S3Drive()
                    self.post(self.conn.config, {"text": "● Connected", "foreground": "#080"})
                    self.job_refresh()
                except Exception:
                    time.sleep(1)
                continue
            try:
                job = self.jobs.get(timeout=0.5)
            except queue.Empty:
                job = None
            try:
                if job:
                    job()
                elif time.time() - last_ping > 2:  # heartbeat: detects unplug, updates free space
                    self.show_space(self.drive.info())
                last_ping = time.time()
            except usb.core.USBError as e:
                if not isinstance(e, usb.core.USBTimeoutError):
                    self.disconnected()  # unplugged; the loop reconnects if it is still there
                    continue
                self.recover(e)
            except Exception as e:  # device-side error or local file problem
                self.recover(e)

    def recover(self, err):
        self.post(self.set_status, f"Error: {err}")
        try:
            self.drive.resync()
            self.job_refresh()
        except Exception:
            self.disconnected()

    def disconnected(self):
        try:
            self.drive.close()
        except Exception:
            pass
        self.drive = None
        self.post(self.conn.config, {"text": "Waiting for device…", "foreground": "#a60"})
        self.post(self.show_files, [])
        self.post(self.space.config, {"text": ""})

    # -- jobs (worker thread) --
    def job_refresh(self):
        files = self.drive.list()
        self.post(self.show_files, files)
        self.show_space(self.drive.info())

    def show_space(self, info):
        self.post(self.space.config, {"text": f"{human(info['free'])} free of {human(info['total'])}"})

    def transfer(self, verb, items, total):
        """items: list of (label, callable(progress)). Drives the progress bar across the batch."""
        done, t0, last = 0, time.perf_counter(), 0.0

        def progress(n):
            nonlocal done, last
            done += n
            now = time.perf_counter()
            if now - last > 0.05 or done == total:
                last = now
                rate = done / max(now - t0, 1e-6)
                eta = (total - done) / rate if rate else 0
                self.post(self.set_progress, done / max(total, 1),
                          f"{verb} {label} ({i}/{len(items)}) · {human(done)} / {human(total)} · "
                          f"{human(rate)}/s · {eta:.0f} s left")

        for i, (label, run) in enumerate(items, 1):
            progress(0)
            run(progress)
        dt = time.perf_counter() - t0
        self.post(self.set_progress, 1, f"{verb[:-3]}ed {len(items)} file(s), {human(total)} in {dt:.1f} s "
                                        f"({human(total / max(dt, 1e-6))}/s)")
        self.job_refresh()

    # -- UI actions (main thread) --
    def selected(self):
        return [self.tree.item(i, "text") for i in self.tree.selection()]

    def on_upload(self):
        if not self.drive:
            return
        paths = filedialog.askopenfilenames(title="Upload files to S3Drive")
        if not paths:
            return
        clash = [os.path.basename(p) for p in paths if os.path.basename(p) in self.files]
        if clash and not messagebox.askyesno("Overwrite?", f"Replace {len(clash)} existing file(s)?\n" + "\n".join(clash[:10])):
            return
        items = [(os.path.basename(p), lambda cb, p=p: self.drive.put(p, os.path.basename(p), cb)) for p in paths]
        total = sum(os.path.getsize(p) for p in paths)
        self.jobs.put(lambda: self.transfer("Uploading", items, total))

    def on_download(self):
        names = self.selected()
        if not self.drive or not names:
            return
        folder = filedialog.askdirectory(title="Download to folder")
        if not folder:
            return
        items = [(n, lambda cb, n=n: self.drive.get(n, os.path.join(folder, os.path.basename(n)), self.files[n][1], cb))
                 for n in names]
        total = sum(self.files[n][0] for n in names)
        self.jobs.put(lambda: self.transfer("Downloading", items, total))

    def on_delete(self):
        names = self.selected()
        if self.drive and names and messagebox.askyesno("Delete", f"Delete {len(names)} file(s) from the device?"):
            def job():
                for n in names:
                    self.drive.delete(n)
                self.post(self.set_status, f"Deleted {len(names)} file(s)")
                self.job_refresh()
            self.jobs.put(job)

    def on_format(self):
        if self.drive and messagebox.askyesno("Format", "Erase ALL files on the device?", icon="warning"):
            def job():
                self.drive.format()
                self.post(self.set_status, "Formatted")
                self.job_refresh()
            self.jobs.put(job)

    def show_files(self, files):
        self.files = {n: (s, m) for n, s, m in files}
        self.tree.delete(*self.tree.get_children())
        for n, s, m in sorted(files, key=lambda f: f[0].lower()):
            self.tree.insert("", "end", text=n, values=(n, human(s), time.strftime("%Y-%m-%d %H:%M", time.localtime(m))))

    def set_progress(self, frac, text):
        self.progress["value"] = frac
        self.status.config(text=text)

    def set_status(self, text):
        self.status.config(text=text)


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
