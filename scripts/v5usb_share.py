#!/usr/bin/env python3
"""Share the emulated brain's USB system port between its usual host side
(the pty bridge or the USB/IP server) and a program on this machine that
uploads to it directly (v5upload.py: Run on Sim without a restart).

QEMU serves the port as a unix socket with room for one client. This is that
client, and serves two sockets of its own:

  SHARED  the usual host side connects here instead of to QEMU's socket
  HOST    an uploader connects here; while it is connected it has the port
          to itself: the brain's replies go to it alone, and bytes from the
          shared side are dropped (a Windows tool polling the brain must not
          interleave with a file transfer)

  python3 scripts/v5usb_share.py /tmp/v5usb-sys.sock /tmp/v5usb-sys-shared.sock /tmp/v5usb-host.sock

Reconnects when QEMU restarts. V5USB_TRACE=FILE logs every chunk.
"""
from __future__ import annotations

import os
import select
import signal
import socket
import sys
import time

RETRY_S = 0.5
SEND_TIMEOUT_S = 1.0
CHUNK = 4096
TRACE = open(os.environ["V5USB_TRACE"], "a", buffering=1) if os.environ.get("V5USB_TRACE") else None
T0 = time.monotonic()


def trace(arrow: str, data: bytes) -> None:
    if TRACE:
        TRACE.write(f"{time.monotonic() - T0:9.3f} share {arrow} {data.hex(' ')}\n")


def listen(path: str) -> socket.socket:
    if os.path.lexists(path):
        os.unlink(path)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.bind(path)
    s.listen(1)
    return s


class Share:
    def __init__(self, brain_path: str, shared_path: str, host_path: str):
        self.brain_path, self.paths = brain_path, (shared_path, host_path)
        self.shared_listener = listen(shared_path)
        self.host_listener = listen(host_path)
        self.brain: socket.socket | None = None
        self.shared: socket.socket | None = None
        self.host: socket.socket | None = None
        self._next_try = 0.0

    def ensure_brain(self) -> None:
        if self.brain or time.monotonic() < self._next_try:
            return
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self.brain_path)
        except OSError:
            s.close()
            self._next_try = time.monotonic() + RETRY_S
            return
        s.settimeout(SEND_TIMEOUT_S)
        self.brain = s
        print(f"share: {self.brain_path} connected", flush=True)

    def sockets(self) -> list[socket.socket]:
        return [s for s in (self.shared_listener, self.host_listener, self.brain, self.shared, self.host) if s]

    def accept(self, listener: socket.socket) -> socket.socket:
        conn, _ = listener.accept()
        conn.settimeout(SEND_TIMEOUT_S)
        return conn

    def on_ready(self, ready: list[socket.socket]) -> None:
        if self.shared_listener in ready:
            close(self.shared)
            self.shared = self.accept(self.shared_listener)
        if self.host_listener in ready:
            close(self.host)                     # a new uploader replaces a stale one
            self.host = self.accept(self.host_listener)
            print("share: uploader connected", flush=True)
        if self.brain and self.brain in ready:
            data = recv(self.brain)
            if not data:
                close(self.brain)
                self.brain = None
                print("share: brain gone, waiting", flush=True)
            else:
                trace("<brain", data)
                self.host = send(self.host, data) if self.host else self.host
                if not self.host and self.shared:
                    self.shared = send(self.shared, data)
        if self.shared and self.shared in ready:
            data = recv(self.shared)
            if not data:
                close(self.shared)
                self.shared = None
            elif not self.host and self.brain:   # dropped while an upload has the port
                trace("shared>", data)
                self.brain = send(self.brain, data)
        if self.host and self.host in ready:
            data = recv(self.host)
            if not data:
                close(self.host)
                self.host = None
                print("share: uploader done", flush=True)
            elif self.brain:
                trace("host>", data)
                self.brain = send(self.brain, data)

    def close(self) -> None:
        for s in (self.brain, self.shared, self.host, self.shared_listener, self.host_listener):
            close(s)
        for p in self.paths:
            if os.path.lexists(p):
                os.unlink(p)


def recv(s: socket.socket) -> bytes:
    try:
        return s.recv(CHUNK)
    except OSError:
        return b""


def send(s: socket.socket, data: bytes) -> socket.socket | None:
    """Sends all of data; a peer that can't take it is dropped (None)."""
    try:
        s.sendall(data)
        return s
    except OSError:
        close(s)
        return None


def close(s: socket.socket | None) -> None:
    if s:
        try:
            s.close()
        except OSError:
            pass


def main(argv: list[str]) -> None:
    if len(argv) != 3:
        sys.exit(__doc__)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    share = Share(*argv)
    try:
        while True:
            share.ensure_brain()
            ready, _, _ = select.select(share.sockets(), [], [], RETRY_S)
            share.on_ready(ready)
    except KeyboardInterrupt:
        pass
    finally:
        share.close()


if __name__ == "__main__":
    main(sys.argv[1:])
