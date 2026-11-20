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

Reconnects when QEMU restarts. Sends are buffered (v5sock.py): a slow
reader never stalls the others. V5USB_TRACE=FILE logs every chunk.
"""
from __future__ import annotations

import os
import select
import signal
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # -I leaves it out
from v5sock import Peer, close  # noqa: E402

RETRY_S = 0.5
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
        self.brain: Peer | None = None
        self.shared: Peer | None = None
        self.host: Peer | None = None
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
        self.brain = Peer(s)
        print(f"share: {self.brain_path} connected", flush=True)

    def peers(self) -> list[Peer]:
        return [p for p in (self.brain, self.shared, self.host) if p]

    def readable(self) -> list:
        return [self.shared_listener, self.host_listener, *self.peers()]

    def writable(self) -> list[Peer]:
        return [p for p in self.peers() if p.wants_write]

    def to_brain(self, data: bytes) -> None:
        if self.brain and not self.brain.send(data):
            self.brain = None
            print("share: brain gone, waiting", flush=True)

    def on_ready(self, ready: list, writable: list) -> None:
        for p in writable:
            p.flush()
        if self.shared_listener in ready:
            close(self.shared)
            self.shared = Peer(self.shared_listener.accept()[0])
        if self.host_listener in ready:
            close(self.host)                     # a new uploader replaces a stale one
            self.host = Peer(self.host_listener.accept()[0])
            print("share: uploader connected", flush=True)
        if self.brain and self.brain in ready:
            data = self.brain.recv()
            if data == b"":
                close(self.brain)
                self.brain = None
                print("share: brain gone, waiting", flush=True)
            elif data:
                trace("<brain", data)
                if self.host:                     # the uploader's reply, even if it just left
                    if not self.host.send(data):
                        self.host = None
                elif self.shared and not self.shared.send(data):
                    self.shared = None
        if self.shared and self.shared in ready:
            data = self.shared.recv()
            if data == b"":
                close(self.shared)
                self.shared = None
            elif data and not self.host:          # dropped while an upload has the port
                trace("shared>", data)
                self.to_brain(data)
        if self.host and self.host in ready:
            data = self.host.recv()
            if data == b"":
                close(self.host)
                self.host = None
                print("share: uploader done", flush=True)
            elif data:
                trace("host>", data)
                self.to_brain(data)
        for name in ("brain", "shared", "host"):
            p = getattr(self, name)
            if p and not p.alive:
                setattr(self, name, None)

    def close(self) -> None:
        for p in self.peers():
            p.close()
        for s in (self.shared_listener, self.host_listener):
            s.close()
        for p in self.paths:
            if os.path.lexists(p):
                os.unlink(p)


def main(argv: list[str]) -> None:
    if len(argv) != 3:
        sys.exit(__doc__)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    share = Share(*argv)
    try:
        while True:
            share.ensure_brain()
            ready, writable, _ = select.select(share.readable(), share.writable(), [], RETRY_S)
            share.on_ready(ready, writable)
    except KeyboardInterrupt:
        pass
    finally:
        share.close()


if __name__ == "__main__":
    main(sys.argv[1:])
