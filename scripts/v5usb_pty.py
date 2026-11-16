#!/usr/bin/env python3
"""Expose the emulated brain's USB serial ports as Linux ptys.

QEMU's USB model serves each CDC port as a unix-socket chardev; host tools
(PROS CLI, pyserial) want a tty. For each SOCK=LINK pair this opens a pty,
links LINK to its slave side and relays bytes both ways, reconnecting when
QEMU restarts:

  python3 scripts/v5usb_pty.py /tmp/v5usb-sys.sock=/tmp/ttyV5sys \\
                               /tmp/v5usb-user.sock=/tmp/ttyV5user
  pros upload --target v5 --port /tmp/ttyV5sys ...

V5USB_TRACE=FILE logs every chunk (time, link, direction, hex) to FILE.
"""
from __future__ import annotations

import os
import select
import signal
import socket
import sys
import time
import tty

RETRY_S = 0.5
SEND_TIMEOUT_S = 1.0          # QEMU not taking host bytes: drop them
TRACE = open(os.environ["V5USB_TRACE"], "a", buffering=1) if os.environ.get("V5USB_TRACE") else None
T0 = time.monotonic()


def trace(link: str, arrow: str, data: bytes) -> None:
    if TRACE:
        TRACE.write(f"{time.monotonic() - T0:9.3f} {link} {arrow} {data.hex(' ')}\n")


class Bridge:
    def __init__(self, sock_path: str, link: str):
        self.sock_path, self.link = sock_path, link
        self.master, slave = os.openpty()
        tty.setraw(slave)                 # no echo / line editing on the tty
        self._slave = slave               # keep open so reads don't EIO
        # Nobody reading the tty must not stall the other links: when its
        # buffer is full, brain output for it is dropped (to_host).
        os.set_blocking(self.master, False)
        if os.path.lexists(link):
            os.unlink(link)
        os.symlink(os.ttyname(slave), link)
        self.sock: socket.socket | None = None
        self._next_try = 0.0

    def ensure_connected(self) -> None:
        if self.sock or time.monotonic() < self._next_try:
            return
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self.sock_path)
        except OSError:
            s.close()
            self._next_try = time.monotonic() + RETRY_S
            return
        s.settimeout(SEND_TIMEOUT_S)
        self.sock = s
        print(f"{self.link} <-> {self.sock_path}", flush=True)

    def to_brain(self, data: bytes) -> None:
        if not self.sock:
            return                        # bytes written with no brain are dropped
        try:
            self.sock.sendall(data)
        except TimeoutError:
            trace(self.link, "drop>", data)
        except OSError:
            self.drop()

    def to_host(self, data: bytes) -> None:
        while data:
            try:
                data = data[os.write(self.master, data):]
            except BlockingIOError:
                trace(self.link, "<drop", data)
                return

    def drop(self) -> None:
        if self.sock:
            self.sock.close()
            self.sock = None
            print(f"{self.link}: brain gone, waiting", flush=True)

    def close(self) -> None:
        self.drop()
        if os.path.islink(self.link):
            os.unlink(self.link)


def main(argv: list[str]) -> None:
    if not argv or any("=" not in a for a in argv):
        sys.exit(__doc__)
    # SIGTERM (v5brain.sh's cleanup) runs the finally below: no stale
    # links left pointing at a /dev/pts another session may reuse.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    bridges = [Bridge(*a.split("=", 1)) for a in argv]
    try:
        while True:
            for b in bridges:
                b.ensure_connected()
            fds = [b.master for b in bridges] + [b.sock for b in bridges if b.sock]
            ready, _, _ = select.select(fds, [], [], RETRY_S)
            for b in bridges:
                if b.master in ready:
                    try:
                        data = os.read(b.master, 4096)
                    except BlockingIOError:
                        data = b""
                    trace(b.link, "host>", data)
                    b.to_brain(data)
                if b.sock and b.sock in ready:
                    try:
                        data = b.sock.recv(4096)
                    except OSError:
                        data = b""
                    trace(b.link, "<brain", data)
                    if data:
                        b.to_host(data)
                    else:
                        b.drop()
    except KeyboardInterrupt:
        pass
    finally:
        for b in bridges:
            b.close()


if __name__ == "__main__":
    main(sys.argv[1:])
