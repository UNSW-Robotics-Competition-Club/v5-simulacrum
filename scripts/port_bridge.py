#!/usr/bin/env python3
"""Cross-wire a smart port of one sim brain to a smart port of another, like a
smart cable between two V5 brains. QEMU listens on each port's unix socket; this
connects as a client to both and shuttles raw bytes between them.

  port_bridge.py /tmp/v5simA/v5portX.sock /tmp/v5simB/v5portY.sock [--trace]
"""
from __future__ import annotations

import argparse
import selectors
import socket
import sys
import time


def connect(path: str, timeout: float = 20.0) -> socket.socket:
    end = time.time() + timeout
    while True:
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.connect(path)
            s.setblocking(False)
            return s
        except (FileNotFoundError, ConnectionRefusedError):
            if time.time() > end:
                raise
            time.sleep(0.1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--trace", action="store_true")
    args = ap.parse_args()

    a, b = connect(args.a), connect(args.b)
    sel = selectors.DefaultSelector()
    sel.register(a, selectors.EVENT_READ, b)
    sel.register(b, selectors.EVENT_READ, a)
    print(f"bridge up: {args.a} <-> {args.b}", flush=True)
    while True:
        for key, _ in sel.select(timeout=1.0):
            src, dst = key.fileobj, key.data
            try:
                data = src.recv(4096)
            except BlockingIOError:
                continue
            if not data:
                print("bridge: peer closed", flush=True)
                return 0
            if args.trace:
                tag = "A->B" if src is a else "B->A"
                print(f"{tag} {data.hex()}", flush=True)
            try:
                dst.sendall(data)
            except OSError:
                return 0


if __name__ == "__main__":
    sys.exit(main())
