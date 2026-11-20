#!/usr/bin/env python3
"""Debug a v5gdb program on the sim brain from GDB: the bridge between the
brain's USB user port and a GDB client, doing what v5gdb's host CLI does on
a real brain (it needs the brain on USB, which the sim may not be).

v5gdb (github.com/vexide/v5gdb, a debugger built into a vexide or PROS
program) multiplexes the user port: brain -> PC is COBS-encoded packets,
0x00 between them, each starting with a channel byte:
  d  GDB remote protocol, to the GDB client
  l  v5gdb's own log (UTF-8), to stderr
  u  the program's output, to the output clients
Anything that isn't such a packet is program output too. PC -> brain is
raw bytes, all of them taken as GDB's (v5gdb's Transports page).

  python3 scripts/v5gdb_bridge.py BRAIN_SOCK DOWNSTREAM_SOCK GDB_PORT

  BRAIN_SOCK       QEMU's user port socket ($T/v5usb-user.sock)
  DOWNSTREAM_SOCK  served here for the usual host side (--usb pty, --usbip):
                   it gets the brain's bytes unchanged, and its bytes reach
                   the brain while no GDB client is connected
  GDB_PORT         GDB connects to 127.0.0.1:GDB_PORT (target remote)
  GDB_PORT + 1     the program's output as plain text, for a terminal

PROS output inside 'u' packets (PROS frames its stdout itself: COBS with a
4-byte stream id, sout/serr) is unwrapped to text too.
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
MAX_FRAME = 2048                 # v5gdb's packets never encode larger
IDLE_FLUSH_S = 0.25              # bytes with no delimiter for this long: plain output
PROS_STREAMS = (b"sout", b"serr")


def cobs_decode(data: bytes) -> bytes | None:
    """One COBS-encoded frame (without its 0x00), or None if invalid."""
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        end = i + code
        if code == 0 or end > n:
            return None
        block = data[i + 1:end]
        if 0 in block:
            return None
        out += block
        i = end
        if code != 0xFF and i < n:                 # a zero between blocks
            out.append(0)
    return bytes(out)


def cobs_encode(data: bytes) -> bytes:
    out, block = bytearray(), bytearray()
    for b in data:
        if b == 0:
            out += bytes([len(block) + 1]) + block
            block.clear()
        else:
            block.append(b)
            if len(block) == 0xFE:
                out += bytes([0xFF]) + block
                block.clear()
    out += bytes([len(block) + 1]) + block
    return bytes(out)


def pros_text(data: bytes) -> bytes:
    """PROS's own stdout frames (COBS, 4-byte stream id) as their text;
    anything else unchanged."""
    if 0 not in data:
        return data
    out = bytearray()
    for part in data.split(b"\0"):
        if not part:
            continue
        f = cobs_decode(part)
        if f is not None and f[:4] in PROS_STREAMS:
            out += f[4:]
        elif f is None or f[:4] not in (b"kdbg", b"jinx"):
            out += part
    return bytes(out)


class Demux:
    """Splits the brain's user-port stream into v5gdb's channels."""

    def __init__(self):
        self.buf = bytearray()
        self.last = time.monotonic()

    def feed(self, data: bytes, now: float) -> list[tuple[str, bytes]]:
        """(channel, bytes) for each complete packet: 'd', 'l', or 'u'
        (program output, also for bytes that aren't a packet)."""
        self.buf += data
        self.last = now
        out: list[tuple[str, bytes]] = []
        while True:
            end = self.buf.find(0)
            if end < 0:
                break
            frame = bytes(self.buf[:end])
            del self.buf[:end + 1]
            if frame:
                out.append(self.packet(frame))
        if len(self.buf) > MAX_FRAME:              # no delimiter coming: plain output
            out.append(("u", bytes(self.buf)))
            self.buf.clear()
        return out

    def idle(self, now: float) -> list[tuple[str, bytes]]:
        """Bytes left without a delimiter for a while: plain output (a
        program printing without v5gdb)."""
        if self.buf and now - self.last >= IDLE_FLUSH_S:
            data = bytes(self.buf)
            self.buf.clear()
            return [("u", data)]
        return []

    @staticmethod
    def packet(frame: bytes) -> tuple[str, bytes]:
        p = cobs_decode(frame)
        if p and chr(p[0]) in "dlu":
            return chr(p[0]), (pros_text(p[1:]) if p[0] == ord("u") else p[1:])
        return "u", frame                          # not a packet: program output as it is


def listen_unix(path: str) -> socket.socket:
    if os.path.lexists(path):
        os.unlink(path)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.bind(path)
    s.listen(1)
    return s


def listen_tcp(port: int) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(4)
    return s


def send(s: socket.socket | None, data: bytes) -> socket.socket | None:
    if not s:
        return None
    try:
        s.sendall(data)
        return s
    except OSError:
        s.close()
        return None


class Bridge:
    def __init__(self, brain_path: str, downstream_path: str, gdb_port: int):
        self.brain_path, self.downstream_path = brain_path, downstream_path
        self.down_listener = listen_unix(downstream_path)
        self.gdb_listener = listen_tcp(gdb_port)
        self.out_listener = listen_tcp(gdb_port + 1)
        self.brain: socket.socket | None = None
        self.down: socket.socket | None = None
        self.gdb: socket.socket | None = None
        self.outs: list[socket.socket] = []
        self.demux = Demux()
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

    def sockets(self) -> list[socket.socket]:
        return [s for s in (self.down_listener, self.gdb_listener, self.out_listener,
                            self.brain, self.down, self.gdb, *self.outs) if s]

    def route(self, packets: list[tuple[str, bytes]]) -> None:
        for channel, data in packets:
            if channel == "d":
                self.gdb = send(self.gdb, data)
            elif channel == "l":
                print(f"[v5gdb] {data.decode(errors='replace').rstrip()}", file=sys.stderr, flush=True)
            else:
                self.outs = [s for s in (send(o, data) for o in self.outs) if s]

    def on_ready(self, ready: list[socket.socket], now: float) -> None:
        if self.down_listener in ready:
            if self.down:
                self.down.close()
            self.down, _ = self.down_listener.accept()
            self.down.settimeout(SEND_TIMEOUT_S)
        if self.gdb_listener in ready:
            if self.gdb:
                self.gdb.close()                     # a new GDB replaces a stale one
            self.gdb, _ = self.gdb_listener.accept()
            self.gdb.settimeout(SEND_TIMEOUT_S)
            print("[v5gdb] GDB connected", file=sys.stderr, flush=True)
        if self.out_listener in ready:
            o, _ = self.out_listener.accept()
            o.settimeout(SEND_TIMEOUT_S)
            self.outs.append(o)
        if self.brain and self.brain in ready:
            data = recv(self.brain)
            if not data:
                self.brain.close()
                self.brain = None
            else:
                self.down = send(self.down, data)    # the usual host side sees it unchanged
                self.route(self.demux.feed(data, now))
        if self.gdb and self.gdb in ready:
            data = recv(self.gdb)
            if not data:
                self.gdb.close()
                self.gdb = None
                print("[v5gdb] GDB disconnected", file=sys.stderr, flush=True)
            elif self.brain:
                self.brain = send(self.brain, data)
        if self.down and self.down in ready:
            data = recv(self.down)
            if not data:
                self.down.close()
                self.down = None
            elif self.brain and not self.gdb:        # v5gdb would read it as GDB's
                self.brain = send(self.brain, data)
        for o in [o for o in self.outs if o in ready]:
            if not recv(o):                          # output only: input is ignored
                o.close()
                self.outs.remove(o)
        self.route(self.demux.idle(now))

    def close(self) -> None:
        for s in self.sockets():
            s.close()
        if os.path.lexists(self.downstream_path):
            os.unlink(self.downstream_path)


def recv(s: socket.socket) -> bytes:
    try:
        return s.recv(CHUNK)
    except OSError:
        return b""


def main(argv: list[str]) -> None:
    if len(argv) != 3 or not argv[2].isdigit():
        sys.exit(__doc__)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    bridge = Bridge(argv[0], argv[1], int(argv[2]))
    print(f"[v5gdb] GDB on 127.0.0.1:{argv[2]}, program output on 127.0.0.1:{int(argv[2]) + 1}",
          file=sys.stderr, flush=True)
    try:
        while True:
            bridge.ensure_brain()
            ready, _, _ = select.select(bridge.sockets(), [], [], IDLE_FLUSH_S / 2)
            bridge.on_ready(ready, time.monotonic())
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()


if __name__ == "__main__":
    main(sys.argv[1:])
