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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # -I leaves it out
from v5sock import Peer, close  # noqa: E402

RETRY_S = 0.5
MAX_FRAME = 2048                 # v5gdb's packets never encode larger
IDLE_FLUSH_S = 0.5               # bytes with no delimiter for this long: plain output
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
                out += self.packets(frame)
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

    def reset(self) -> None:
        """The brain went away: a half-received packet won't be finished."""
        self.buf.clear()

    @staticmethod
    def packets(frame: bytes) -> list[tuple[str, bytes]]:
        """A frame as packets: one v5gdb packet; or plain output that ran
        into one (text with no 0x00 printed just before a packet), split
        where the packet starts; or plain output only."""
        for start in range(len(frame)):
            p = cobs_decode(frame[start:])
            if p and chr(p[0]) in "dlu":
                packet = (chr(p[0]), pros_text(p[1:]) if p[0] == ord("u") else p[1:])
                return ([("u", frame[:start])] if start else []) + [packet]
        return [("u", frame)]                      # not a packet: program output as it is


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


class Bridge:
    def __init__(self, brain_path: str, downstream_path: str, gdb_port: int):
        self.brain_path, self.downstream_path = brain_path, downstream_path
        self.gdb_listener = listen_tcp(gdb_port)       # first: a taken port fails before anything else
        self.out_listener = listen_tcp(gdb_port + 1)
        self.down_listener = listen_unix(downstream_path)
        self.brain: Peer | None = None
        self.down: Peer | None = None
        self.gdb: Peer | None = None
        self.outs: list[Peer] = []
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
        self.brain = Peer(s)
        self.demux.reset()

    def peers(self) -> list[Peer]:
        return [p for p in (self.brain, self.down, self.gdb, *self.outs) if p]

    def readable(self) -> list:
        return [self.down_listener, self.gdb_listener, self.out_listener, *self.peers()]

    def writable(self) -> list[Peer]:
        return [p for p in self.peers() if p.wants_write]

    def route(self, packets: list[tuple[str, bytes]]) -> None:
        for channel, data in packets:
            if channel == "d":
                if self.gdb and not self.gdb.send(data):
                    self.gdb = None
            elif channel == "l":
                print(f"[v5gdb] {data.decode(errors='replace').rstrip()}", file=sys.stderr, flush=True)
            else:
                self.outs = [o for o in self.outs if o.send(data)]

    def to_brain(self, data: bytes) -> None:
        if self.brain and not self.brain.send(data):
            self.brain = None

    def on_ready(self, ready: list, writable: list, now: float) -> None:
        for p in writable:
            p.flush()
        if self.down_listener in ready:
            close(self.down)
            self.down = Peer(self.down_listener.accept()[0])
        if self.gdb_listener in ready:
            close(self.gdb)                              # a new GDB replaces a stale one
            self.gdb = Peer(self.gdb_listener.accept()[0])
            print("[v5gdb] GDB connected", file=sys.stderr, flush=True)
        if self.out_listener in ready:
            self.outs.append(Peer(self.out_listener.accept()[0]))
        if self.brain and self.brain in ready:
            data = self.brain.recv()
            if data == b"":
                close(self.brain)
                self.brain = None
            elif data:
                if self.down and not self.down.send(data):   # the usual host side sees it unchanged
                    self.down = None
                self.route(self.demux.feed(data, now))
        if self.gdb and self.gdb in ready:
            data = self.gdb.recv()
            if data == b"":
                close(self.gdb)
                self.gdb = None
                print("[v5gdb] GDB disconnected", file=sys.stderr, flush=True)
            elif data:
                self.to_brain(data)
        if self.down and self.down in ready:
            data = self.down.recv()
            if data == b"":
                close(self.down)
                self.down = None
            elif data and not self.gdb:                  # v5gdb would read it as GDB's
                self.to_brain(data)
        for o in [o for o in self.outs if o in ready]:
            if o.recv() == b"":                          # output only: input is ignored
                o.close()
        self.outs = [o for o in self.outs if o.alive]
        self.route(self.demux.idle(now))
        for name in ("brain", "down", "gdb"):
            p = getattr(self, name)
            if p and not p.alive:
                setattr(self, name, None)

    def close(self) -> None:
        for p in self.peers():
            p.close()
        for s in (self.down_listener, self.gdb_listener, self.out_listener):
            s.close()
        if os.path.lexists(self.downstream_path):
            os.unlink(self.downstream_path)


def main(argv: list[str]) -> None:
    if len(argv) != 3 or not argv[2].isdigit() or not 1024 <= int(argv[2]) <= 65534:
        sys.exit(__doc__)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        bridge = Bridge(argv[0], argv[1], int(argv[2]))
    except OSError as e:
        print(f"[v5gdb] can't serve GDB on ports {argv[2]} and {int(argv[2]) + 1}: {e}. "
              "Another simulator or program has them: pick others with --gdb PORT.", file=sys.stderr, flush=True)
        sys.exit(1)
    print(f"[v5gdb] GDB on 127.0.0.1:{argv[2]}, program output on 127.0.0.1:{int(argv[2]) + 1}",
          file=sys.stderr, flush=True)
    try:
        while True:
            bridge.ensure_brain()
            ready, writable, _ = select.select(bridge.readable(), bridge.writable(), [], IDLE_FLUSH_S / 2)
            bridge.on_ready(ready, writable, time.monotonic())
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()


if __name__ == "__main__":
    main(sys.argv[1:])
