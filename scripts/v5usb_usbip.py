"""The emulated brain's USB port as a USB/IP device, for tools on Windows.

  python3 v5usb_usbip.py SYS_SOCK USER_SOCK [--port 3300] [--serial V5SIM0]

Windows (usbip-win2: `usbip attach -r localhost -b 1-1`) then sees a real V5
brain on USB: VID 2888 / PID 0501, two CDC-ACM functions, so its inbox
serial driver makes two COM ports (interface MI_00 = system, MI_02 = user)
and VEXcode and PROS find the brain like a plugged-in one. Unlike VEXos it
reports a USB serial number (--serial, V5SIM<instance> from v5brain.sh), so
Windows tells simulator instances apart and V5Upload.cmd can name them.

The descriptors and control replies are VEXos's own (notes/usb-protocol.md
4.3, 5); the bulk data of the system port (EP3 OUT / EP2 IN) and the user
port (EP6 OUT / EP5 IN) goes to and from QEMU's CDC chardevs SYS_SOCK and
USER_SOCK, the sockets scripts/v5usb_pty.py otherwise turns into ptys. The
notify endpoints (EP1, EP4 IN) never complete, as on the brain.

USB/IP 1.1.1, one client at a time, localhost only. Everything on the wire
is big-endian.
"""
from __future__ import annotations

import argparse
import asyncio
import struct
import sys

VERSION = 0x0111
OP_REQ_DEVLIST, OP_REP_DEVLIST = 0x8005, 0x0005
OP_REQ_IMPORT, OP_REP_IMPORT = 0x8003, 0x0003
CMD_SUBMIT, CMD_UNLINK, RET_SUBMIT, RET_UNLINK = 1, 2, 3, 4
DIR_OUT, DIR_IN = 0, 1
EPIPE, ECONNRESET = -32, -104
SPEED_HIGH = 3
BUSID = b"1-1"
BUSNUM, DEVNUM = 1, 1

DEVICE = bytes.fromhex("12 01 00 02 EF 02 01 40 88 28 01 05 00 01 01 02 03 01")
QUALIFIER = bytes.fromhex("0A 06 00 02 EF 02 01 40 01 00")
CONFIG = bytes.fromhex(
    "09 02 8D 00 04 01 04 C0 00"
    "08 0B 00 02 02 02 00 00"
    "09 04 00 00 01 02 02 00 06"
    "05 24 00 10 01 05 24 01 01 01 04 24 02 07 05 24 06 00 01"
    "07 05 81 03 08 00 04"
    "09 04 01 00 02 0A 00 00 00"
    "07 05 03 02 00 02 00"
    "07 05 82 02 00 02 00"
    "08 0B 02 02 02 02 00 00"
    "09 04 02 00 01 02 02 00 07"
    "05 24 00 10 01 05 24 01 01 03 04 24 02 07 05 24 06 02 03"
    "07 05 84 03 08 00 04"
    "09 04 03 00 02 0A 00 00 00"
    "07 05 06 02 00 02 00"
    "07 05 85 02 00 02 00")
INTERFACES = ((0x02, 0x02, 0x00), (0x0A, 0x00, 0x00), (0x02, 0x02, 0x00), (0x0A, 0x00, 0x00))
STRINGS = {1: "VEX Robotics, Inc", 2: "VEX Robotics V5 Brain - 00000000", 3: "V5SIM0",
           4: "Default Configuration", 5: "Default Interface",
           6: "VEX Robotics Communications Port", 7: "VEX Robotics User Port",
           8: "VEX Robotics Debug Port"}
LINE_CODING = bytes.fromhex("00 C2 01 00 00 00 08")         # 115200 8N1
BULK_IN = {2: "sys", 5: "user"}
BULK_OUT = {3: "sys", 6: "user"}


def string_descriptor(index: int) -> bytes | None:
    if index == 0:
        return bytes([4, 3, 0x09, 0x04])     # LANGID en-US (VEXos's own is a quirk)
    text = STRINGS.get(index)
    if text is None:
        return None
    data = text.encode("utf-16-le")
    return bytes([2 + len(data), 3]) + data


def control(setup: bytes, data: bytes) -> tuple[int, bytes]:
    """(status, IN data) for an EP0 request, as usbHandleControlRequest."""
    req_type, req, value, index, length = struct.unpack("<BBHHH", setup)
    kind = req_type & 0x60
    if kind == 0x00:                                  # standard
        if req == 6:                                  # GET_DESCRIPTOR
            dtype, didx = value >> 8, value & 0xFF
            desc = {1: DEVICE, 2: CONFIG, 6: QUALIFIER}.get(dtype)
            if dtype == 3:
                desc = string_descriptor(didx)
            return (0, desc[:length]) if desc is not None else (EPIPE, b"")
        if req == 0:                                  # GET_STATUS
            return 0, (b"\x01\x00" if req_type & 0x1F == 0 else b"\x00\x00")[:length]
        if req == 8:                                  # GET_CONFIGURATION
            return 0, b"\x01"[:length]
        if req == 10:                                 # GET_INTERFACE
            return 0, b"\x00"[:length]
        if req in (1, 3, 5, 9, 11):                   # features, address, config, iface
            return 0, b""
        return EPIPE, b""
    if kind == 0x20:                                  # class: CDC-ACM
        if req == 0x21:                               # GET_LINE_CODING
            return 0, LINE_CODING[:length]
        return 0, b""                                 # SET_LINE_CODING, control lines, ...
    return EPIPE, b""                                 # vendor: no known host uses it


def device_record() -> bytes:
    path = b"/sys/devices/vex/v5brain".ljust(256, b"\0")
    busid = BUSID.ljust(32, b"\0")
    return (path + busid + struct.pack(">III", BUSNUM, DEVNUM, SPEED_HIGH)
            + struct.pack(">HHH", 0x2888, 0x0501, 0x0100)
            + bytes([0xEF, 0x02, 0x01, 1, 1, len(INTERFACES)]))


class Brain:
    """The two CDC data channels to QEMU, and the URBs waiting on them."""

    def __init__(self, socks: dict[str, str]):
        self.paths = socks
        self.writers: dict[str, asyncio.StreamWriter] = {}
        self.rx = {name: bytearray() for name in socks}
        self.pending_in: dict[str, list[tuple[int, int]]] = {name: [] for name in socks}
        self.data_ready = asyncio.Event()

    async def connect(self) -> None:
        for name, path in self.paths.items():
            reader, writer = await asyncio.open_unix_connection(path)
            self.writers[name] = writer
            asyncio.create_task(self._pump(name, reader))

    async def _pump(self, name: str, reader: asyncio.StreamReader) -> None:
        while chunk := await reader.read(4096):
            self.rx[name] += chunk
            self.data_ready.set()
        print(f"usbip: QEMU closed the {name} port", file=sys.stderr, flush=True)

    def write(self, name: str, data: bytes) -> None:
        self.writers[name].write(data)


class Session:
    def __init__(self, brain: Brain, writer: asyncio.StreamWriter):
        self.brain = brain
        self.writer = writer
        self.parked: dict[int, tuple[int, int]] = {}     # seqnum -> (ep, length)
        self.alive = True
        self.flusher = asyncio.create_task(self._flush_in())

    def _ret_submit(self, seq: int, status: int, data: bytes = b"",
                    actual: int | None = None) -> None:
        n = len(data) if actual is None else actual
        hdr = struct.pack(">IIIII", RET_SUBMIT, seq, 0, 0, 0)
        hdr += struct.pack(">iiiii", status, n, 0, 0, 0) + bytes(8)
        self.writer.write(hdr + data)

    async def _flush_in(self) -> None:
        """Complete parked bulk IN URBs as the brain sends data."""
        while self.alive:
            for name, queue in self.brain.pending_in.items():
                rx = self.brain.rx[name]
                while queue and rx:
                    seq, length = queue.pop(0)
                    if seq not in self.parked:
                        continue                         # unlinked meanwhile
                    del self.parked[seq]
                    chunk = bytes(rx[:length])
                    del rx[:length]
                    self._ret_submit(seq, 0, chunk)
            await self.writer.drain()
            self.brain.data_ready.clear()
            if not any(self.brain.rx[n] and self.brain.pending_in[n]
                       for n in self.brain.rx):
                await self.brain.data_ready.wait()

    def submit(self, seq: int, direction: int, ep: int, length: int, setup: bytes,
               data: bytes) -> None:
        if ep == 0:
            status, reply = control(setup, data)
            self._ret_submit(seq, status, reply if direction == DIR_IN else b"",
                             None if direction == DIR_IN else len(data))
        elif direction == DIR_OUT and ep in BULK_OUT:
            self.brain.write(BULK_OUT[ep], data)
            self._ret_submit(seq, 0, b"", len(data))
        elif direction == DIR_IN and ep in BULK_IN:
            self.parked[seq] = (ep, length)
            self.brain.pending_in[BULK_IN[ep]].append((seq, length))
            self.brain.data_ready.set()
        elif direction == DIR_IN and ep in (1, 4):
            self.parked[seq] = (ep, length)                # notify: never fires
        else:
            self._ret_submit(seq, EPIPE)

    def unlink(self, seq: int, victim: int) -> None:
        status = ECONNRESET if self.parked.pop(victim, None) else 0
        hdr = struct.pack(">IIIII", RET_UNLINK, seq, 0, 0, 0)
        self.writer.write(hdr + struct.pack(">i", status) + bytes(24))


async def serve_urbs(brain: Brain, reader: asyncio.StreamReader,
                     writer: asyncio.StreamWriter) -> None:
    s = Session(brain, writer)
    try:
        while True:
            hdr = await reader.readexactly(48)
            cmd, seq, _devid, direction, ep = struct.unpack(">IIIII", hdr[:20])
            if cmd == CMD_SUBMIT:
                _flags, length, _sf, npk, _interval = struct.unpack(">IiiiI", hdr[20:40])
                setup = hdr[40:48]
                data = (await reader.readexactly(length)
                        if direction == DIR_OUT and length > 0 else b"")
                if npk not in (0, -1, 0xFFFFFFFF) and npk > 0:
                    await reader.readexactly(16 * npk)       # iso descriptors: none here
                s.submit(seq, direction, ep, length, setup, data)
            elif cmd == CMD_UNLINK:
                (victim,) = struct.unpack(">I", hdr[20:24])
                s.unlink(seq, victim)
            else:
                raise ValueError(f"unknown USB/IP command {cmd}")
            await writer.drain()
    finally:
        s.alive = False
        s.flusher.cancel()
        for queue in brain.pending_in.values():
            queue.clear()


async def handle(brain: Brain, reader: asyncio.StreamReader,
                 writer: asyncio.StreamWriter) -> None:
    peer = writer.get_extra_info("peername")
    try:
        while True:
            version, code, _status = struct.unpack(">HHI", await reader.readexactly(8))
            if code == OP_REQ_DEVLIST:
                rec = device_record() + b"".join(bytes([c, sc, p, 0]) for c, sc, p in INTERFACES)
                writer.write(struct.pack(">HHII", VERSION, OP_REP_DEVLIST, 0, 1) + rec)
                await writer.drain()
            elif code == OP_REQ_IMPORT:
                busid = (await reader.readexactly(32)).rstrip(b"\0")
                ok = busid == BUSID
                writer.write(struct.pack(">HHI", VERSION, OP_REP_IMPORT, 0 if ok else 1)
                             + (device_record() if ok else b""))
                await writer.drain()
                if ok:
                    print(f"usbip: {peer} attached the brain", file=sys.stderr, flush=True)
                    await serve_urbs(brain, reader, writer)
                return
            else:
                return
    except (asyncio.IncompleteReadError, ConnectionError):
        pass
    finally:
        print(f"usbip: {peer} detached", file=sys.stderr, flush=True)
        writer.close()


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sys_sock")
    ap.add_argument("user_sock")
    ap.add_argument("--port", type=int, default=3300)  # not 3240: usbipd-win owns it
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--serial", default="V5SIM0", help="USB serial number string")
    args = ap.parse_args()
    STRINGS[3] = args.serial
    brain = Brain({"sys": args.sys_sock, "user": args.user_sock})
    await brain.connect()
    server = await asyncio.start_server(lambda r, w: handle(brain, r, w), args.host, args.port)
    print(f"usbip: V5 brain on {args.host}:{args.port}, busid {BUSID.decode()}",
          file=sys.stderr, flush=True)
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
