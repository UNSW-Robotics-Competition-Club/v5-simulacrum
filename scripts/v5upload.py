#!/usr/bin/env python3
"""Upload a program to a running sim brain over its USB system port and
start it: Run on Sim without restarting QEMU. Speaks VEXos's CDC2 file
transfer (notes/usb-protocol.md 7) as PROS and vexcom do, and stores what
--run installs (mkspiffs_vex.py), so both routes leave the same files:

  prog.bin                   user/slot_N.bin at 0x03800000
  hot.package.bin --link cold.package.bin
                             PROS hot/cold: pros/cold_<crc> at 0x03800000
                             (skipped when the brain already has it), the hot
                             package at 0x07800000 linked to it
  main.py --link python_vm.bin
                             VEXcode Python: vxvm/python_vm.bin (skipped when
                             already there), the source at 0x07000000

plus user/slot_N.ini. Programs are sent gzip-compressed, as uploads are.

  python3 -I v5upload.py SOCK FILE [--link LIB] [--slot N] [--name NAME] [--after run|screen|none]

SOCK is v5usb_share.py's host socket (v5brain.sh prints it as "upload ...")
or any unix socket / tty that is the brain's system port. Progress goes to
stdout as "upload NAME DONE/TOTAL" lines.
"""
from __future__ import annotations

import argparse
import os
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # -I leaves it out
from mkspiffs_vex import (HOT_LOAD_ADDR, LIB_LOAD_ADDR, PY_LOAD_ADDR,  # noqa: E402
                          USER_LOAD_ADDR, VEX_EPOCH, VID_PROS, VID_USER, VID_VXVM,
                          crc32_v5, header_version, packed, program_ini)

HEADER = b"\xC9\x36\xB8\x47"
REPLY = b"\xAA\x55"
EXT = 0x56
ACK = 0x76
FT_INIT, FT_COMPLETE, FT_WRITE, FT_SET_LINK, FT_META_BY_NAME = 0x11, 0x12, 0x13, 0x15, 0x19
FUNCTION_UPLOAD, TARGET_FLASH, OPT_OVERWRITE = 1, 1, 1
AFTER = {"none": 0, "run": 1, "screen": 3}
PROGRAM_VERSION = 0x01000000          # as PROS writes it
NAME_LEN = 24
TIMEOUT_S = 2.0
RETRIES = 3
NACKS = {0xFF: "general NACK", 0xCE: "CRC error", 0xD0: "payload too small", 0xD1: "transfer size too large",
         0xD2: "program CRC error", 0xD3: "program file error", 0xD4: "uninitialized transfer",
         0xD5: "initialization invalid", 0xD6: "data not a multiple of 4 bytes", 0xD7: "packet address mismatch",
         0xD8: "downloaded size mismatch", 0xD9: "directory entry exists", 0xDA: "max user files",
         0xDB: "user file exists"}


class UploadError(Exception):
    pass


def crc16(data: bytes, crc: int = 0) -> int:
    """CRC-16/XMODEM (poly 0x1021, init 0), as the brain checks it."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def varint(n: int) -> bytes:
    """A CDC2 length: one byte below 0x80, else two (high bit set)."""
    return bytes([n]) if n < 0x80 else bytes([0x80 | n >> 8, n & 0xFF])


def ext_packet(cmd: int, payload: bytes) -> bytes:
    body = HEADER + bytes([EXT, cmd]) + varint(len(payload)) + payload
    return body + crc16(body).to_bytes(2, "big")


class Port:
    """The brain's system port: a unix socket (or a tty) to write packets
    to and read replies from."""

    def __init__(self, path: str):
        if Path(path).is_socket():
            self.sock: socket.socket | None = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.connect(path)
            self.fd = self.sock.fileno()
        else:
            self.sock = None
            self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        self.buf = bytearray()

    def write(self, data: bytes) -> None:
        if self.sock:
            self.sock.sendall(data)
        else:
            while data:
                data = data[os.write(self.fd, data):]

    def read_some(self, timeout: float) -> bytes:
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        data = self.sock.recv(4096) if self.sock else os.read(self.fd, 4096)
        if not data:
            raise UploadError("the brain's port closed")
        return data

    def drain(self) -> None:
        """Drops replies nobody waited for (a stale one would be read as ours)."""
        self.buf.clear()
        while self.read_some(0):
            pass

    def close(self) -> None:
        if self.sock:
            self.sock.close()
        else:
            os.close(self.fd)


def parse_reply(buf: bytearray, cmd: int) -> bytes | None:
    """The payload of the first complete ext reply to cmd in buf (consumed),
    without its ACK byte; None if none is complete yet. Other bytes before
    it (noise, replies to other commands) are dropped."""
    while True:
        start = buf.find(REPLY + bytes([EXT]))
        if start < 0:
            del buf[:max(0, len(buf) - 2)]
            return None
        del buf[:start]
        if len(buf) < 4:
            return None
        n, hdr = buf[3], 4
        if n & 0x80:
            if len(buf) < 5:
                return None
            n, hdr = ((n & 0x7F) << 8) | buf[4], 5
        if len(buf) < hdr + n:
            return None
        frame = bytes(buf[:hdr + n])
        del buf[:hdr + n]
        if crc16(frame) != 0 or n < 3:
            continue                                   # corrupt: look for the next
        ext, payload = frame[hdr], frame[hdr + 1:-2]
        if ext != cmd:
            continue
        if not payload or payload[0] != ACK:
            code = payload[0] if payload else 0xFF
            raise UploadError(f"brain refused command {cmd:#04x}: {NACKS.get(code, hex(code))}")
        return payload[1:]


class Brain:
    def __init__(self, port: Port, timeout: float = TIMEOUT_S):
        self.port, self.timeout = port, timeout

    def ext(self, cmd: int, payload: bytes, timeout: float | None = None) -> bytes:
        """Sends an ext command and returns its reply (after the ACK),
        resending on a timeout."""
        wait = timeout or self.timeout
        for _ in range(RETRIES):
            self.port.drain()
            self.port.write(ext_packet(cmd, payload))
            end = time.monotonic() + wait
            while time.monotonic() < end:
                self.port.buf += self.port.read_some(max(0.0, end - time.monotonic()))
                reply = parse_reply(self.port.buf, cmd)
                if reply is not None:
                    return reply
        raise UploadError(f"no reply from the brain to command {cmd:#04x} (is the simulator running?)")

    def file_meta(self, name: str, vid: int) -> tuple[int, int] | None:
        """(size, crc) of a file on the brain, or None when it isn't there."""
        try:
            r = self.ext(FT_META_BY_NAME, struct.pack("<2B24s", vid, 0, name.encode()))
        except UploadError as e:
            if "refused" in str(e):
                return None
            raise
        if len(r) < 13:
            return None
        _linked_vid, size, _addr, crc = struct.unpack_from("<B3I", r)
        return size, crc

    def write_file(self, name: str, data: bytes, *, vid: int, addr: int, kind: bytes, version: int,
                   link: tuple[str, int] | None = None, after: int = 0, progress=None) -> None:
        if len(name) > NAME_LEN:
            raise UploadError(f"name too long for the brain: {name}")
        stamp = int(time.time()) - VEX_EPOCH
        init = struct.pack("<4B3I4s2I24s", FUNCTION_UPLOAD, TARGET_FLASH, vid, OPT_OVERWRITE, len(data), addr,
                           crc32_v5(data), kind.ljust(4, b"\0")[:4], stamp, version, name.encode())
        r = self.ext(FT_INIT, init, timeout=self.timeout * 5)
        max_packet = struct.unpack_from("<H", r)[0]
        if link:
            self.ext(FT_SET_LINK, struct.pack("<2B24s", link[1], 0, link[0].encode()))
        chunk = max(4, (max_packet // 2) & ~3)                 # as PROS: half the brain's maximum
        for i in range(0, len(data), chunk):
            part = data[i:i + chunk]
            part += bytes(-len(part) % 4)
            self.ext(FT_WRITE, struct.pack("<I", addr + i) + part)
            if progress:
                progress(name, min(i + chunk, len(data)), len(data))
        # writing to flash takes longer for bigger files
        self.ext(FT_COMPLETE, bytes([after]), timeout=max(self.timeout, len(data) / 50_000))


@dataclass(frozen=True)
class Upload:
    """What to send: each file as the brain stores it."""
    files: list[dict]


def plan(program: Path, link: Path | None, slot: int, name: str, after: str) -> Upload:
    """The files an upload writes, in order: library, ini, program (which
    runs when its transfer completes)."""
    data = program.read_bytes()
    remote = f"slot_{slot}"
    files: list[dict] = []
    is_py = program.suffix.lower() == ".py"
    if is_py and link is None:
        raise UploadError(f"{program}: a Python program needs --link with VEX's python_vm.bin")
    if link is not None:
        lib = link.read_bytes()
        link_name = link.name.lower()
        if is_py:
            if link_name.startswith("cold"):
                raise UploadError(f"{link}: a PROS cold package; a Python program links to VEX's python_vm.bin")
            lib_name, lib_vid, lib_version = "python_vm.bin", VID_VXVM, header_version(lib, link)
            addr, kind, version = PY_LOAD_ADDR, b"bina", 1
        else:
            if "python_vm" in link_name:
                raise UploadError(f"{link}: VEX's Python VM; a .bin links to a PROS cold package")
            if lib[:4] != b"XVX5" and lib[:2] != b"\x1f\x8b":
                raise UploadError(f"{link}: not a PROS cold package (no XVX5 header)")
            lib_name, lib_vid, lib_version = f"cold_{crc32_v5(lib):08x}", VID_PROS, PROGRAM_VERSION
            addr, kind, version = HOT_LOAD_ADDR, b"bin", PROGRAM_VERSION
        files.append(dict(name=lib_name, data=packed(lib), vid=lib_vid, addr=LIB_LOAD_ADDR, kind=b"bin",
                          version=lib_version, library=True))
        prog_link = (lib_name, lib_vid)
    else:
        if data[:4] != b"XVX5" and data[:2] != b"\x1f\x8b":
            raise UploadError(f"{program}: not a V5 program (no XVX5 header)")
        addr, kind, version, prog_link = USER_LOAD_ADDR, b"bin", PROGRAM_VERSION, None
    ini = program_ini(name, slot - 1, False)
    files.append(dict(name=f"{remote}.ini", data=ini, vid=VID_USER, addr=addr, kind=b"ini", version=1))
    files.append(dict(name=f"{remote}.bin", data=packed(data), vid=VID_USER, addr=addr, kind=kind,
                      version=version, link=prog_link, after=AFTER[after]))
    return Upload(files)


def upload(brain: Brain, up: Upload, progress=None) -> list[str]:
    """Sends the files; returns the names written (a library the brain
    already has is skipped)."""
    written = []
    for f in up.files:
        if f.get("library"):
            have = brain.file_meta(f["name"], f["vid"])
            if have == (len(f["data"]), crc32_v5(f["data"])):
                continue
        brain.write_file(f["name"], f["data"], vid=f["vid"], addr=f["addr"], kind=f["kind"], version=f["version"],
                         link=f.get("link"), after=f.get("after", 0), progress=progress)
        written.append(f["name"])
    return written


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port", help="the brain's system port: v5usb_share.py's host socket, or a tty")
    ap.add_argument("program", type=Path)
    ap.add_argument("--link", type=Path, help="a PROS cold package, or VEX's python_vm.bin for a .py")
    ap.add_argument("--slot", type=int, default=1, choices=range(1, 9), metavar="1-8")
    ap.add_argument("--name", help="the program's name on the brain (default: the file's)")
    ap.add_argument("--after", choices=AFTER, default="run")
    a = ap.parse_args(argv)
    name = a.name or ("slot_%d" % a.slot if a.program.name == "hot.package.bin" else a.program.stem)[:20]

    def progress(remote: str, done: int, total: int) -> None:
        print(f"upload {remote} {done}/{total}", flush=True)

    try:
        up = plan(a.program, a.link, a.slot, name, a.after)
        port = Port(a.port)
        try:
            written = upload(Brain(port), up, progress)
        finally:
            port.close()
    except (UploadError, OSError) as e:
        print(f"v5upload: {e}", file=sys.stderr)
        return 1
    print(f"uploaded {', '.join(written)} to slot {a.slot}" + (", running it" if a.after == "run" else ""), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
