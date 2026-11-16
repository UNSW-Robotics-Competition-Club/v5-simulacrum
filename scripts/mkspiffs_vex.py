"""Build the SPIFFS image VEXos 1.1.5.0 keeps user programs in.

MainLoop mounts SPIFFS (pellepl, 0.3.x with erase counts) at QSPI 0x1800000,
8 MB: page 256, block 64 KB, object name 32, object meta 64, u16 ids
(notes/cpu1-launch.md 4.1, 4.3, 5.B). Files are written in one pass, so
every page is either free (0xFF) or final:

  block   lookup pages 0..1: one u16 per page (object id, 0xFFFF free),
          then u16 magic 0x20140529 ^ page size, u16 erase count
  header  index page, span 0: page header, pad, u32 size, u8 type (1 file),
          name[32], meta[64], then data page indices
  index   further index pages (span 1..) for larger files
  data    page header + 251 bytes

A program is user/<name>.bin (meta: type "bin", load address, size, CRC)
plus user/<name>.ini (slot, name, autorun); progRescanFilesystem lists
every .ini whose .bin exists.

Usage (writes into an existing QSPI image in place):
  python -I mkspiffs_vex.py qspi.img NAME=prog.bin[@slot][,autorun] ...
"""
from __future__ import annotations

import struct
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path

SPIFFS_OFFSET = 0x1800000
SPIFFS_SIZE = 0x800000
PAGE = 256
BLOCK = 0x10000
PAGES_PER_BLOCK = BLOCK // PAGE
LOOKUP_PAGES = 2
NAME_LEN = 32
META_LEN = 64
MAGIC = (0x20140529 ^ PAGE) & 0xFFFF

PH_FLAG_USED = 1 << 0
PH_FLAG_FINAL = 1 << 1
PH_FLAG_INDEX = 1 << 2
FLAGS_DATA = 0xFF & ~(PH_FLAG_FINAL | PH_FLAG_USED)
FLAGS_INDEX = 0xFF & ~(PH_FLAG_FINAL | PH_FLAG_INDEX | PH_FLAG_USED)
OBJ_ID_IX_FLAG = 0x8000
TYPE_FILE = 1

PAGE_HDR = struct.Struct("<HHB")            # obj_id, span_ix, flags
IX_HDR_LEN = PAGE_HDR.size + 3 + 4 + 1 + NAME_LEN + META_LEN   # 0x6D
IX_LEN = PAGE_HDR.size + 3                  # plain index page header
DATA_LEN = PAGE - PAGE_HDR.size             # 251
HDR_ENTRIES = (PAGE - IX_HDR_LEN) // 2      # 73
IX_ENTRIES = (PAGE - IX_LEN) // 2           # 124

USER_LOAD_ADDR = 0x03800000
VID_USER = 1


@dataclass(frozen=True)
class SpiffsFile:
    name: str
    data: bytes
    meta: bytes = b""


def crc32_v5(data: bytes) -> int:
    """CRC-32 poly 0x04C11DB7, MSB first, init 0, no xorout (as V5AS)."""
    crc = 0
    for byte in data:
        crc ^= byte << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7 if crc & 0x80000000 else crc << 1) & 0xFFFFFFFF
    return crc


def program_meta(data: bytes, timestamp: int, version: int = 1) -> bytes:
    """64-byte object meta MainLoop's loader expects for user/*.bin."""
    meta = struct.pack("<BBHIII4sII", VID_USER, 0, 0, len(data), USER_LOAD_ADDR,
                       crc32_v5(data), b"bin\0", timestamp, version)
    meta += b"\xFF" * 32                      # no linked file
    return meta.ljust(META_LEN, b"\xFF")


class Image:
    def __init__(self) -> None:
        self.buf = bytearray(b"\xFF" * SPIFFS_SIZE)
        self.next_page = 0
        self.next_obj_id = 1
        for b in range(SPIFFS_SIZE // BLOCK):
            end = b * BLOCK + LOOKUP_PAGES * PAGE
            struct.pack_into("<HH", self.buf, end - 4, MAGIC, 0)

    def _alloc(self) -> int:
        """Next free page index, skipping each block's lookup pages."""
        while True:
            pix = self.next_page
            self.next_page += 1
            if pix % PAGES_PER_BLOCK < LOOKUP_PAGES:
                continue
            if pix * PAGE >= SPIFFS_SIZE:
                raise SystemExit("SPIFFS image full")
            return pix

    def _write_page(self, pix: int, obj_id: int, span: int, flags: int,
                    body: bytes) -> None:
        page = PAGE_HDR.pack(obj_id, span, flags) + body
        self.buf[pix * PAGE:pix * PAGE + len(page)] = page
        block, in_block = divmod(pix, PAGES_PER_BLOCK)
        struct.pack_into("<H", self.buf,
                         block * BLOCK + (in_block - LOOKUP_PAGES) * 2, obj_id)

    def add(self, f: SpiffsFile) -> None:
        name = f.name.encode()
        if len(name) >= NAME_LEN:
            raise SystemExit(f"name too long: {f.name}")
        obj_id = self.next_obj_id
        self.next_obj_id += 1
        chunks = [f.data[i:i + DATA_LEN] for i in range(0, len(f.data), DATA_LEN)]
        n_index = 1 + max(0, -(-(len(chunks) - HDR_ENTRIES) // IX_ENTRIES))
        index_pages = [self._alloc() for _ in range(n_index)]
        data_pages = []
        for span, chunk in enumerate(chunks):
            pix = self._alloc()
            data_pages.append(pix)
            self._write_page(pix, obj_id, span, FLAGS_DATA, chunk)

        entries = struct.pack(f"<{len(data_pages)}H", *data_pages)
        hdr = (b"\xFF" * 3 + struct.pack("<IB", len(f.data), TYPE_FILE)
               + name.ljust(NAME_LEN, b"\0") + f.meta.ljust(META_LEN, b"\xFF"))
        self._write_page(index_pages[0], obj_id | OBJ_ID_IX_FLAG, 0, FLAGS_INDEX,
                         hdr + entries[:HDR_ENTRIES * 2])
        rest = entries[HDR_ENTRIES * 2:]
        for span, pix in enumerate(index_pages[1:], start=1):
            part, rest = rest[:IX_ENTRIES * 2], rest[IX_ENTRIES * 2:]
            self._write_page(pix, obj_id | OBJ_ID_IX_FLAG, span, FLAGS_INDEX,
                             b"\xFF" * 3 + part)


def program_ini(name: str, slot: int, autorun: bool) -> bytes:
    return (f"[program]\nversion = 1\nname = {name}\nslot = {slot}\n"
            f"icon = USER029x.bmp\ndescription = built by mkspiffs_vex\n"
            f"date = {time.strftime('%Y-%m-%dT%H:%M:%S')}\n"
            f"[project]\nide = mkspiffs_vex\n"
            f"[options]\nautorun = {'true' if autorun else 'false'}\n").encode()


def parse_spec(spec: str) -> tuple[str, Path, int, bool]:
    name, _, rest = spec.partition("=")
    rest, _, flags = rest.partition(",")
    path, _, slot = rest.partition("@")
    return name, Path(path), int(slot or 0), flags == "autorun"


def build(specs: list[str]) -> bytes:
    img = Image()
    now = int(time.time())
    for spec in specs:
        name, path, slot, autorun = parse_spec(spec)
        data = path.read_bytes()
        if data[:4] != b"XVX5":
            raise SystemExit(f"{path}: not a V5 user program (no XVX5 header)")
        img.add(SpiffsFile(f"user/{name}.bin", data, program_meta(data, now)))
        img.add(SpiffsFile(f"user/{name}.ini", program_ini(name, slot, autorun)))
    return bytes(img.buf)


def is_formatted(qspi: Path) -> bool:
    """Block 0 of the SPIFFS region carries the magic (formatted, maybe empty)."""
    with qspi.open("rb") as f:
        f.seek(SPIFFS_OFFSET + LOOKUP_PAGES * PAGE - 4)
        raw = f.read(2)
    return len(raw) == 2 and struct.unpack("<H", raw)[0] == MAGIC


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    if sys.argv[1] == "--formatted":          # exit status only
        sys.exit(0 if is_formatted(Path(sys.argv[2])) else 1)
    qspi = Path(sys.argv[1])
    image = bytearray(qspi.read_bytes())
    if len(image) < SPIFFS_OFFSET + SPIFFS_SIZE:
        raise SystemExit(f"{qspi}: smaller than {SPIFFS_OFFSET + SPIFFS_SIZE:#x}")
    image[SPIFFS_OFFSET:SPIFFS_OFFSET + SPIFFS_SIZE] = build(sys.argv[2:])
    qspi.write_bytes(image)
    print(f"SPIFFS @{SPIFFS_OFFSET:#x}: {len(sys.argv) - 2} program(s)")


if __name__ == "__main__":
    main()
