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

A program can be linked to a library file, which VEXos loads with it (as
VEXos itself stores what PROS and vexcom upload; meta[32:64] names the
library as "<vid dir>/<name>"); both are kept gzip-compressed, as uploads are:
  prog.bin,link=cold.bin   PROS hot/cold: the hot package at 0x07800000,
                           linked to pros/cold_<crc> (the cold package) at
                           0x03800000
  main.py,link=vm.bin      VEXcode Python: the source at 0x07000000, linked
                           to vxvm/python_vm.bin (VEX's MicroPython VM, from
                           the VEXcode extension's SDK) at 0x03800000

Usage (writes into an existing QSPI image in place):
  python -I mkspiffs_vex.py qspi.img NAME=prog.bin[@slot][,autorun][,link=lib.bin] ...
  python -I mkspiffs_vex.py --list qspi.img      the files, their meta and links
"""
from __future__ import annotations

import gzip
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
PH_FLAG_DELET = 1 << 7                  # cleared when a page is deleted
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
VID_PROS = 0x18                      # PROS's libraries (pros/...)
VID_VXVM = 0x40                      # VEX's Python VM (vxvm/...)
HOT_LOAD_ADDR = 0x07800000           # a PROS hot package
PY_LOAD_ADDR = 0x07000000            # a VEXcode Python program's source
LIB_LOAD_ADDR = 0x03800000           # its library (cold package, Python VM)
OPT_UPLOADED = 1                     # meta byte 1 as VEXos writes uploads (.ini too: not a gzip flag)
LINK_LEN = 32                        # meta[32:64]: the linked file's name


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


def file_meta(vid: int, stored: bytes, addr: int, kind: bytes, timestamp: int,
              version: int, link: str = "") -> bytes:
    """The meta VEXos writes for an uploaded (gzip) file: size and CRC are of
    the stored bytes; a linked file's name fills meta[32:64]."""
    meta = struct.pack("<BBHIII4sII", vid, OPT_UPLOADED, 0xFFFF, len(stored), addr,
                       crc32_v5(stored), kind, timestamp, version)
    meta += b"\xFF" * 4
    if link:
        name = link.encode()
        if len(name) >= LINK_LEN:
            raise SystemExit(f"linked name too long: {link}")
        meta += name.ljust(LINK_LEN, b"\0")
    return meta.ljust(META_LEN, b"\xFF")


def packed(data: bytes) -> bytes:
    """gzip, as uploads are stored (already-gzip data as it is)."""
    return data if data[:2] == b"\x1f\x8b" else gzip.compress(data, mtime=0)


def header_version(data: bytes) -> int:
    """The version in a program's XVX5 header (bytes 28..31), gzip or not."""
    raw = gzip.decompress(data) if data[:2] == b"\x1f\x8b" else data
    if raw[:4] != b"XVX5":
        raise SystemExit("not a V5 program image (no XVX5 header)")
    return struct.unpack_from("<I", raw, 28)[0]


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


@dataclass(frozen=True)
class Spec:
    name: str
    path: Path
    slot: int = 0
    autorun: bool = False
    link: Path | None = None


def parse_spec(spec: str) -> Spec:
    name, _, rest = spec.partition("=")
    rest, *options = rest.split(",")
    path, _, slot = rest.partition("@")
    autorun, link = False, None
    for opt in options:
        if opt == "autorun":
            autorun = True
        elif opt.startswith("link="):
            link = Path(opt[5:])
        else:
            raise SystemExit(f"{spec}: unknown option {opt!r}")
    return Spec(name, Path(path), int(slot or 0), autorun, link)


# VEX's timestamps count seconds from 2000-01-01
VEX_EPOCH = 946684800


def linked_files(spec: Spec, data: bytes, now: int) -> list[SpiffsFile]:
    """A PROS hot package or a VEXcode Python program, and its library."""
    lib = spec.link.read_bytes()
    stamp = now - VEX_EPOCH
    if spec.path.suffix.lower() == ".py":
        lib_name, lib_meta_args = "vxvm/python_vm.bin", (VID_VXVM, header_version(lib))
        prog_addr, prog_kind, prog_version = PY_LOAD_ADDR, b"bina", 1
    else:
        if lib[:4] != b"XVX5" and lib[:2] != b"\x1f\x8b":
            raise SystemExit(f"{spec.link}: not a PROS cold package (no XVX5 header)")
        # named for its contents, so programs built on one cold package
        # share it (PROS names it by a hash too, under 18 characters)
        lib_name = f"pros/cold_{crc32_v5(lib):08x}"
        lib_meta_args = (VID_PROS, 0x01000000)
        prog_addr, prog_kind, prog_version = HOT_LOAD_ADDR, b"bin\0", 0x01000000
    lib_stored = packed(lib)
    prog_stored = packed(data)
    vid, version = lib_meta_args
    return [
        SpiffsFile(lib_name, lib_stored,
                   file_meta(vid, lib_stored, LIB_LOAD_ADDR, b"bin\0", stamp, version)),
        SpiffsFile(f"user/{spec.name}.bin", prog_stored,
                   file_meta(VID_USER, prog_stored, prog_addr, prog_kind, stamp, prog_version, lib_name)),
    ]


def build(specs: list[str]) -> bytes:
    img = Image()
    now = int(time.time())
    libraries: set[str] = set()
    for text in specs:
        spec = parse_spec(text)
        data = spec.path.read_bytes()
        if spec.link is not None:
            for f in linked_files(spec, data, now):
                if f.name.startswith("user/") or f.name not in libraries:
                    img.add(f)
                    libraries.add(f.name)
        else:
            if data[:4] != b"XVX5":
                raise SystemExit(f"{spec.path}: not a V5 user program (no XVX5 header)")
            img.add(SpiffsFile(f"user/{spec.name}.bin", data, program_meta(data, now)))
        img.add(SpiffsFile(f"user/{spec.name}.ini", program_ini(spec.name, spec.slot, spec.autorun)))
    return bytes(img.buf)


def read_files(spiffs: bytes) -> dict[str, tuple[bytes, bytes]]:
    """name -> (data, meta) for every live file (the inverse of Image.add)."""
    index: dict[tuple[int, int], int] = {}
    heads: dict[int, tuple[str, int, bytes]] = {}
    for pix in range(len(spiffs) // PAGE):
        if pix % PAGES_PER_BLOCK < LOOKUP_PAGES:
            continue
        obj_id, span, flags = PAGE_HDR.unpack_from(spiffs, pix * PAGE)
        if obj_id in (0, 0xFFFF) or not obj_id & OBJ_ID_IX_FLAG:
            continue
        if flags & (PH_FLAG_USED | PH_FLAG_FINAL | PH_FLAG_INDEX) or not flags & PH_FLAG_DELET:
            continue                           # active-low; deleted pages have 0x80 clear
        index[(obj_id, span)] = pix
        if span == 0:
            off = pix * PAGE + PAGE_HDR.size + 3
            size = struct.unpack_from("<I", spiffs, off)[0]
            name = spiffs[off + 5:off + 5 + NAME_LEN].split(b"\0")[0].decode(errors="replace")
            heads[obj_id] = (name, size, bytes(spiffs[off + 5 + NAME_LEN:off + 5 + NAME_LEN + META_LEN]))
    out = {}
    for obj_id, (name, size, meta) in heads.items():
        pages: list[int] = []
        span = 0
        while (obj_id, span) in index and len(pages) * DATA_LEN < size:
            pix = index[(obj_id, span)]
            start, count = (IX_HDR_LEN, HDR_ENTRIES) if span == 0 else (IX_LEN, IX_ENTRIES)
            for i in range(count):
                p = struct.unpack_from("<H", spiffs, pix * PAGE + start + 2 * i)[0]
                if p == 0xFFFF:
                    break
                pages.append(p)
            span += 1
        data = b"".join(spiffs[p * PAGE + PAGE_HDR.size:(p + 1) * PAGE] for p in pages)
        out[name] = (data[:size], meta)
    return out


def describe(name: str, data: bytes, meta: bytes) -> str:
    if meta[:1] == b"\xFF":
        return f"{name}  {len(data)} bytes"
    vid, _opt, _, size, addr, crc, kind, _, version = struct.unpack_from("<BBHIII4sII", meta)
    link = meta[32:64].split(b"\0")[0]
    link_text = "" if link[:1] in (b"", b"\xFF") else f"  link {link.decode(errors='replace')}"
    gz = " gzip" if data[:2] == b"\x1f\x8b" else ""
    ok = "" if crc == crc32_v5(data) and size == len(data) else "  BAD CRC/SIZE"
    return (f"{name}  {len(data)} bytes{gz}  vid {vid:#x} addr {addr:#010x} type {kind.rstrip(bytes(1))!r}"
            f" version {version:#x}{link_text}{ok}")


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
    if sys.argv[1] == "--list":
        raw = Path(sys.argv[2]).read_bytes()[SPIFFS_OFFSET:SPIFFS_OFFSET + SPIFFS_SIZE]
        for name, (data, meta) in sorted(read_files(raw).items()):
            print(describe(name, data, meta))
        return
    qspi = Path(sys.argv[1])
    image = bytearray(qspi.read_bytes())
    if len(image) < SPIFFS_OFFSET + SPIFFS_SIZE:
        raise SystemExit(f"{qspi}: smaller than {SPIFFS_OFFSET + SPIFFS_SIZE:#x}")
    image[SPIFFS_OFFSET:SPIFFS_OFFSET + SPIFFS_SIZE] = build(sys.argv[2:])
    qspi.write_bytes(image)
    print(f"SPIFFS @{SPIFFS_OFFSET:#x}: {len(sys.argv) - 2} program(s)")


if __name__ == "__main__":
    main()
