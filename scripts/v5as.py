"""Decode a VEXos V5AS assets.bin as the brain's MainLoop does.

Layout: 0x200-byte header {magic 'V5AS', fmt 0x100, asset_ver, crc, len}, then a
payload XOR'ed in 0x1000-byte blocks with a fixed key from MainLoop, which is a gzipped tar.
CRC = CRC-32 poly 0x04C11DB7, MSB-first, init 0, no xorout, over the stored payload.

Usage: python -I v5as.py <assets.bin> <MainLoop.bin> <out_dir>
"""
import gzip
import struct
import sys
import tarfile
from pathlib import Path

MAGIC = 0x53413556  # 'V5AS'
HEADER_LEN = 0x200
BLOCK = 0x1000
XOR_FORMAT = 0x100
MAINLOOP_BASE = 0x01000000
KEY_ADDR = 0x010C60BC  # *(DAT_0103cb5c) + 1


def crc32_mpeg_noxor(data: bytes) -> int:
    table = []
    for i in range(256):
        c = i << 24
        for _ in range(8):
            c = ((c << 1) ^ 0x04C11DB7) if c & 0x80000000 else c << 1
        table.append(c & 0xFFFFFFFF)
    crc = 0
    for b in data:
        crc = (table[(b ^ (crc >> 24)) & 0xFF] ^ (crc << 8)) & 0xFFFFFFFF
    return crc


def decode(assets: bytes, mainloop: bytes) -> tuple[dict, bytes]:
    magic, fmt, asset_ver, crc, length = struct.unpack_from("<5I", assets, 0)
    if magic != MAGIC:
        raise ValueError("not a V5AS file")
    header = {"format": fmt, "asset_version": f"{asset_ver:08x}", "crc": f"{crc:08x}", "length": length}
    payload = assets[HEADER_LEN:HEADER_LEN + length]
    actual = crc32_mpeg_noxor(payload)
    if actual != crc:
        raise ValueError(f"crc mismatch: header {crc:08x} computed {actual:08x}")
    if fmt == XOR_FORMAT:
        key_off = KEY_ADDR - MAINLOOP_BASE
        key = mainloop[key_off:key_off + BLOCK]
        payload = bytes(b ^ key[i % BLOCK] for i, b in enumerate(payload))
    if payload[:2] != b"\x1f\x8b":
        raise ValueError(f"decoded payload is not gzip (starts {payload[:4].hex()})")
    return header, gzip.decompress(payload)


def main(assets_path: str, mainloop_path: str, out_dir: str) -> None:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    header, blob = decode(Path(assets_path).read_bytes(), Path(mainloop_path).read_bytes())
    tar_path = out / "assets.decoded.bin"
    tar_path.write_bytes(blob)
    with tarfile.open(tar_path) as tar:
        members = tar.getmembers()
        tar.extractall(out / "files", filter="data")
    print(header, f"decoded {len(blob):#x} bytes, {len(members)} files -> {out / 'files'}")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
