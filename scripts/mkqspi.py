"""Build a 32 MB QSPI flash image for the vex-v5 QEMU machine.

Layout (from MainLoop): BOOT.bin at 0, assets.bin (V5AS container) at 0x800000.
Usage: python -I mkqspi.py <VEXOS dir> <out.img>
"""
import sys
from pathlib import Path

FLASH_SIZE = 32 * 1024 * 1024
ASSETS_OFFSET = 0x800000


def main():
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    img = bytearray(b"\xff" * FLASH_SIZE)
    for name, off in (("BOOT.bin", 0), ("assets.bin", ASSETS_OFFSET)):
        data = (src / name).read_bytes()
        if off + len(data) > FLASH_SIZE:
            raise SystemExit(f"{name} does not fit at {off:#x}")
        img[off:off + len(data)] = data
        print(f"{name}: {len(data):#x} bytes at {off:#x}")
    out.write_bytes(img)


if __name__ == "__main__":
    main()
