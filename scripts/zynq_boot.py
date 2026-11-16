"""Split a Xilinx Zynq-7000 BOOT.bin into its partitions (layout per UG585 / bootgen).

Usage: python -I zynq_boot.py <BOOT.bin> <out_dir>
"""
import json
import struct
import sys
from pathlib import Path

SYNC_WORD = 0xAA995566
XNLX = b"XNLX"
ENCRYPTION_NONE = 0x0
PART_HDR_WORDS = 16
DEST_DEVICE = {0: "none", 1: "ps", 2: "pl", 3: "int"}


def u32(buf, off):
    return struct.unpack_from("<I", buf, off)[0]


def parse_header(buf):
    if u32(buf, 0x20) != SYNC_WORD or buf[0x24:0x28] != XNLX:
        raise ValueError("not a Zynq boot image (missing width-detect / XNLX)")
    return {
        "encryption_status": u32(buf, 0x28),
        "header_version": u32(buf, 0x2C),
        "fsbl_offset": u32(buf, 0x30),
        "fsbl_length": u32(buf, 0x34),
        "fsbl_load_addr": u32(buf, 0x38),
        "fsbl_exec_addr": u32(buf, 0x3C),
        "fsbl_total_length": u32(buf, 0x40),
        "image_header_table_offset": u32(buf, 0x98),
        "partition_header_table_offset": u32(buf, 0x9C),
    }


def image_name(buf, image_hdr_word_off):
    """Image header: next(4) part_hdr(4) reserved(4) part_count(4) name(big-endian words, NUL-terminated)."""
    off = image_hdr_word_off * 4 + 16
    raw = bytearray()
    while off + 4 <= len(buf):
        word = buf[off:off + 4][::-1]  # name chars are stored per-word big-endian
        raw += word
        off += 4
        if b"\x00" in word:
            break
    return raw.split(b"\x00")[0].decode("ascii", "replace")


def parse_partitions(buf, table_off):
    parts = []
    off = table_off
    while True:
        words = struct.unpack_from(f"<{PART_HDR_WORDS}I", buf, off)
        if not any(words[:15]):
            break  # null terminator header
        attr = words[6]
        parts.append({
            "encrypted_len": words[0] * 4,
            "unencrypted_len": words[1] * 4,
            "total_len": words[2] * 4,
            "load_addr": words[3],
            "exec_addr": words[4],
            "data_offset": words[5] * 4,
            "attributes": attr,
            "dest_device": DEST_DEVICE.get((attr >> 4) & 0x3, "?"),
            "authenticated": bool(attr & (1 << 15)),
            "section_count": words[7],
            "name": image_name(buf, words[9]) if words[9] else "",
        })
        off += PART_HDR_WORDS * 4
    return parts


def main(boot_path, out_dir):
    buf = Path(boot_path).read_bytes()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    header = parse_header(buf)
    parts = parse_partitions(buf, header["partition_header_table_offset"])
    for i, p in enumerate(parts):
        p["encrypted"] = p["encrypted_len"] != p["unencrypted_len"] or header["encryption_status"] != ENCRYPTION_NONE
        data = buf[p["data_offset"]:p["data_offset"] + p["unencrypted_len"]]
        stem = Path(p["name"]).stem or f"part{i}"
        p["file"] = f"p{i}_{stem}_{p['dest_device']}_{p['load_addr']:08x}.bin"
        (out / p["file"]).write_bytes(data)
        print(f"[{i}] {p['name']:<24} dest={p['dest_device']:<4} load=0x{p['load_addr']:08x} "
              f"exec=0x{p['exec_addr']:08x} off=0x{p['data_offset']:06x} len=0x{len(data):06x} "
              f"enc={p['encrypted']} auth={p['authenticated']}")
    (out / "partitions.json").write_text(json.dumps({"header": header, "partitions": parts}, indent=2))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
