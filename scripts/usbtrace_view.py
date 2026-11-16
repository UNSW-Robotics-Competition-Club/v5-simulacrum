#!/usr/bin/env python3
"""Read a V5USB_TRACE file (scripts/v5usb_pty.py; e2e_all.sh keeps one in
/tmp/v5usb_e2e_trace.txt with a "=== <test>" line before each test).

  usbtrace_view.py TRACE [SECTION] [--user | --sys]

Prints a timeline: user-port chunks as text, system-port host commands as
"C9 36 B8 47 <cmd> [ext]" with their brain replies' status byte.
"""
from __future__ import annotations

import sys

CDC2_HEADER = bytes.fromhex("c936b847")


def describe_sys(data: bytes) -> str:
    if data[:4] == CDC2_HEADER and len(data) > 4:
        cmd = data[4]
        if cmd == 0x56 and len(data) > 6:
            ext_at = 7 if data[5] & 0x80 else 6
            return f"cmd 56 ext {data[ext_at]:02x} ({len(data)} B)"
        return f"cmd {cmd:02x} ({len(data)} B)"
    if data[:2] == b"\xaa\x55" and len(data) > 5:
        return f"reply {data[2]:02x} " + (f"ext {data[4]:02x} ack {data[5]:02x}"
                                          if data[2] == 0x56 else f"({len(data)} B)")
    return f"{len(data)} B"


def main(argv: list[str]) -> None:
    if not argv:
        sys.exit(__doc__)
    path, rest = argv[0], argv[1:]
    only = {"--user": "ttyV5user", "--sys": "ttyV5sys"}
    want = [only[a] for a in rest if a in only] or list(only.values())
    section = next((a for a in rest if not a.startswith("--")), None)
    active = section is None
    for line in open(path, errors="replace"):
        if line.startswith("==="):
            active = section is None or section in line
            if active:
                print(line.rstrip())
            continue
        if not active:
            continue
        t, link, arrow, *hexes = line.split()
        if not any(w in link for w in want):
            continue
        data = bytes.fromhex("".join(hexes))
        if "ttyV5user" in link:
            print(f"{t:>9} user {arrow:6} {data[:100]!r}")
        else:
            print(f"{t:>9} sys  {arrow:6} {describe_sys(data)}")


if __name__ == "__main__":
    main(sys.argv[1:])
