#!/usr/bin/env python3
"""Wait for simulator events instead of sleeping a fixed time (the e2e
checks; e2e_env.sh wraps these as e2e_started / e2e_still / e2e_log).

    e2e_wait.py started OUT [TIMEOUT]       v5brain.sh's output says the
                                            program started
    e2e_wait.py still PORT SECS [TIMEOUT]   the robot (field view on PORT)
                                            moved, then it and every motor
                                            stayed still for SECS
    e2e_wait.py log FILE REGEX [TIMEOUT]    a line in FILE matches REGEX

Exit 0 when it happened, 1 on timeout (with a note on stderr), so a check
carries on and reports what it saw either way.
"""
from __future__ import annotations

import json
import math
import re
import sys
import time
import urllib.request
from pathlib import Path

POLL_S = 0.1
STILL_M = 0.003
STILL_DEG = 0.5
STILL_RPM = 2.0


def wait_log(path: str, pattern: str, timeout: float) -> bool:
    rx = re.compile(pattern)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            if any(rx.search(line) for line in
                   Path(path).read_text(errors="replace").splitlines()):
                return True
        except OSError:
            pass
        time.sleep(POLL_S)
    return False


def _get(port: int, path: str) -> dict | None:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=1.0) as r:
            return json.load(r)
    except (OSError, ValueError):
        return None


def wait_still(port: int, secs: float, timeout: float) -> bool:
    end = time.monotonic() + timeout
    anchor, moved = None, False
    while time.monotonic() < end:
        time.sleep(POLL_S)
        s, d = _get(port, "/state.json"), _get(port, "/devices.json")
        if s is None or d is None:
            continue
        spinning = any(abs(m.get("velocity_rpm") or 0) > STILL_RPM
                       for m in d["devices"] if m["kind"] == "motor")
        if anchor is None:
            anchor = (s["t"], s["x"], s["y"], s["heading"])
        if (math.hypot(s["x"] - anchor[1], s["y"] - anchor[2]) > STILL_M
                or abs(math.degrees(s["heading"] - anchor[3])) > STILL_DEG or spinning):
            anchor, moved = (s["t"], s["x"], s["y"], s["heading"]), True
        elif moved and s["t"] - anchor[0] >= secs:
            return True
    return False


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    what = argv[0]
    if what == "started":
        ok = wait_log(argv[1], r"\[v5brain\] started slot", float(argv[2]) if len(argv) > 2 else 60)
    elif what == "still":
        ok = wait_still(int(argv[1]), float(argv[2]), float(argv[3]) if len(argv) > 3 else 60)
    elif what == "log":
        ok = wait_log(argv[1], argv[2], float(argv[3]) if len(argv) > 3 else 60)
    else:
        print(f"e2e_wait.py: unknown event {what!r}", file=sys.stderr)
        return 2
    if not ok:
        print(f"e2e_wait.py: timed out waiting for {' '.join(argv)}", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
