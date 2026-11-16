"""Press the brain's front button through QEMU's QMP socket.

The button is bit 7 of AXI GPIO 1 (0x41210000), active low
(notes/mmio-map.md 3.1): VEXos polls it every 10 ms (sysUserExitMonitor).
A tap goes Back/Home in the UI, about 1 s held stops a running program,
about 3 s held with no program powers off: VEXos sends the NXP its power-off
request and QEMU exits, closing the sim. The machine exposes the
GPIO's input word as QOM property /machine/gpio1 "in1".
"""
from __future__ import annotations

import json
import socket
import sys
import threading
import time

GPIO_PATH = "/machine/gpio1"
IDLE = 0x80                     # bit 7 high: not pressed
PRESSED = 0x00
MAX_PRESS_S = 10.0


class Qmp:
    def __init__(self, path: str, timeout_s: float = 2.0):
        self.sock = socket.socket(socket.AF_UNIX)
        self.sock.settimeout(timeout_s)
        self.sock.connect(path)
        self.buf = b""
        self._recv()                              # greeting
        self.cmd("qmp_capabilities")

    def _recv(self) -> dict:
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("QMP closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)

    def cmd(self, name: str, **args) -> object:
        msg = {"execute": name, **({"arguments": args} if args else {})}
        self.sock.sendall(json.dumps(msg).encode() + b"\n")
        while True:
            reply = self._recv()
            if "event" in reply:
                continue
            if "error" in reply:
                raise RuntimeError(f"{name}: {reply['error']}")
            return reply.get("return")

    def close(self) -> None:
        self.sock.close()


def set_button(qmp: Qmp, pressed: bool) -> None:
    qmp.cmd("qom-set", path=GPIO_PATH, property="in1",
            value=PRESSED if pressed else IDLE)


def press(path: str, secs: float) -> None:
    """Hold the button for secs (blocking). QMP takes one client at a time,
    so connect only for the press."""
    qmp = Qmp(path)
    try:
        set_button(qmp, True)
        time.sleep(secs)
    finally:
        try:
            set_button(qmp, False)
        finally:
            qmp.close()


NXP_PATH = "/chardevs/vex-nxp"


def set_battery_async(path: str, volts: float, amps: float,
                      percent: float | None = None) -> threading.Thread:
    """The battery reading VEXos gets from its NXP co-processor (and, with
    percent, from the pack's own packets through it)."""
    def run() -> None:
        try:
            qmp = Qmp(path, timeout_s=0.5)
            try:
                qmp.cmd("qom-set", path=NXP_PATH, property="battery-volts", value=volts)
                qmp.cmd("qom-set", path=NXP_PATH, property="battery-amps", value=amps)
                if percent is not None:
                    qmp.cmd("qom-set", path=NXP_PATH, property="battery-percent",
                            value=percent)
            finally:
                qmp.close()
        except (OSError, RuntimeError, ValueError):
            pass                        # an older QEMU, or the brain is gone
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


def press_async(path: str, secs: float) -> threading.Thread:
    if not 0 < secs <= MAX_PRESS_S:
        raise ValueError(f"button press must be 0..{MAX_PRESS_S:g} s")
    def run() -> None:
        try:
            press(path, secs)
        except (OSError, RuntimeError, ValueError) as e:
            print(f"brain button: {e}", file=sys.stderr, flush=True)

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t
