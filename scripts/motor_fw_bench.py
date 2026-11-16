#!/usr/bin/env python3
"""Bench test for the emulated motor firmware (sim/v5sim/firmware_motor.py).

Plays the brain against one vex-motor QEMU: the handshake (03, 04), the
SDK's init 56, heartbeats, then a voltage step and a velocity command,
with a free shaft (motor.step_shaft) as the plant. Prints the status
frames the real firmware sends back next to the plant state.

  cd sim && ~/venvs/sim/bin/python ../scripts/motor_fw_bench.py [--5w5]
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))

from v5sim import motor, protocol                       # noqa: E402
from v5sim.firmware_motor import FirmwareMotor          # noqa: E402
from v5sim.motor import Cartridge, MotorState           # noqa: E402

DT = 0.001
BRAKE_COAST = 1


def frame(*body: int) -> bytes:
    return protocol.build(bytes(body))


def cmd_velocity(counts_s: int) -> bytes:
    return protocol.build(b"\x56\x02" + struct.pack("<h", counts_s)
                          + bytes(6) + bytes([1, 0, BRAKE_COAST, 0]))


def cmd_voltage(percent: int) -> bytes:
    return protocol.build(b"\x56\x03" + bytes(6) + struct.pack("<b", percent)
                          + bytes([0, 8, 0, BRAKE_COAST, 0]))


def cmd_limits(voltage_mv: int, current_ma: int) -> bytes:
    return protocol.build(b"\x55" + bytes(7) + struct.pack("<HH", voltage_mv, current_ma))


def decode_status(f: bytes) -> dict:
    temp, cur, pos, vel, volts, flags, faults = struct.unpack("<xBHihhBB", f[:14])
    return dict(temp=temp, mA=cur, pos=pos, vel=vel, mV=volts,
                flags=f"{flags:02x}", faults=faults)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--5w5", dest="small", action="store_true", help="5.5 W motor")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--full", type=int, default=-100, help="second voltage step, %%")
    ap.add_argument("--stall", action="store_true",
                    help="lock the shaft and drive 100%% (current limit)")
    args = ap.parse_args()
    cart = Cartridge.MOTOR_5_5W if args.small else Cartridge.GREEN_18
    m = FirmwareMotor(cart, name="bench")
    state = MotorState()
    script = {0.30: cmd_voltage(50), 2.0: cmd_voltage(args.full), 3.5: cmd_velocity(1500),
              5.0: cmd_velocity(0)}
    if args.stall:
        script = {0.20: cmd_limits(12000, 1000), 0.30: cmd_voltage(100),
                  1.5: cmd_limits(12000, 1800)}
    queue: list[bytes] = []
    vels: list[int] = []
    next_print = 0.0
    last: dict | None = None
    t0 = time.monotonic()
    try:
        step = 0
        while (now := step * DT) < args.seconds:
            for t, cmd in list(script.items()):
                if now >= t:
                    queue.append(cmd)
                    del script[t]
            # Like the brain: one frame in the slot after each device frame.
            for f in m.due_frames(now, state):
                kind = f[0]
                if kind == 0x0A:
                    reply = frame(0x03, 1, 100, 0, 0, 0, 0, 0x0A)
                elif kind == 0x13:
                    reply = frame(0x04, 0, 5, 0)
                elif queue:
                    reply = queue.pop(0)
                else:
                    reply = frame(0x02, kind, 1, 1, 0xFF, 0, 0, 0, 0, 0, 1)
                if kind == 0x76:
                    last = decode_status(f)
                    if 4.0 <= now < 5.0:
                        vels.append(last["vel"])
                elif kind != 0x11 or now < 0.1:
                    print(f"{now:6.3f} < {f.hex(' ')}")
                if reply[0] != 0x02:
                    print(f"{now:6.3f} > {reply.hex(' ')}")
                m.on_brain_frame(reply, now)
            drive = m.drive(state, now, DT)
            if args.stall:
                state = motor.advance(state, drive, 0.0, DT, m.cartridge,
                                      limit_a=m.effective_current_limit_a)
            else:
                state = motor.step_shaft(state, drive, DT, m.cartridge,
                                         limit_a=m.effective_current_limit_a)
            if now >= next_print:
                d = "coast" if drive is None else f"{drive:6.2f}V"
                pins = ""
                if m.out is not None:
                    b = m.board
                    pins = (f" in1 {m.out.pin_fraction(b.pin_in1):.3f}"
                            f" in2 {m.out.pin_fraction(b.pin_in2):.3f}"
                            f" en {(m.out.levels >> b.pin_enable) & 1}")
                print(f"{now:6.3f} drive {d:>8} plant {state.velocity_rpm:7.1f}rpm "
                      f"{state.current_a:5.2f}A{pins}  status {last}")
                next_print = now + 0.25
            step += 1
            lag = t0 + now - time.monotonic()
            if lag > 0:
                time.sleep(lag)
    finally:
        m.close()
    if vels:
        mean = sum(vels) / len(vels)
        sd = (sum((v - mean) ** 2 for v in vels) / len(vels)) ** 0.5
        print(f"status velocity 4..5 s: mean {mean:.0f} sd {sd:.1f} "
              f"min {min(vels)} max {max(vels)} (n={len(vels)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
