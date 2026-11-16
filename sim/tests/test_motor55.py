"""5.5 W motor against notes/motor55-protocol.md section 5."""
import math

import pytest

from v5sim import motor, protocol
from v5sim.motor import Cartridge, MotorState
from v5sim.smart_motor import SmartMotor


def H(s: str) -> bytes:
    return bytes.fromhex(s)


def linked(m):
    m.on_brain_frame(H("03 05 64 00 00 00 00 0A 00 00 00 00 00 00 29 7D"), 0.0)
    m.on_brain_frame(H("04 00 05 00 00 00 00 00 00 00 00 00 00 00 5A 6E"), 0.0)
    return m


def test_55w_info_config_and_idle_frames():
    m = SmartMotor(Cartridge.MOTOR_5_5W)
    assert m.info_frame() == H("0A 05 00 00 05 00 00 40 78 56 34 12 00 00 B0 E3")
    assert m.on_brain_frame(H("03 05 64 00 00 00 00 0A 00 00 00 00 00 00 29 7D"), 0.0) == \
        [H("13 10 30 28 40 20 23 00 7F 00 7F 00 C0 09 52 0C")]
    assert m.status_frame(MotorState()) == H("76 19 00 00 00 00 00 00 00 00 00 00 86 00 3B 39")


def test_55w_reports_green_cartridge_units():
    m = SmartMotor(Cartridge.MOTOR_5_5W)
    one_rev_200rpm = MotorState(position_rad=2 * math.pi, velocity_rad_s=200 / 60 * 2 * math.pi)
    f = m.status_frame(one_rev_200rpm)
    assert int.from_bytes(f[4:8], "little", signed=True) == 900
    assert int.from_bytes(f[8:10], "little", signed=True) == pytest.approx(3000, abs=2)


def test_55w_is_weaker_than_an_11w_green():
    stall = lambda c: (motor.output_torque(motor.motor_current(12.0, 0.0, c), c)
                       - motor.gear_friction_nm(c)[0])
    assert stall(Cartridge.MOTOR_5_5W) == pytest.approx(0.5, rel=0.05)
    assert stall(Cartridge.GREEN_18) == pytest.approx(1.05, rel=0.05)
    assert Cartridge.MOTOR_5_5W.free_rpm == pytest.approx(200)

