import pytest

from v5sim import motor, protocol
from v5sim.motor import Cartridge, MotorState
from v5sim.smart_motor import (MAX_VEL_COUNTS, LinkMode, SmartMotor,
                               shaft_counts, shaft_velocity)

DT = 0.001


def hexframe(s: str) -> bytes:
    return bytes.fromhex(s)


def brain(payload: str) -> bytes:
    return protocol.build(bytes.fromhex(payload))


def handshake(m: SmartMotor) -> None:
    m.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"), 0.0)
    m.on_brain_frame(hexframe("04 00 05 00 00 00 00 00 00 00 00 00 00 00 5A 6E"), 0.0)


def run(m: SmartMotor, seconds: float, state: MotorState = MotorState()
        ) -> MotorState:
    t = 0.0
    for _ in range(round(seconds / DT)):
        m.on_brain_frame(brain("02 76 01 01 FF"), t)  # keep the watchdog fed
        state = motor.step_shaft(state, m.drive(state, t, DT), DT, m.cartridge)
        t += DT
    return state


def test_info_frame_matches_real_motor_layout():
    assert SmartMotor().info_frame() == hexframe(
        "0A 1F 00 00 02 00 00 40 78 56 34 12 00 00 3B 31")


def test_handshake_replies_and_link_modes():
    m = SmartMotor()
    assert m.due_frames(0.0, MotorState())[0][0] == 0x0A
    r = m.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"), 0.0)
    assert r == [hexframe("13 10 30 28 40 20 23 00 7F 00 7F 00 C0 09 52 0C")]
    assert m.link == LinkMode.IDLE and m.interval_s == pytest.approx(0.1)
    assert m.due_frames(0.1, MotorState())[0][0] == 0x11
    r = m.on_brain_frame(hexframe("04 00 05 00 00 00 00 00 00 00 00 00 00 00 5A 6E"), 0.1)
    assert r == [hexframe("02 04 00 00 00 00 00 00 00 00 00 00 00 00 5A 05")]
    assert m.link == LinkMode.DATA and m.interval_s == pytest.approx(0.005)
    assert m.due_frames(0.106, MotorState())[0][0] == 0x76


def test_bad_crc_is_ignored():
    m = SmartMotor()
    assert m.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 00 00"), 0.0) == []
    assert m.link == LinkMode.INFO


def test_status_frame_fields():
    m = SmartMotor(Cartridge.GREEN_18)
    # 2 output revs at green = 36 shaft revs = 1800 counts
    s = MotorState(position_rad=2 * 2 * 3.141592653589793, current_a=0.5,
                   voltage_v=6.0, temperature_c=31.0)
    f = m.status_frame(s)
    assert protocol.is_valid(f)
    assert f[1] == 30                                       # 5 degC steps
    assert int.from_bytes(f[2:4], "little") == 500          # mA
    assert int.from_bytes(f[4:8], "little", signed=True) == 1800
    assert int.from_bytes(f[10:12], "little", signed=True) == 6000
    assert f[12] & 0x02                                      # not moving


def test_velocity_command_reaches_target():
    m = SmartMotor(Cartridge.GREEN_18)
    handshake(m)
    m.on_brain_frame(hexframe("56 02 DC 05 00 00 00 00 00 00 01 00 01 00 5F C5"), 0.0)
    s = run(m, 1.0)
    assert shaft_velocity(s, m.cartridge) == pytest.approx(1500, rel=0.03)
    assert s.velocity_rpm == pytest.approx(100, rel=0.03)    # green: 1500 raw = 100 rpm


def test_voltage_command_runs_open_loop():
    m = SmartMotor(Cartridge.GREEN_18)
    handshake(m)
    m.on_brain_frame(hexframe("56 03 00 00 00 00 00 00 32 00 08 00 01 00 92 DE"), 0.0)
    s = run(m, 1.0)
    assert s.voltage_v == pytest.approx(6.0)
    assert 80 < s.velocity_rpm < 100                        # about half of 200 rpm


def test_position_command_settles_on_target():
    m = SmartMotor(Cartridge.GREEN_18)
    handshake(m)
    m.on_brain_frame(hexframe("56 04 DC 05 10 0E 00 00 00 00 05 00 01 00 FD BE"), 0.0)
    s = run(m, 3.0)
    assert shaft_counts(s, m.cartridge) == pytest.approx(3600, abs=15)
    assert m.status_frame(s)[12] & 0x04                      # at target


def test_hold_keeps_position_after_stop():
    m = SmartMotor(Cartridge.GREEN_18)
    handshake(m)
    m.on_brain_frame(brain("56 03 00 00 00 00 00 00 64 00 08 00 02"), 0.0)
    s = run(m, 0.5)
    m.on_brain_frame(brain("56 03 00 00 00 00 00 00 00 00 08 00 02"), 0.5)
    stopped = run(m, 1.0, s)
    held = run(m, 1.0, stopped)
    assert abs(shaft_velocity(held, m.cartridge)) < 5
    assert shaft_counts(held, m.cartridge) == pytest.approx(
        shaft_counts(stopped, m.cartridge), abs=10)


def test_hold_after_coast_holds_the_new_position():
    m = SmartMotor(Cartridge.GREEN_18)
    handshake(m)
    m.on_brain_frame(brain("56 03 00 00 00 00 00 00 00 00 08 00 02"), 0.0)
    run(m, 0.1)                                   # hold latches at 0
    m.on_brain_frame(brain("56 03 00 00 00 00 00 00 00 00 08 00 01"), 0.1)
    pushed = MotorState(position_rad=10.0)        # coasting, pushed by hand
    m.on_brain_frame(brain("56 03 00 00 00 00 00 00 00 00 08 00 02"), 0.2)
    held = run(m, 1.0, pushed)
    assert shaft_counts(held, m.cartridge) == pytest.approx(
        shaft_counts(pushed, m.cartridge), abs=10)


def test_watchdog_coasts_without_brain_traffic():
    m = SmartMotor()
    handshake(m)
    m.on_brain_frame(hexframe("56 03 00 00 00 00 00 00 32 00 08 00 01 00 92 DE"), 0.0)
    assert m.drive(MotorState(), 1.0, DT) == pytest.approx(6.0)
    assert m.drive(MotorState(), 2.5, DT) is None


def test_current_limit_config():
    m = SmartMotor()
    m.on_brain_frame(hexframe("55 00 00 00 00 00 00 00 00 00 D0 07 00 00 EF 75"), 0.0)
    assert m.current_limit_a == pytest.approx(2.0)


def test_velocity_is_clamped_to_motor_max():
    m = SmartMotor(Cartridge.BLUE_6)
    handshake(m)
    m.on_brain_frame(brain("56 02 B8 0B 00 00 00 00 00 00 01 00 01"), 0.0)  # 3000
    s = run(m, 1.0)
    assert shaft_velocity(s, m.cartridge) <= MAX_VEL_COUNTS * 1.01


def test_velocity_step_does_not_overshoot():
    """Integrator must not wind up while the output saturates (V5 motors
    settle on the target speed rather than overshooting by 20 %)."""
    m = SmartMotor(Cartridge.BLUE_6)
    handshake(m)
    m.on_brain_frame(brain("56 02 DC 05 00 00 00 00 00 00 01 00 01"), 0.0)  # 1500
    state, t, peak = MotorState(), 0.0, 0.0
    heavy = 20 * motor.SHAFT_INERTIA_KG_M2      # e.g. a drivetrain's load
    for _ in range(1500):
        m.on_brain_frame(brain("02 76 01 01 FF"), t)
        state = motor.step_shaft(state, m.drive(state, t, DT), DT, m.cartridge,
                                 inertia_kg_m2=heavy)
        peak = max(peak, shaft_velocity(state, m.cartridge))
        t += DT
    assert peak <= 1500 * 1.05
    assert shaft_velocity(state, m.cartridge) == pytest.approx(1500, rel=0.03)


def test_motor_reannounces_after_brain_silence():
    m = SmartMotor()
    handshake(m)
    m.on_brain_frame(brain("56 03 00 00 00 00 00 00 32 00 08 00 01"), 0.0)
    assert m.link == LinkMode.DATA
    assert m.due_frames(1.0, MotorState())[0][0] != 0x0A
    frames = m.due_frames(2.6, MotorState())          # > 2 s without the brain
    assert m.link == LinkMode.INFO and frames == [m.info_frame()]
    assert m.drive(MotorState(), 2.6, DT) is None


def test_idle_status_frame_matches_the_real_motor():
    # motor-protocol.md section 7 "76 idle": flags 0x06 (still, at target 0)
    assert SmartMotor().status_frame(MotorState()) == hexframe(
        "76 19 00 00 00 00 00 00 00 00 00 00 06 00 20 A1")
