"""Loaded mechanisms (mechanism.py): an arm with gravity and hard stops on a
motor shaft."""
import math

import pytest

from v5sim import motor
from v5sim.mechanism import ArmLoad, arm_angle_deg, gravity_torque_nm, step_arm
from v5sim.motor import Cartridge, MotorState

DT = 0.001
# a red motor through 5:1 to a 12 in arm carrying 1.5 kg, horizontal at 0,
# stops at -30 and +90 degrees
ARM = ArmLoad(ratio=5.0, length_m=12 * 0.0254, mass_kg=1.5, start_deg=0.0,
              lo_deg=-30.0, hi_deg=90.0)


def run(load, drive, seconds, state=MotorState(), cart=Cartridge.RED_36):
    for _ in range(round(seconds / DT)):
        state = step_arm(state, drive, DT, cart, load)
    return state


def test_gravity_torque_is_largest_level_and_zero_upright():
    assert gravity_torque_nm(ARM, 0.0) == pytest.approx(-1.5 * 9.81 * 12 * 0.0254, rel=1e-6)
    assert gravity_torque_nm(ARM, 90.0) == pytest.approx(0.0, abs=1e-9)


def test_a_coasting_arm_falls_to_its_lower_stop_and_stays():
    st = run(ARM, None, 2.0)
    assert arm_angle_deg(ARM, st) == pytest.approx(-30.0, abs=0.01)
    assert st.velocity_rad_s == pytest.approx(0.0, abs=1e-9)


def test_full_power_lifts_it_to_the_upper_stop_where_the_motor_stalls():
    st = run(ARM, 12.0, 3.0)
    assert arm_angle_deg(ARM, st) == pytest.approx(90.0, abs=0.01)
    assert st.velocity_rad_s == pytest.approx(0.0, abs=1e-9)
    assert abs(st.current_a) > 1.5                            # pushing the stop


def test_holding_level_needs_the_voltage_gravity_asks_for():
    """Stall voltage to hold the arm level: torque m g L / ratio at the motor."""
    need_nm = 1.5 * 9.81 * 12 * 0.0254 / 5.0
    amps = need_nm / motor.output_torque(1.0, Cartridge.RED_36)
    volts = amps * Cartridge.RED_36.r_ohm
    held = run(ARM, volts, 1.0)
    assert arm_angle_deg(ARM, held) == pytest.approx(0.0, abs=1.0)
    sag = run(ARM, volts * 0.8, 1.0)
    assert arm_angle_deg(ARM, sag) < -1.0                     # not enough: it sags


def test_a_heavier_arm_rises_slower():
    light = run(ARM, 12.0, 0.4)
    heavy = run(ArmLoad(5.0, 12 * 0.0254, 3.0, 0.0, -30.0, 90.0), 12.0, 0.4)
    assert arm_angle_deg(ARM, heavy) < arm_angle_deg(ARM, light)


def test_bad_arms_are_refused():
    with pytest.raises(ValueError):
        ArmLoad(0.0, 0.3, 1.0, 0.0, -30.0, 90.0)
    with pytest.raises(ValueError):
        ArmLoad(5.0, 0.3, 1.0, 100.0, -30.0, 90.0)            # starts outside its stops


def test_arm_option_attaches_to_a_motor():
    from v5sim.robot import parser
    args = parser().parse_args(["--motor", "8:red", "--arm", "8:5:12:1.5:-30:-30:90"])
    (port, load), = args.arm
    assert port == 8 and load.ratio == 5.0 and load.mass_kg == 1.5
    assert load.length_m == pytest.approx(12 * 0.0254)
    assert (load.start_deg, load.lo_deg, load.hi_deg) == (-30.0, -30.0, 90.0)


def test_robot_drives_an_arm_and_reports_its_angle():
    from v5sim.robot import Port, Robot
    from v5sim.smart_motor import Command, ControlMode, SmartMotor
    fw = SmartMotor(Cartridge.RED_36)
    robot = Robot([Port(8, None, None, fw)], None)
    robot.arms = {8: ARM}
    fw.command = Command(mode=ControlMode.VOLTAGE, percent=100)
    fw._last_brain = 0.0
    for i in range(3000):
        robot.step(i * DT)
    (m,) = robot.device_snapshot(3.0)
    assert m["arm_deg"] == pytest.approx(90.0, abs=0.01)     # lifted to its stop
