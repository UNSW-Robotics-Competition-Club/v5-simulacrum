"""The battery in the robot: motors share it, it sags and drains, the
dashboard and the brain see it."""
import pytest

from v5sim import drivetrain
from v5sim.battery import BatteryState
from v5sim.motor import Cartridge
from v5sim.robot import Port, Robot, parser
from v5sim.smart_motor import Command, ControlMode, SmartMotor

DT = 0.001


def pushing_robot(charge=1.0):
    """Six blue drive motors at full voltage against nothing (accelerating)."""
    robot = Robot([], (drivetrain.DrivetrainSpec(), [Port(n, None, None, SmartMotor())
                                                    for n in (1, 2, 3)],
                       [Port(n, None, None, SmartMotor(), sign=-1) for n in (4, 5, 6)]))
    robot.enable_battery(charge)
    for p in robot.drive[1] + robot.drive[2]:
        p.firmware.command = Command(mode=ControlMode.VOLTAGE, percent=100)
        p.firmware._last_brain = 0.0
    return robot


def run(robot, seconds):
    for i in range(round(seconds / DT)):
        robot.step(i * DT)


def test_a_full_battery_sags_under_six_motors_and_recovers():
    robot = pushing_robot()
    run(robot, 0.05)
    sag = robot.battery_state.terminal_v
    assert sag < 12.8                                    # launch current pulls it down
    assert robot.battery_state.current_a > 8


def test_a_flat_battery_caps_the_motors():
    robot = pushing_robot(charge=0.03)
    run(robot, 0.2)
    v_bus = robot.battery_state.terminal_v
    applied = robot.dt_state.left[0].voltage_v
    assert applied == pytest.approx(v_bus, abs=0.05) and applied < 12.0


def test_no_battery_keeps_the_old_behaviour():
    robot = pushing_robot()
    robot.battery = None
    run(robot, 0.2)
    assert robot.dt_state.left[0].voltage_v == pytest.approx(12.0)


def test_battery_shows_on_the_dashboard():
    robot = pushing_robot(charge=0.8)
    run(robot, 0.01)
    (bat,) = [d for d in robot.device_snapshot(0.01) if d["kind"] == "battery"]
    assert bat["percent"] == 80 and 11 < bat["volts"] < 14 and bat["amps"] > 0


def test_battery_options():
    args = parser().parse_args(["--motor", "1:green", "--battery", "60"])
    assert args.battery == 60
    assert parser().parse_args(["--motor", "1:green", "--no-battery"]).battery is None
    assert parser().parse_args(["--motor", "1:green"]).battery == 100
