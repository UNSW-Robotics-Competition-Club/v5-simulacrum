"""Thermal derating of the V5 motor: the current limit halves at each 5 C
step from 55 C, and the status faults byte reports the levels."""
from dataclasses import replace

import pytest

from v5sim.motor import MotorState
from v5sim.smart_motor import (FAULT_OVER_TEMP, SmartMotor, thermal_derate)


@pytest.mark.parametrize("temp, factor", [(25, 1.0), (54.9, 1.0), (55, 0.5), (59, 0.5),
                                          (60, 0.25), (65, 0.125), (70, 0.0), (90, 0.0)])
def test_derate_ladder(temp, factor):
    assert thermal_derate(temp) == factor


def test_the_limit_the_physics_uses_follows_the_motor_temperature():
    m = SmartMotor()
    assert m.effective_current_limit_a == pytest.approx(m.current_limit_a)
    m.drive(MotorState(temperature_c=61.0), 0.0, 0.001)
    assert m.effective_current_limit_a == pytest.approx(m.current_limit_a * 0.25)
    m.drive(MotorState(temperature_c=30.0), 0.0, 0.001)
    assert m.effective_current_limit_a == pytest.approx(m.current_limit_a)


@pytest.mark.parametrize("temp, bits", [(50, 0x00), (55, 0x11), (60, 0x31), (66, 0x71)])
def test_faults_byte_reports_thermal_levels(temp, bits):
    m = SmartMotor()
    st = MotorState(temperature_c=temp)
    faults = m.faults(st)
    assert faults & 0x71 == bits
    assert bool(faults & FAULT_OVER_TEMP) == (temp >= 55)


def test_a_hot_motor_pushes_less():
    """Through the robot: a motor at 62 C gets a quarter of its current."""
    from v5sim import drivetrain
    from v5sim.robot import Port, Robot
    from v5sim.smart_motor import Command, ControlMode
    robot = Robot([], (drivetrain.DrivetrainSpec(motors_per_side=1),
                       [Port(1, None, None, SmartMotor())],
                       [Port(2, None, None, SmartMotor(), sign=-1)]))
    for p in robot.drive[1] + robot.drive[2]:
        p.firmware.command = Command(mode=ControlMode.VOLTAGE, percent=100)
        p.firmware._last_brain = 0.0                       # the brain is talking
    st = robot.dt_state
    robot.dt_state = replace(st, left=(replace(st.left[0], temperature_c=62.0),),
                             right=(replace(st.right[0], temperature_c=62.0),))
    for i in range(3):
        robot.step(i * 0.001)
    amps = abs(robot.dt_state.left[0].current_a)
    assert 0 < amps <= SmartMotor().current_limit_a * 0.25 + 1e-6
