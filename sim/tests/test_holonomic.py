import math

import pytest

from v5sim import drivetrain, holonomic
from v5sim.drivetrain import DrivetrainSpec, Pose
from v5sim.motor import Cartridge

DT = 0.001
X_SPEC = DrivetrainSpec(motors_per_side=2, cartridge=Cartridge.GREEN_18,
                        external_ratio=0.6, holonomic="x")


def run(spec, lf, lb, rf, rb, seconds, state=None):
    """Plant drives (volts, + = the wheel rolls forward): left (LF, LB),
    right (RF, RB), as robot.py hands them over."""
    st = state or drivetrain.initial_state(spec)
    for _ in range(round(seconds / DT)):
        st = holonomic.step(spec, st, [lf, lb], [rf, rb], DT)
    return st


def test_all_wheels_forward_drives_straight_ahead():
    st = run(X_SPEC, 12, 12, 12, 12, 2.0)
    assert st.pose.x_m > 1.0                         # heading 0 = +x
    assert st.pose.y_m == pytest.approx(0, abs=1e-6)
    assert st.pose.heading_rad == pytest.approx(0, abs=1e-6)
    assert st.strafe_m_s == pytest.approx(0, abs=1e-6)


def test_the_strafe_pattern_slides_right_without_turning():
    st = run(X_SPEC, 12, -12, -12, 12, 2.0)          # mikLib: + strafe
    assert st.strafe_m_s > 0.5 and st.v_m_s == pytest.approx(0, abs=1e-6)
    assert st.pose.y_m < -1.0                        # right of +x is -y
    assert st.pose.x_m == pytest.approx(0, abs=1e-6)
    assert st.pose.heading_rad == pytest.approx(0, abs=1e-6)


def test_left_forward_right_back_turns_clockwise_on_the_spot():
    st = run(X_SPEC, 12, 12, -12, -12, 0.5)
    assert st.omega_rad_s < -1.0                     # plant yaw is CCW-positive
    assert math.hypot(st.pose.x_m, st.pose.y_m) == pytest.approx(0, abs=1e-6)


def test_x_drive_goes_root_two_faster_than_its_wheels_mecanum_does_not():
    wheel_m_s = 200 / 0.6 / 60 * math.pi * X_SPEC.wheel_diameter_m   # free speed
    x = run(X_SPEC, 12, 12, 12, 12, 3.0)
    mec = run(DrivetrainSpec(**{**X_SPEC.__dict__, "holonomic": "mecanum"}),
              12, 12, 12, 12, 3.0)
    # sqrt(2) at free speed; drag loads the faster X-drive a little more
    assert 1.25 < x.v_m_s / mec.v_m_s <= math.sqrt(2)
    assert mec.v_m_s == pytest.approx(wheel_m_s, rel=0.15)


def test_wheel_motors_turn_with_the_ground():
    st = run(X_SPEC, 12, -12, -12, 12, 2.0)
    lf, lb = st.left
    rf, rb = st.right
    assert lf.velocity_rad_s > 0 and rb.velocity_rad_s > 0
    assert lb.velocity_rad_s < 0 and rf.velocity_rad_s < 0
    assert lf.velocity_rad_s == pytest.approx(-lb.velocity_rad_s, rel=1e-3)


def test_motion_carries_on_across_a_turn_in_the_field_frame():
    # strafing right while facing +y moves the robot towards +x
    st = drivetrain.initial_state(X_SPEC, Pose(0, 0, math.pi / 2))
    st = run(X_SPEC, 12, -12, -12, 12, 1.0, st)
    assert st.pose.x_m > 0.3 and st.pose.y_m == pytest.approx(0, abs=1e-4)


def test_walls_stop_the_robot_in_any_direction():
    st = drivetrain.initial_state(X_SPEC)
    for _ in range(4000):                            # strafe right into -y wall
        st = holonomic.constrain_to_square(
            X_SPEC, holonomic.step(X_SPEC, st, [12, -12], [-12, 12], DT), 1.0, 0.2)
    assert st.pose.y_m == pytest.approx(-0.8)
    assert st.strafe_m_s == pytest.approx(0, abs=1e-6)
    # still free to drive along the wall
    for _ in range(500):
        st = holonomic.constrain_to_square(
            X_SPEC, holonomic.step(X_SPEC, st, [12, 12], [12, 12], DT), 1.0, 0.2)
    assert st.pose.x_m > 0.2 and st.pose.y_m == pytest.approx(-0.8)


def test_sideways_tracker_reads_strafing():
    from dataclasses import replace
    from v5sim.robot import INCH, Robot, _rotation_arg
    robot = Robot([], None)
    robot.dt_state = replace(drivetrain.initial_state(X_SPEC), strafe_m_s=0.5)
    side = robot.tracker_sample(_rotation_arg("7:strafe:2:0:-2")[1])
    fwd = robot.tracker_sample(_rotation_arg("8:track:2")[1])
    for _ in range(1000):
        robot._advance_trackers(DT)
    assert side().shaft_deg == pytest.approx(0.5 / (math.pi * 2 * INCH) * 360, rel=1e-3)
    assert fwd().shaft_deg == pytest.approx(0)


def test_holonomic_option_needs_two_motors_a_side(capsys):
    from v5sim.robot import drive_spec, main, parser
    spec = drive_spec(parser().parse_args(["--drive", "14,13:12,11:green",
                                           "--holonomic", "x"]))
    assert spec.holonomic == "x" and spec.motors_per_side == 2
    with pytest.raises(SystemExit):
        main(["--drive", "1,2,3:4,5,6:blue", "--holonomic", "x",
              "--sock", "/nonexistent/{port}.sock"])
    assert "--holonomic" in capsys.readouterr().err
