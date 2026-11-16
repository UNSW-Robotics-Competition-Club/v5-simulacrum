import math

import pytest

from v5sim import drivetrain, motor
from v5sim.drivetrain import DrivetrainSpec
from v5sim.motor import Cartridge, MotorState


@pytest.mark.parametrize("cart, rpm", [(Cartridge.RED_36, 100),
                                       (Cartridge.GREEN_18, 200),
                                       (Cartridge.BLUE_6, 600)])
def test_free_speed_only_overcomes_the_gearbox(cart, rpm):
    """At the published free speed the motor draws just what its gearbox's
    friction takes, and delivers nothing."""
    w = rpm * 2 * math.pi / 60
    current = motor.motor_current(motor.NOMINAL_V, w, cart)
    assert current == pytest.approx(motor.FREE_CURRENT_A, rel=1e-6)
    assert motor.net_output_torque(current, w, cart) == pytest.approx(0, abs=1e-9)


def test_red_cartridge_stall_torque_matches_spec():
    current = motor.motor_current(motor.NOMINAL_V, 0.0, Cartridge.RED_36)
    sliding, _ = motor.gear_friction_nm(Cartridge.RED_36)
    torque = motor.output_torque(current, Cartridge.RED_36) - sliding
    assert torque == pytest.approx(motor.STALL_TORQUE_RED_NM, rel=1e-6)
    assert current <= motor.CURRENT_LIMIT_A


def test_current_is_clamped_when_back_driven():
    # Full reverse voltage while spinning forward exceeds the limit.
    current = motor.motor_current(-motor.MAX_V, 60.0, Cartridge.BLUE_6)
    assert current == -motor.CURRENT_LIMIT_A


def test_voltage_command_is_clamped():
    s = motor.advance(MotorState(), 20.0, 0.0, 0.01, Cartridge.GREEN_18)
    assert s.voltage_v == motor.MAX_V


def test_motor_heats_under_stall_and_cools_at_rest():
    hot = MotorState()
    for _ in range(60_000):
        hot = motor.advance(hot, motor.NOMINAL_V, 0.0, 0.001, Cartridge.RED_36)
    assert hot.temperature_c > motor.AMBIENT_C + 5
    cool = hot
    for _ in range(60_000):
        cool = motor.advance(cool, 0.0, 0.0, 0.001, Cartridge.RED_36)
    assert cool.temperature_c < hot.temperature_c


def test_straight_drive_reaches_near_theoretical_top_speed():
    spec = DrivetrainSpec()
    end = drivetrain.simulate(spec, drivetrain.initial_state(spec), 12.0, 12.0, 3.0)
    wheel_rpm = spec.cartridge.free_rpm / spec.external_ratio
    top = wheel_rpm * 2 * math.pi / 60 * spec.wheel_radius_m
    assert 0.8 * top < end.v_m_s < top
    assert end.pose.heading_rad == pytest.approx(0, abs=1e-9)
    assert end.pose.y_m == pytest.approx(0, abs=1e-9)
    assert end.pose.x_m > 1.0


def test_opposite_voltages_turn_in_place():
    end = drivetrain.simulate(DrivetrainSpec(), drivetrain.initial_state(DrivetrainSpec()), -6.0, 6.0, 1.0)
    assert end.omega_rad_s > 1.0
    assert math.hypot(end.pose.x_m, end.pose.y_m) < 1e-6


def test_robot_at_rest_stays_put_with_no_voltage():
    end = drivetrain.simulate(DrivetrainSpec(), drivetrain.initial_state(DrivetrainSpec()), 0.0, 0.0, 1.0)
    assert end.pose == drivetrain.Pose()


def test_robot_coasts_to_a_stop():
    spec = DrivetrainSpec()
    moving = drivetrain.simulate(spec, drivetrain.initial_state(spec), 12.0, 12.0, 1.0)
    stopped = drivetrain.simulate(spec, moving, 0.0, 0.0, 3.0)
    assert abs(stopped.v_m_s) < 0.01


def test_motor_encoders_track_wheel_travel():
    spec = DrivetrainSpec()
    end = drivetrain.simulate(spec, drivetrain.initial_state(spec), 8.0, 8.0, 1.0)
    wheel_turns = end.pose.x_m / (2 * math.pi * spec.wheel_radius_m)
    motor_turns = end.left[0].position_rad / (2 * math.pi)
    assert motor_turns == pytest.approx(wheel_turns * spec.external_ratio, rel=1e-3)


def test_coast_draws_no_current_and_brake_does():
    w = 20.0
    assert motor.motor_current(None, w, Cartridge.GREEN_18) == 0.0
    assert motor.motor_current(0.0, w, Cartridge.GREEN_18) < 0.0


def test_free_shaft_spins_up_to_free_speed():
    s = MotorState()
    for _ in range(2000):
        s = motor.step_shaft(s, motor.NOMINAL_V, 0.001, Cartridge.GREEN_18)
    assert 0.9 * 200 < s.velocity_rpm < 200


def test_default_drivetrain_gearing_is_600rpm_to_450rpm_wheels():
    spec = DrivetrainSpec()
    wheel_rpm = spec.cartridge.free_rpm / spec.external_ratio
    assert wheel_rpm == pytest.approx(450)


def push_wall(volts: float, seconds: float):
    spec = DrivetrainSpec()
    half, r = 1.8288, 0.2286
    st = drivetrain.initial_state(spec)
    drives = ([volts] * 3, [volts] * 3)
    for _ in range(round(seconds / 0.001)):
        st = drivetrain.step(spec, st, *drives, 0.001)
        st = drivetrain.constrain_to_square(spec, st, half, r, drives, 0.001)
    return spec, st, half - r


def test_walls_stop_the_robot_at_the_field_edge():
    spec, st, wall = push_wall(12.0, 4.0)
    assert st.pose.x_m == pytest.approx(wall)
    assert st.v_m_s == pytest.approx(0, abs=1e-9)


def test_wheels_spin_against_the_wall_at_full_power():
    # 6 blue motors out-push the tiles' traction: the wheels slip, so the
    # encoders keep counting while the robot stays put (odometry drifts)
    spec, st, wall = push_wall(12.0, 4.0)
    free = 600 / 60 * 2 * math.pi                   # blue output, rad/s
    w = st.left[0].velocity_rad_s
    assert 0.1 * free < w < 0.5 * free
    _, before, _ = push_wall(12.0, 3.0)
    assert st.left[0].position_rad - before.left[0].position_rad == pytest.approx(w, rel=0.1)


def test_gentle_push_against_the_wall_doesnt_slip():
    _, st, _ = push_wall(3.0, 6.0)                 # 3 V reaches the wall in ~5 s
    assert abs(st.left[0].velocity_rad_s) < 0.5


def test_walls_let_the_robot_back_away():
    spec = DrivetrainSpec()
    st = drivetrain.initial_state(spec, drivetrain.Pose(x_m=1.6))
    st = drivetrain.simulate(spec, st, -6.0, -6.0, 0.5)
    st = drivetrain.constrain_to_square(spec, st, 1.8288, 0.2286)
    assert st.pose.x_m < 1.6 and st.v_m_s < 0

def test_gearbox_holds_a_stopped_shaft_below_breakaway():
    cart = Cartridge.GREEN_18
    sliding, breakaway = motor.gear_friction_nm(cart)
    assert motor.step_velocity(0.0, 0.9 * breakaway, 2e-4, 0.001, cart) == 0.0
    assert motor.step_velocity(0.0, 1.1 * breakaway, 2e-4, 0.001, cart) > 0.0
    assert motor.step_velocity(0.0, -1.1 * breakaway, 2e-4, 0.001, cart) < 0.0


def test_gearbox_friction_stops_a_coasting_shaft_without_reversing_it():
    cart, w = Cartridge.GREEN_18, 0.5
    for _ in range(1000):
        w = motor.step_velocity(w, 0.0, 2e-4, 0.001, cart)
        assert w >= 0.0
    assert w == 0.0


def test_one_volt_barely_moves_a_motor_and_half_a_volt_does_not():
    """About 1 V breaks a stalled motor loose (motor.BREAKAWAY_FACTOR)."""
    for volts, moves in ((0.5, False), (1.5, True)):
        s = MotorState()
        for _ in range(500):
            s = motor.step_shaft(s, volts, 0.001, Cartridge.GREEN_18)
        assert (abs(s.velocity_rad_s) > 0) == moves


def test_a_stopped_drivetrain_stays_put_under_a_tiny_drive():
    spec = DrivetrainSpec()
    st = drivetrain.simulate(spec, drivetrain.initial_state(spec), 0.3, 0.3, 1.0)
    assert st.v_m_s == 0.0 and st.pose.x_m == 0.0
