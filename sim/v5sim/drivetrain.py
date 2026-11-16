"""Differential-drive robot dynamics for V5 drivetrains.

Each side is a group of identical motors on one shaft through a fixed
external ratio to the wheels; every motor gets its own drive (volts, or None
to coast), as each sits on its own smart port. Wheels roll without slip;
the chassis has mass and yaw inertia, plus rolling resistance and a
speed-proportional drag so it coasts to a stop. Integration is
semi-implicit Euler at a small fixed step.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Sequence

from . import motor
from .motor import Cartridge, Drive, MotorState

INCH = 0.0254
G = 9.81


@dataclass(frozen=True)
class DrivetrainSpec:
    motors_per_side: int = 3
    cartridge: Cartridge = Cartridge.BLUE_6
    # Motor shaft turns per wheel turn: 36T on the motor driving 48T on the
    # wheel, the common 600 rpm -> 450 rpm drive.
    external_ratio: float = 48 / 36
    wheel_diameter_m: float = 3.25 * INCH
    track_width_m: float = 12.0 * INCH
    mass_kg: float = 6.5
    yaw_inertia_kg_m2: float = 0.25
    rolling_resistance: float = 0.02     # fraction of weight per side
    drag_n_per_m_s: float = 2.0
    turn_scrub_nm_per_rad_s: float = 0.6
    traction_mu: float = 0.8             # wheels on foam tiles
    wheel_spin_inertia_kg_m2: float = 0.002   # one side's wheels + gearing, at the wheel
    # None: tank (differential) drive. "x" / "mecanum": a four-wheel
    # holonomic drive (holonomic.py), each side's motors listed front, back.
    holonomic: str | None = None

    @property
    def wheel_radius_m(self) -> float:
        return self.wheel_diameter_m / 2


@dataclass(frozen=True)
class Pose:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0     # 0 = +x, counter-clockwise positive


@dataclass(frozen=True)
class DrivetrainState:
    pose: Pose = Pose()
    v_m_s: float = 0.0           # forward speed
    omega_rad_s: float = 0.0     # yaw rate, counter-clockwise positive
    strafe_m_s: float = 0.0      # sideways speed, + = right (holonomic only)
    left: tuple[MotorState, ...] = ()
    right: tuple[MotorState, ...] = ()
    # motor rad/s each side spins beyond rolling while pushing a wall
    left_slip: float = 0.0
    right_slip: float = 0.0


def initial_state(spec: DrivetrainSpec, pose: Pose = Pose()) -> DrivetrainState:
    side = tuple(MotorState() for _ in range(spec.motors_per_side))
    return DrivetrainState(pose=pose, left=side, right=side)


def _motor_speed(spec: DrivetrainSpec, wheel_m_s: float) -> float:
    return wheel_m_s / spec.wheel_radius_m * spec.external_ratio


def _side_force(spec: DrivetrainSpec, drives: Sequence[Drive],
                wheel_m_s: float, limits: Sequence[float]) -> float:
    """Ground force from one side's motors at the current wheel speed."""
    w = _motor_speed(spec, wheel_m_s)
    torque = sum(motor.net_output_torque(
        motor.motor_current(d, w, spec.cartridge, lim), w, spec.cartridge)
        for d, lim in zip(drives, limits))
    return torque * spec.external_ratio / spec.wheel_radius_m


def _resist(force: float, speed: float, limit: float) -> float:
    """Coulomb resistance: opposes motion, or holds still up to its limit."""
    if abs(speed) > 1e-4:
        return -math.copysign(limit, speed)
    return -max(-limit, min(limit, force))


def step(spec: DrivetrainSpec, state: DrivetrainState,
         left: Sequence[Drive], right: Sequence[Drive], dt: float,
         limits_a: Sequence[float] | None = None) -> DrivetrainState:
    """Advance the robot by dt seconds with per-motor drives."""
    n = spec.motors_per_side
    if len(left) != n or len(right) != n:
        raise ValueError(f"need {n} drives per side")
    lim = list(limits_a) if limits_a else [motor.CURRENT_LIMIT_A] * (2 * n)
    lim_l, lim_r = lim[:n], lim[n:]

    half = spec.track_width_m / 2
    f_left = _side_force(spec, left, state.v_m_s - state.omega_rad_s * half, lim_l)
    f_right = _side_force(spec, right, state.v_m_s + state.omega_rad_s * half, lim_r)

    roll = spec.rolling_resistance * spec.mass_kg * G
    f_drive = f_left + f_right
    f_net = (f_drive - spec.drag_n_per_m_s * state.v_m_s
             + _resist(f_drive, state.v_m_s, roll))
    tau_drive = (f_right - f_left) * half
    tau_net = (tau_drive - spec.turn_scrub_nm_per_rad_s * state.omega_rad_s
               + _resist(tau_drive, state.omega_rad_s, roll * half))

    v = state.v_m_s + f_net / spec.mass_kg * dt
    omega = state.omega_rad_s + tau_net / spec.yaw_inertia_kg_m2 * dt

    heading = state.pose.heading_rad + omega * dt
    pose = Pose(
        x_m=state.pose.x_m + v * math.cos(heading) * dt,
        y_m=state.pose.y_m + v * math.sin(heading) * dt,
        heading_rad=heading,
    )
    w_left = _motor_speed(spec, v - omega * half)
    w_right = _motor_speed(spec, v + omega * half)
    return replace(
        state,
        pose=pose,
        v_m_s=v,
        omega_rad_s=omega,
        left=tuple(motor.advance(m, d, w_left, dt, spec.cartridge, li)
                   for m, d, li in zip(state.left, left, lim_l)),
        right=tuple(motor.advance(m, d, w_right, dt, spec.cartridge, li)
                    for m, d, li in zip(state.right, right, lim_r)),
    )


def simulate(spec: DrivetrainSpec, state: DrivetrainState, left_v: Drive,
             right_v: Drive, seconds: float, dt: float = 0.001
             ) -> DrivetrainState:
    """Run with every motor on a side at the same drive (for tests/tools)."""
    left = [left_v] * spec.motors_per_side
    right = [right_v] * spec.motors_per_side
    for _ in range(round(seconds / dt)):
        state = step(spec, state, left, right, dt)
    return state


def _slipping_side(spec: DrivetrainSpec, motors: tuple[MotorState, ...],
                   drives: Sequence[Drive], ground_m_s: float, prev_slip: float,
                   dt: float, limits: Sequence[float]
                   ) -> tuple[tuple[MotorState, ...], float]:
    """One side's wheels against a wall: the motors' torque fights traction
    (mu * half the weight at the wheel's radius). Below it the wheels roll
    with the ground; above it they slip and spin up, and the encoders
    count the spin. Each motor's position moves by what it spun beyond the
    no-slip speed the plant step gave it."""
    ratio, r = spec.external_ratio, spec.wheel_radius_m
    w_ground = _motor_speed(spec, ground_m_s)            # motor rad/s if rolling
    w = w_ground + prev_slip
    torque = sum(motor.net_output_torque(motor.motor_current(d, w, spec.cartridge, lim),
                                         w, spec.cartridge)
                 for d, lim in zip(drives, limits)) * ratio   # at the wheel
    friction = spec.traction_mu * spec.mass_kg * G / 2 * r
    slip = (w - w_ground) / ratio                         # wheel rad/s of slip
    if abs(slip) < 1e-3 and abs(torque) <= friction:
        w_new = w_ground                                  # static: no slip
    else:
        resist = -math.copysign(friction, slip if abs(slip) >= 1e-3 else torque)
        w_new = w + (torque + resist) / spec.wheel_spin_inertia_kg_m2 * ratio * dt
        if (w_new - w_ground) * (w - w_ground) < 0 and abs(torque) <= friction:
            w_new = w_ground                              # slip ended this step
    out = []
    for m, d, lim in zip(motors, drives, limits):
        extra = (w_new - m.velocity_rad_s) * dt
        out.append(replace(m, velocity_rad_s=w_new, position_rad=m.position_rad + extra,
                           current_a=motor.motor_current(d, w_new, spec.cartridge, lim)))
    return tuple(out), w_new - w_ground


def constrain_to_square(spec: DrivetrainSpec, state: DrivetrainState,
                        half_side_m: float, robot_half_m: float,
                        drives: tuple[Sequence[Drive], Sequence[Drive]] | None = None,
                        dt: float = 0.0, limits_a: Sequence[float] | None = None
                        ) -> DrivetrainState:
    """Keep the robot inside a square field centred on the origin.

    Driving into a wall stops the forward motion; with the step's drives
    given, the wheels then slip against the tiles when the motors out-push
    traction (so encoder odometry drifts, as on a real field), else they
    stall. Turning in place and backing away still work.
    """
    lim = half_side_m - robot_half_m
    p = state.pose
    x, y = max(-lim, min(lim, p.x_m)), max(-lim, min(lim, p.y_m))
    if (x, y) == (p.x_m, p.y_m):
        if state.left_slip or state.right_slip:
            return replace(state, left_slip=0.0, right_slip=0.0)   # off the wall
        return state
    vx = state.v_m_s * math.cos(p.heading_rad)
    vy = state.v_m_s * math.sin(p.heading_rad)
    outward = (p.x_m - x) * vx > 0 or (p.y_m - y) * vy > 0
    if not outward:
        return replace(state, pose=replace(p, x_m=x, y_m=y), left_slip=0.0,
                       right_slip=0.0)
    half = spec.track_width_m / 2
    if drives is None:
        w_left = _motor_speed(spec, -state.omega_rad_s * half)
        w_right = _motor_speed(spec, state.omega_rad_s * half)
        left = tuple(replace(m, velocity_rad_s=w_left) for m in state.left)
        right = tuple(replace(m, velocity_rad_s=w_right) for m in state.right)
    else:
        n = spec.motors_per_side
        lim = list(limits_a) if limits_a else [motor.CURRENT_LIMIT_A] * (2 * n)
        left, ls = _slipping_side(spec, state.left, drives[0],
                                  -state.omega_rad_s * half, state.left_slip, dt, lim[:n])
        right, rs = _slipping_side(spec, state.right, drives[1],
                                   state.omega_rad_s * half, state.right_slip, dt, lim[n:])
        return replace(state, pose=replace(p, x_m=x, y_m=y), v_m_s=0.0,
                       left=left, right=right, left_slip=ls, right_slip=rs)
    return replace(state, pose=replace(p, x_m=x, y_m=y), v_m_s=0.0,
                   left=left, right=right)
