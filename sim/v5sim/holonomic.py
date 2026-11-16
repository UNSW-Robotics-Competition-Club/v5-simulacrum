"""Four-wheel holonomic drivetrains: X-drive (omni wheels at 45 degrees on
the corners) and mecanum.

Each wheel pushes the chassis along its own direction at 45 degrees to the
frame: front-left and back-right roll towards forward-right, front-right
and back-left towards forward-left, so all wheels forward drives ahead,
LF+ LB- RF- RB+ slides right and left+ right- turns clockwise, as VEXcode
and mikLib drive them. Rollers let the wheels slide freely across their
direction. A mecanum wheel's rim moves sqrt(2) times faster than an X-drive
omni wheel's for the same chassis motion, so an X-drive tops out sqrt(2)
faster than its wheels and a mecanum drive at their speed.

The chassis is integrated in the field frame (semi-implicit Euler, like
drivetrain.py), with the same mass, rolling resistance, drag and turn scrub.
Walls stop the motion into them; the wheels then turn with what motion is
left (no wall slip model, unlike the tank drive).
"""
from __future__ import annotations

import math
from dataclasses import replace
from typing import Sequence

from . import motor
from .drivetrain import G, DrivetrainSpec, DrivetrainState, Pose, _resist
from .motor import Drive

ROOT_HALF = math.sqrt(0.5)
RIM_SCALE = {"x": 1.0, "mecanum": math.sqrt(2)}


def _wheels(spec: DrivetrainSpec) -> list[tuple[float, float, float, float]]:
    """(x right, y forward, ux, uy) for LF, LB, RF, RB: where each wheel
    touches the floor and the unit direction it pushes when spun forward.
    Square frame, track_width_m between the sides and front to back."""
    a = spec.track_width_m / 2
    return [(-a, a, ROOT_HALF, ROOT_HALF), (-a, -a, -ROOT_HALF, ROOT_HALF),
            (a, a, -ROOT_HALF, ROOT_HALF), (a, -a, ROOT_HALF, ROOT_HALF)]


def _rim_speeds(spec: DrivetrainSpec, fwd: float, right: float,
                omega_ccw: float) -> list[float]:
    """Each wheel's rim speed (m/s, + = forward) for a body velocity."""
    c = RIM_SCALE[spec.holonomic]
    out = []
    for x, y, ux, uy in _wheels(spec):
        # a point at (x, y) of a body yawing CCW moves (-omega y, omega x)
        vr, vf = right - omega_ccw * y, fwd + omega_ccw * x
        out.append(c * (vr * ux + vf * uy))
    return out


def _motor_speeds(spec: DrivetrainSpec, rims: Sequence[float]) -> list[float]:
    return [r / spec.wheel_radius_m * spec.external_ratio for r in rims]


def step(spec: DrivetrainSpec, state: DrivetrainState,
         left: Sequence[Drive], right: Sequence[Drive], dt: float,
         limits_a: Sequence[float] | None = None) -> DrivetrainState:
    """Advance the robot by dt seconds; left = (LF, LB), right = (RF, RB)."""
    if spec.motors_per_side != 2 or len(left) != 2 or len(right) != 2:
        raise ValueError("a holonomic drive has two motors a side (front, back)")
    drives = [*left, *right]
    lim = list(limits_a) if limits_a else [motor.CURRENT_LIMIT_A] * 4
    c, r = RIM_SCALE[spec.holonomic], spec.wheel_radius_m
    w_now = _motor_speeds(spec, _rim_speeds(spec, state.v_m_s, state.strafe_m_s,
                                            state.omega_rad_s))
    f_fwd = f_right = tau = 0.0
    for (x, y, ux, uy), d, w, li in zip(_wheels(spec), drives, w_now, lim):
        torque = motor.net_output_torque(motor.motor_current(d, w, spec.cartridge, li),
                                         w, spec.cartridge)
        push = torque * spec.external_ratio / r * c      # along (ux, uy)
        f_right += push * ux
        f_fwd += push * uy
        tau += x * push * uy - y * push * ux             # CCW about the centre

    roll = spec.rolling_resistance * spec.mass_kg * G
    half = spec.track_width_m / 2
    f_fwd += -spec.drag_n_per_m_s * state.v_m_s + _resist(f_fwd, state.v_m_s, roll)
    f_right += (-spec.drag_n_per_m_s * state.strafe_m_s
                + _resist(f_right, state.strafe_m_s, roll))
    tau += (-spec.turn_scrub_nm_per_rad_s * state.omega_rad_s
            + _resist(tau, state.omega_rad_s, roll * half))

    h = state.pose.heading_rad
    fwd = state.v_m_s + f_fwd / spec.mass_kg * dt
    side = state.strafe_m_s + f_right / spec.mass_kg * dt
    # to the field frame: forward is (cos h, sin h), right (sin h, -cos h)
    vx = fwd * math.cos(h) + side * math.sin(h)
    vy = fwd * math.sin(h) - side * math.cos(h)
    omega = state.omega_rad_s + tau / spec.yaw_inertia_kg_m2 * dt
    heading = h + omega * dt
    pose = Pose(x_m=state.pose.x_m + vx * dt, y_m=state.pose.y_m + vy * dt,
                heading_rad=heading)
    return _with_body_velocity(spec, replace(state, pose=pose, omega_rad_s=omega),
                               vx, vy, drives, lim, dt)


def _with_body_velocity(spec: DrivetrainSpec, state: DrivetrainState,
                        vx: float, vy: float, drives: Sequence[Drive],
                        lim: Sequence[float], dt: float) -> DrivetrainState:
    """Set the field-frame velocity (in the body frame at the new heading)
    and turn each wheel's motor with the ground."""
    h = state.pose.heading_rad
    fwd = vx * math.cos(h) + vy * math.sin(h)
    side = vx * math.sin(h) - vy * math.cos(h)
    w = _motor_speeds(spec, _rim_speeds(spec, fwd, side, state.omega_rad_s))
    motors = [motor.advance(m, d, wi, dt, spec.cartridge, li)
              for m, d, wi, li in zip([*state.left, *state.right], drives, w, lim)]
    return replace(state, v_m_s=fwd, strafe_m_s=side,
                   left=tuple(motors[:2]), right=tuple(motors[2:]))


def constrain_to_square(spec: DrivetrainSpec, state: DrivetrainState,
                        half_side_m: float, robot_half_m: float) -> DrivetrainState:
    """Keep the robot inside a square field centred on the origin: a wall
    stops the motion into it; sliding along it and turning still work."""
    lim = half_side_m - robot_half_m
    p = state.pose
    x, y = max(-lim, min(lim, p.x_m)), max(-lim, min(lim, p.y_m))
    if (x, y) == (p.x_m, p.y_m):
        return state
    h = p.heading_rad
    vx = state.v_m_s * math.cos(h) + state.strafe_m_s * math.sin(h)
    vy = state.v_m_s * math.sin(h) - state.strafe_m_s * math.cos(h)
    if (p.x_m - x) * vx > 0:
        vx = 0.0
    if (p.y_m - y) * vy > 0:
        vy = 0.0
    fwd = vx * math.cos(h) + vy * math.sin(h)
    side = vx * math.sin(h) - vy * math.cos(h)
    w = _motor_speeds(spec, _rim_speeds(spec, fwd, side, state.omega_rad_s))
    motors = [replace(m, velocity_rad_s=wi)
              for m, wi in zip([*state.left, *state.right], w)]
    return replace(state, pose=replace(p, x_m=x, y_m=y), v_m_s=fwd, strafe_m_s=side,
                   left=tuple(motors[:2]), right=tuple(motors[2:]))
