"""Loaded mechanisms on a motor shaft: an arm with gravity and hard stops.

A motor drives the arm's axle through a reduction `ratio` (motor turns per
arm turn). The arm carries `mass_kg` at `length_m` from the axle (a lift or
claw arm with its game piece, as a point mass), so gravity pulls it with
m g L cos(angle) about the axle, largest level and zero upright; its
inertia m L^2 adds to the motor's. Angles are degrees above horizontal;
`start_deg` is where it sits when the motor reads 0. Hard stops at
`lo_deg` / `hi_deg` stop it dead (an inelastic hit): pushing a stop stalls
the motor, which draws current, heats up and derates like any stalled motor.

One axis, integrated with the motor model, so it works with either physics
engine.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

from . import motor
from .motor import Cartridge, Drive, MotorState

G = 9.81


@dataclass(frozen=True)
class ArmLoad:
    ratio: float                # motor turns per arm turn
    length_m: float             # axle to the load
    mass_kg: float              # load at the end of the arm
    start_deg: float = 0.0      # arm angle at motor position 0
    lo_deg: float = -90.0       # hard stops
    hi_deg: float = 90.0

    def __post_init__(self) -> None:
        if self.ratio <= 0 or self.length_m <= 0 or self.mass_kg < 0:
            raise ValueError("an arm needs ratio > 0, length > 0, mass >= 0")
        if not self.lo_deg <= self.start_deg <= self.hi_deg:
            raise ValueError("the arm must start between its stops")

    @property
    def inertia_kg_m2(self) -> float:
        return self.mass_kg * self.length_m ** 2


def arm_angle_deg(load: ArmLoad, state: MotorState) -> float:
    return load.start_deg + math.degrees(state.position_rad) / load.ratio


def gravity_torque_nm(load: ArmLoad, angle_deg: float) -> float:
    """Gravity's torque about the axle, + = raising."""
    return -load.mass_kg * G * load.length_m * math.cos(math.radians(angle_deg))


def step_arm(state: MotorState, drive: Drive, dt: float, cartridge: Cartridge,
             load: ArmLoad, limit_a: float = motor.CURRENT_LIMIT_A) -> MotorState:
    """One step of a motor driving an arm (positions at the motor output)."""
    current = motor.motor_current(drive, state.velocity_rad_s, cartridge, limit_a)
    angle = arm_angle_deg(load, state)
    torque = (motor.output_torque(current, cartridge)
              + gravity_torque_nm(load, angle) / load.ratio)
    inertia = motor.SHAFT_INERTIA_KG_M2 + load.inertia_kg_m2 / load.ratio ** 2
    velocity = motor.step_velocity(state.velocity_rad_s, torque, inertia, dt, cartridge)
    new = motor.advance(state, drive, velocity, dt, cartridge, limit_a)
    new_angle = arm_angle_deg(load, new)
    stop = (load.hi_deg if new_angle > load.hi_deg
            else load.lo_deg if new_angle < load.lo_deg else None)
    if stop is None:
        return new
    # inelastic hit: sit on the stop, stalled (the current is a stalled motor's)
    pos = math.radians((stop - load.start_deg) * load.ratio)
    return replace(new, position_rad=pos, velocity_rad_s=0.0,
                   current_a=motor.motor_current(drive, 0.0, cartridge, limit_a))
