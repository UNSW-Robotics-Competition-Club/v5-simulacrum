"""V5 Smart Motor (11 W) electromechanical model.

A brushed-DC model of the motor before its cartridge, scaled to match the
published V5 figures [VEX: 11 W, 2.1 N·m stall and 100 rpm free with the
36:1 red cartridge, 2.5 A current limit]. Internal free speed is therefore
3600 rpm at the nominal 12 V, whichever cartridge is fitted.

  V = I·R + Ke·ω_m          (armature)
  τ_m = Kt·I                (Kt = Ke, SI units)
  τ_out = τ_m · gear        (cartridge)

minus the gearbox's Coulomb friction (step_velocity), as much torque as
FREE_CURRENT_A makes. Ke and R are fitted so that a motor with that friction
still meets the published free speed and stall torque.

A drive of None means coast (H-bridge off, no current). The motor's own
controller clamps current at its limit. A first-order thermal model gives
the temperature the SDK reports.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional

NOMINAL_V = 12.0
MAX_V = 12.8                 # SDK voltage commands are ±12.8 V (±12800 mV)
INTERNAL_FREE_RPM = 3600.0
STALL_TORQUE_RED_NM = 2.1    # at the 36:1 output
CURRENT_LIMIT_A = 2.5

# Gearbox and brush friction inside the motor, before the cartridge: a
# free-spinning motor draws FREE_CURRENT_A just to turn it, and breaking a
# stopped motor loose takes BREAKAWAY_FACTOR times that torque (about 1 V
# on a stalled motor). Estimates, to be replaced by measurements of a real
# motor (notes/motor-emulation.md). Without it a holding motor on a light
# shaft never settles: any correction moves it.
FREE_CURRENT_A = 0.12
BREAKAWAY_FACTOR = 1.25
STOPPED_RAD_S = 1e-3          # slower than this counts as stopped

_W_FREE = INTERNAL_FREE_RPM * 2 * math.pi / 60
_STALL_TORQUE_INTERNAL = STALL_TORQUE_RED_NM / 36


def _fit_ke_r() -> tuple[float, float]:
    """Ke and R for which free speed (current = FREE_CURRENT_A) and stall
    torque (Ke x (V/R - FREE_CURRENT_A)) hit the published figures."""
    ke = NOMINAL_V / _W_FREE
    for _ in range(50):
        r = NOMINAL_V / (_STALL_TORQUE_INTERNAL / ke + FREE_CURRENT_A)
        ke = (NOMINAL_V - FREE_CURRENT_A * r) / _W_FREE
    return ke, r


KE, R_OHM = _fit_ke_r()       # V·s/rad (= Kt, N·m/A); armature resistance

# Thermal: heat from I²R into a lumped body cooling to ambient.
AMBIENT_C = 25.0
THERMAL_CAP_J_PER_C = 60.0
THERMAL_COND_W_PER_C = 0.25

# Free shaft (motor + light mechanism) defaults, at the output.
SHAFT_INERTIA_KG_M2 = 2e-4
SHAFT_FRICTION_NM_S = 2e-3

Drive = Optional[float]      # volts, or None to coast


class Cartridge(Enum):
    """What sits on the output: (gear ratio from the internal motor to the
    output shaft, armature resistance and gearbox friction relative to the
    11 W motor)."""

    RED_36 = (36.0, 1.0, 1.0)
    GREEN_18 = (18.0, 1.0, 1.0)
    BLUE_6 = (6.0, 1.0, 1.0)
    # The 5.5 W motor: fixed 200 rpm output, 0.5 N.m stall [VEX]. Its
    # firmware reports in green-cartridge units (900 counts/rev,
    # notes/motor55-protocol.md), so it shares the 18:1 ratio; the
    # resistance and gearbox friction that give 200 rpm and 0.5 N.m.
    MOTOR_5_5W = (18.0, 2.1, 0.476)
    # The same motor driven by its real firmware (firmware_motor.py): the
    # firmware caps the bridge at 60 % on-time, so it sees at most 0.6 x
    # 12.8 V. 200 rpm free and 0.5 N.m stall at that voltage give these.
    MOTOR_5_5W_FW = (11.76, 0.896, 0.476)

    @property
    def ratio(self) -> float:
        return self.value[0]

    @property
    def r_ohm(self) -> float:
        return R_OHM * self.value[1]

    @property
    def friction_scale(self) -> float:
        return self.value[2]

    @property
    def free_rpm(self) -> float:
        return INTERNAL_FREE_RPM / self.ratio


@dataclass(frozen=True)
class MotorState:
    position_rad: float = 0.0    # output shaft
    velocity_rad_s: float = 0.0  # output shaft
    current_a: float = 0.0
    voltage_v: float = 0.0       # applied, after clamping (0 when coasting)
    temperature_c: float = AMBIENT_C
    current_limited: bool = False

    @property
    def position_deg(self) -> float:
        return math.degrees(self.position_rad)

    @property
    def velocity_rpm(self) -> float:
        return self.velocity_rad_s * 60 / (2 * math.pi)

    @property
    def power_w(self) -> float:
        return self.voltage_v * self.current_a


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def motor_current(drive: Drive, output_velocity_rad_s: float,
                  cartridge: Cartridge,
                  limit_a: float = CURRENT_LIMIT_A) -> float:
    """Armature current for a drive at a given output speed."""
    if drive is None:
        return 0.0
    internal_w = output_velocity_rad_s * cartridge.ratio
    current = (clamp(drive, -MAX_V, MAX_V) - KE * internal_w) / cartridge.r_ohm
    return clamp(current, -limit_a, limit_a)


def output_torque(current_a: float, cartridge: Cartridge) -> float:
    """Torque at the output shaft for an armature current."""
    return KE * current_a * cartridge.ratio


def step_thermal(temperature_c: float, current_a: float, dt: float) -> float:
    heat_w = current_a * current_a * R_OHM
    loss_w = (temperature_c - AMBIENT_C) * THERMAL_COND_W_PER_C
    return temperature_c + (heat_w - loss_w) * dt / THERMAL_CAP_J_PER_C


def advance(state: MotorState, drive: Drive, velocity_rad_s: float,
            dt: float, cartridge: Cartridge,
            limit_a: float = CURRENT_LIMIT_A) -> MotorState:
    """Move a motor one step once the mechanism has set its new speed.

    The plant integrates the dynamics; this records the result and updates
    the electrical and thermal state.
    """
    voltage = 0.0 if drive is None else clamp(drive, -MAX_V, MAX_V)
    current = motor_current(drive, velocity_rad_s, cartridge, limit_a)
    return replace(
        state,
        position_rad=state.position_rad + velocity_rad_s * dt,
        velocity_rad_s=velocity_rad_s,
        current_a=current,
        voltage_v=voltage,
        temperature_c=step_thermal(state.temperature_c, current, dt),
        current_limited=drive is not None and abs(current) >= limit_a,
    )


def gear_friction_nm(cartridge: Cartridge) -> tuple[float, float]:
    """(sliding, breakaway) gearbox friction torque at the output."""
    sliding = KE * FREE_CURRENT_A * cartridge.friction_scale * cartridge.ratio
    return sliding, sliding * BREAKAWAY_FACTOR


def net_output_torque(current_a: float, output_velocity_rad_s: float,
                      cartridge: Cartridge) -> float:
    """Torque a motor delivers to what it drives, after its gearbox's
    friction: that opposes motion, and a stopped gearbox holds (delivers
    nothing) until the motor beats breakaway. For mechanisms with several
    motors (drivetrains); a single shaft uses step_velocity."""
    torque = output_torque(current_a, cartridge)
    sliding, breakaway = gear_friction_nm(cartridge)
    if abs(output_velocity_rad_s) >= STOPPED_RAD_S:
        return torque - math.copysign(sliding, output_velocity_rad_s)
    if abs(torque) <= breakaway:
        return 0.0
    return torque - math.copysign(sliding, torque)


def step_velocity(velocity_rad_s: float, torque_nm: float, inertia_kg_m2: float,
                  dt: float, cartridge: Cartridge,
                  viscous_nm_s: float = SHAFT_FRICTION_NM_S) -> float:
    """One step of a shaft under torque_nm (motor, gravity: everything but
    friction) with viscous friction and the gearbox's Coulomb friction. A
    stopped shaft stays put until the torque beats breakaway, and a moving
    one stops dead instead of friction reversing it."""
    sliding, breakaway = gear_friction_nm(cartridge)
    if abs(velocity_rad_s) < STOPPED_RAD_S:
        if abs(torque_nm) <= breakaway:
            return 0.0
        direction = math.copysign(1.0, torque_nm)
    else:
        direction = math.copysign(1.0, velocity_rad_s)
    net = torque_nm - viscous_nm_s * velocity_rad_s - direction * sliding
    velocity = velocity_rad_s + net / inertia_kg_m2 * dt
    return 0.0 if velocity * direction < 0 else velocity


def step_shaft(state: MotorState, drive: Drive, dt: float,
               cartridge: Cartridge,
               inertia_kg_m2: float = SHAFT_INERTIA_KG_M2,
               friction_nm_s: float = SHAFT_FRICTION_NM_S,
               limit_a: float = CURRENT_LIMIT_A) -> MotorState:
    """A motor driving only its own light load (intake, lift without load)."""
    current = motor_current(drive, state.velocity_rad_s, cartridge, limit_a)
    velocity = step_velocity(state.velocity_rad_s, output_torque(current, cartridge),
                             inertia_kg_m2, dt, cartridge, friction_nm_s)
    return advance(state, drive, velocity, dt, cartridge, limit_a)
