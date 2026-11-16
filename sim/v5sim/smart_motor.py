"""V5 Smart Motor firmware model: smart-port protocol plus control loops.

Behaves like DEVFRM02 as far as notes/motor-protocol.md describes it:

  power-up   send 0x0A device info every 50 ms
  brain 03   reply 0x13 config echo (first time) or 02 03; idle mode:
             0x11 keepalive every <interval> ms (03 bytes 2..3, 100)
  brain 04   reply 02 04; data mode: 0x76 status every <interval> ms (5)
  brain 56   control command (velocity / voltage / position, brake code)
  brain 55   voltage and current limits
  brain 02   heartbeat: refreshes the 2 s watchdog

The motor knows nothing of the cartridge setting in the SDK: positions and
speeds are raw shaft counts, 50 per internal motor revolution. The PID
gains here are this model's own (the real firmware's scaling isn't decoded).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import IntEnum
from pathlib import Path

from . import protocol
from .motor import AMBIENT_C, Cartridge, Drive, MotorState, clamp

COUNTS_PER_SHAFT_REV = 50
MAX_VEL_COUNTS = 3000            # 3600 shaft rpm

INFO_PERIOD_S = 0.050
WATCHDOG_S = 2.0

FW_VERSION = 0x1F                # >= DEVFRM02 min version
FW_ID = 0x4000                   # == DEVFRM02 id
DEVICE_TYPE_MOTOR = 0x02
FW_VERSION_5_5W = 0x05
DEVICE_TYPE_MOTOR_5_5W = 0x05
FLAG_5_5W = 0x80                 # status byte 12 bit 7 on the 5.5 W motor

FIRMWARE_DIR = Path(__file__).resolve().parents[2] / "extracted" / "assets" / "files"


CONFIG_ECHO = bytes([0x13, 0x10, 0x30, 0x28, 0x40, 0x20, 0x23, 0x00,
                     0x7F, 0x00, 0x7F, 0x00, 0xC0, 0x09])

# Model-specific loop gains (volts per count/s, volts per count/s·s, 1/s).
VEL_FF = 12.0 / MAX_VEL_COUNTS
VEL_KP = 0.004
VEL_KI = 0.04
VEL_I_ZONE = 300                 # counts/s around the target
# The integrator may supply the whole supply voltage: a motor holding a
# load (an arm against gravity) has to, or it settles off its target and
# never reports "at target" (VEXcode's spinToPosition would wait forever).
VEL_I_MAX = 12.0                 # volts
POS_KP = 10.0
ZERO_VEL_COUNTS = 5
STALL_VEL_COUNTS = 20            # counts/s: asked to move but not moving
STALL_HOLD_S = 0.05              # ... for this long
AT_TARGET_COUNTS = 10

FLAG_ZERO_VELOCITY = 0x02
FLAG_ZERO_POSITION = 0x04
FAULT_OVER_TEMP = 0x01
FAULT_OVER_CURRENT = 0x04
OVER_TEMP_C = 55.0
# Thermal derating: from 55 C the motor halves its current limit at each
# 5 C step, down to none at 70 C; faults bits 4..6 mark the levels above the
# over-temp bit [I: VEX's published behaviour; the firmware thresholds
# (motor-protocol.md "thermal-state thresholds") aren't decoded].
THERMAL_STEPS = ((70.0, 0.0), (65.0, 0.125), (60.0, 0.25), (55.0, 0.5))
FAULT_THERMAL_LEVELS = ((55.0, 0x10), (60.0, 0x20), (65.0, 0x40))


def thermal_derate(temperature_c: float) -> float:
    """Fraction of the current limit a motor at this temperature allows."""
    for threshold, factor in THERMAL_STEPS:
        if temperature_c >= threshold:
            return factor
    return 1.0


class LinkMode(IntEnum):
    INFO = 0x20
    IDLE = 0x21
    DATA = 0x22


class ControlMode(IntEnum):
    NONE = 0
    VELOCITY = 2
    VOLTAGE = 3
    POSITION = 4
    PROFILE = 5


class Brake(IntEnum):
    COAST = 1
    HOLD = 2
    BRAKE = 3


@dataclass(frozen=True)
class Command:
    mode: ControlMode = ControlMode.NONE
    velocity: int = 0            # counts/s (target, or max in position mode)
    target: int = 0              # counts, absolute
    percent: int = 0             # voltage mode, of the voltage limit
    brake: Brake = Brake.COAST


def shaft_counts(state: MotorState, cartridge: Cartridge) -> int:
    revs = state.position_rad / (2 * math.pi) * cartridge.ratio
    return round(revs * COUNTS_PER_SHAFT_REV)


def shaft_velocity(state: MotorState, cartridge: Cartridge) -> float:
    revs_s = state.velocity_rad_s / (2 * math.pi) * cartridge.ratio
    return revs_s * COUNTS_PER_SHAFT_REV


def _s16(b: bytes) -> int:
    return int.from_bytes(b, "little", signed=True)


def apply_control(c: Command, frame: bytes) -> Command:
    """The command after a brain 0x56 frame (motor-protocol.md 1.2.1)."""
    mode, flags, brake = frame[1], frame[10], frame[12]
    if brake in (1, 2, 3):
        c = replace(c, brake=Brake(brake))
    if mode in (2, 3, 4, 5):
        c = replace(c, mode=ControlMode(mode))
    if flags & 0x01 or mode in (2, 4, 5):
        c = replace(c, velocity=_s16(frame[2:4]))
    if flags & 0x04:
        c = replace(c, target=int.from_bytes(frame[4:8], "little", signed=True))
    elif flags & 0x02:
        c = replace(c, target=c.target + int.from_bytes(frame[4:8], "little", signed=True))
    if flags & 0x08:
        c = replace(c, percent=int.from_bytes(frame[8:9], "little", signed=True))
    return c


class SmartMotor:
    """One motor's firmware. Feed it brain frames and time; it answers."""

    def __init__(self, cartridge: Cartridge = Cartridge.GREEN_18,
                 serial: int = 0x12345678):
        self.cartridge = cartridge
        self.serial = serial
        self.link = LinkMode.INFO
        self.interval_s = INFO_PERIOD_S
        self.command = Command()
        self.voltage_limit_v = 12.0
        self.current_limit_a = 2.496
        self._temp_c = AMBIENT_C             # last temperature seen (drive)
        self._next_tx = 0.0
        self._last_brain = None
        self._integral = 0.0
        self._stall_s = 0.0                  # how long it's been stalled
        self._hold_target: int | None = None

    # ---- frames -------------------------------------------------------

    @property
    def is_5_5w(self) -> bool:
        return self.cartridge == Cartridge.MOTOR_5_5W

    def info_frame(self) -> bytes:
        # the 5.5 W firmware announces version 5, type 5 (motor55-protocol.md 2)
        version, kind = ((FW_VERSION_5_5W, DEVICE_TYPE_MOTOR_5_5W) if self.is_5_5w
                         else (FW_VERSION, DEVICE_TYPE_MOTOR))
        return protocol.build(
            bytes([0x0A, version, 0, 0, kind, 0])
            + FW_ID.to_bytes(2, "little") + self.serial.to_bytes(4, "little"))

    @property
    def effective_current_limit_a(self) -> float:
        """The current limit after thermal derating: what the physics gets."""
        return self.current_limit_a * thermal_derate(self._temp_c)

    def faults(self, state: MotorState) -> int:
        faults = FAULT_OVER_CURRENT if state.current_limited else 0
        if state.temperature_c >= OVER_TEMP_C:
            faults |= FAULT_OVER_TEMP
        for threshold, bit in FAULT_THERMAL_LEVELS:
            if state.temperature_c >= threshold:
                faults |= bit
        return faults

    def status_frame(self, state: MotorState) -> bytes:
        counts = shaft_counts(state, self.cartridge)
        vel = round(shaft_velocity(state, self.cartridge))
        flags = FLAG_ZERO_VELOCITY if abs(vel) < ZERO_VEL_COUNTS else 0
        # "at target" compares with the position target in every mode: an
        # idle motor (target 0) at 0 reports it (motor-protocol.md 7, 76 idle)
        if abs(self.command.target - counts) < AT_TARGET_COUNTS:
            flags |= FLAG_ZERO_POSITION
        if self.is_5_5w:
            flags |= FLAG_5_5W                      # always set by its firmware
        faults = self.faults(state)
        temp = int(clamp(5 * round(state.temperature_c / 5), 0, 255))
        current_ma = int(clamp(abs(state.current_a) * 1000, 0, 0xFFFF))
        mv = int(clamp(state.voltage_v * 1000, -32768, 32767))
        return protocol.build(
            bytes([0x76, temp]) + current_ma.to_bytes(2, "little")
            + (counts & 0xFFFFFFFF).to_bytes(4, "little")
            + int(clamp(vel, -32768, 32767)).to_bytes(2, "little", signed=True)
            + mv.to_bytes(2, "little", signed=True) + bytes([flags, faults]))

    # ---- brain -> motor ----------------------------------------------

    def on_brain_frame(self, frame: bytes, now: float) -> list[bytes]:
        """Handle one brain frame; return immediate replies."""
        if not protocol.is_valid(frame):
            return []
        cmd = frame[0] & 0x7F
        self._last_brain = now
        if cmd == 0x03:
            return self._on_link(frame, LinkMode.IDLE, 100, 0x03, now)
        if cmd == 0x04:
            return self._on_link(frame, LinkMode.DATA, 100, 0x04, now)
        if cmd == 0x55:
            self._on_config(frame)
        elif cmd == 0x56:
            self._on_control(frame, now)
        return []

    def _on_link(self, frame: bytes, mode: LinkMode, default_ms: int,
                 cmd: int, now: float) -> list[bytes]:
        ms = int.from_bytes(frame[2:4], "little") or default_ms
        reply = (protocol.build(CONFIG_ECHO) if self.link == LinkMode.INFO
                 else protocol.build(bytes([0x02, cmd])))
        self.link = mode
        self.interval_s = ms / 1000
        self._next_tx = now + self.interval_s
        return [reply]

    def _on_config(self, frame: bytes) -> None:
        mv = int.from_bytes(frame[8:10], "little")
        ma = int.from_bytes(frame[10:12], "little")
        if mv:
            self.voltage_limit_v = mv / 1000
        if ma:
            self.current_limit_a = ma / 1000

    def _on_control(self, frame: bytes, now: float) -> None:
        c = apply_control(self.command, frame)
        if c.brake != Brake.HOLD:
            self._hold_target = None      # re-latch where it stops next time
        if c.mode != self.command.mode:
            self._integral = 0.0
        self.command = c

    # ---- periodic output ---------------------------------------------

    def reboot(self, now: float) -> None:
        """Start over at power-up: announce again, forget the command."""
        self.link = LinkMode.INFO
        self.interval_s = INFO_PERIOD_S
        self.command = Command()
        self._integral = 0.0
        self._stall_s = 0.0
        self._hold_target = None
        self._next_tx = now

    def due_frames(self, now: float, state: MotorState) -> list[bytes]:
        if (self.link != LinkMode.INFO and self._last_brain is not None
                and now - self._last_brain > max(WATCHDOG_S, 2 * self.interval_s)):
            self.reboot(now)     # the brain reset the port (or stalled)
        if now < self._next_tx:
            return []
        self._next_tx = max(self._next_tx + self.interval_s, now)
        if self.link == LinkMode.INFO:
            return [self.info_frame()]
        if self.link == LinkMode.IDLE:
            return [protocol.build(b"\x11")]
        return [self.status_frame(state)]

    # ---- control -----------------------------------------------------

    def drive(self, state: MotorState, now: float, dt: float) -> Drive:
        """Voltage to apply this step, or None to coast."""
        self._temp_c = state.temperature_c
        if self._last_brain is None or now - self._last_brain > WATCHDOG_S:
            return None
        c = self.command
        counts = shaft_counts(state, self.cartridge)
        vel = shaft_velocity(state, self.cartridge)
        if c.mode == ControlMode.VOLTAGE and c.percent:
            self._hold_target = None
            return c.percent / 100 * self.voltage_limit_v
        if c.mode == ControlMode.VELOCITY and c.velocity:
            self._hold_target = None
            return self._velocity_loop(c.velocity, vel, dt)
        if c.mode in (ControlMode.POSITION, ControlMode.PROFILE):
            self._hold_target = None
            limit = abs(c.velocity) or MAX_VEL_COUNTS
            want = clamp(POS_KP * (c.target - counts), -limit, limit)
            return self._velocity_loop(want, vel, dt)
        return self._stopped(counts, vel, dt)

    def _stopped(self, counts: int, vel: float, dt: float) -> Drive:
        brake = self.command.brake
        if brake == Brake.COAST:
            self._integral = 0.0
            return None
        if brake == Brake.BRAKE:
            return 0.0
        if self._hold_target is None:
            self._hold_target = counts
        want = clamp(POS_KP * (self._hold_target - counts),
                     -MAX_VEL_COUNTS, MAX_VEL_COUNTS)
        return self._velocity_loop(want, vel, dt)

    def _velocity_loop(self, target: float, vel: float, dt: float) -> float:
        err = target - vel
        limit = self.voltage_limit_v
        unclamped = VEL_FF * target + VEL_KP * err + self._integral
        # Integrate only near the target and while the output isn't saturated
        # in the error's direction; otherwise the integrator winds up during
        # acceleration and the speed overshoots. Feedforward does the rest.
        # A motor held stalled by a load (an arm on its stop, a lift) for a
        # while integrates too, so it pushes harder until it moves; a motor
        # just starting from rest doesn't (that would wind up and overshoot).
        stalled = bool(target) and abs(vel) < STALL_VEL_COUNTS
        self._stall_s = self._stall_s + dt if stalled else 0.0
        near = abs(err) < VEL_I_ZONE or self._stall_s > STALL_HOLD_S
        if near and (abs(unclamped) < limit or (unclamped > 0) != (err > 0)):
            self._integral = clamp(self._integral + VEL_KI * err * dt,
                                   -VEL_I_MAX, VEL_I_MAX)
        return clamp(VEL_FF * target + VEL_KP * err + self._integral,
                     -limit, limit)
