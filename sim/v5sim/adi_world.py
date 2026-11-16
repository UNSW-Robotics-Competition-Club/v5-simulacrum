"""What the robot's 3-wire devices sense, for SmartAdi (smart_adi.py).

Wiring is `A=kind,...` with ports A..H:

  bumper, limit   digital in at the robot's front: pressed (reads 1) while
                  the robot touches the wall it faces
  encoder         legacy quadrature encoder (360 ticks/rev) on a 2.75"
                  tracking wheel; takes the port and the next (A/B, C/D, ...)
  sonar           legacy ultrasonic at the front, echo off the field wall;
                  takes the port pair like the encoder
  piston          digital out driving a pneumatic solenoid (reported, no
                  mechanism modelled)
  gyro            legacy yaw-rate gyro on the drivetrain (the device
                  calibrates for 1 s after its config, then integrates)
  pot:N, potv2:N  potentiometer (250 / 330 degree) on the shaft of the
                  motor on smart port N, clamped to its travel
  line            line tracker facing the floor: grey tile, 30 % reflective
  light           light sensor: room light, 50 %
  accel           one accelerometer axis, the robot's forward acceleration

Analog values are the raw 12-bit ADC count, scaled by VEXcode/PROS
(notes/vision-and-extras-protocol.md D.1).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .drivetrain import INCH
from .smart_adi import NO_ECHO, PORTS, AdiSample

KINDS = ("bumper", "limit", "encoder", "sonar", "piston", "gyro",
         "pot", "potv2", "line", "light", "accel")
PAIRED = ("encoder", "sonar")
SHAFT_KINDS = {"pot": 250.0, "potv2": 330.0}      # kind -> travel, degrees
ADC_MAX = 4095
FLOOR_REFLECT_PCT = 30
ROOM_LIGHT_PCT = 50
ACCEL_COUNTS_PER_G = 275
STANDARD_G = 9.80665
ENCODER_TICKS_PER_REV = 360
TRACKING_WHEEL_DIAMETER_M = 2.75 * INCH
BUMPER_TRAVEL_M = 0.005          # pressed within 5 mm of the wall
SONAR_RANGE_M = (0.03, 3.0)
US_PER_MM = 58 / 10              # round trip at the SDK's conversion


@dataclass(frozen=True)
class AdiWiring:
    kinds: tuple[str | None, ...]
    shafts: tuple[int | None, ...] = (None,) * PORTS   # pot: motor port

    @classmethod
    def parse(cls, text: str) -> "AdiWiring":
        kinds: list[str | None] = [None] * PORTS
        shafts: list[int | None] = [None] * PORTS
        for item in filter(None, text.split(",")):
            letter, _, spec = item.partition("=")
            kind, _, shaft = spec.partition(":")
            p = ord(letter.upper()) - ord("A") if len(letter) == 1 else -1
            if not 0 <= p < PORTS or kind not in KINDS:
                raise ValueError(f"bad 3-wire device {item!r}: use A..H=" + "|".join(KINDS))
            if (kind in SHAFT_KINDS) != bool(shaft) or (shaft and not shaft.isdigit()):
                raise ValueError(f"{item!r}: pot and potv2 take the motor port "
                                 "they sit on (A=pot:7); nothing else takes one")
            shafts[p] = int(shaft) if shaft else None
            ports = (p, p + 1) if kind in PAIRED else (p,)
            if kind in PAIRED and p % 2:
                raise ValueError(f"{kind} goes on A, C, E or G (it uses the next port too)")
            if any(kinds[q] for q in ports):
                raise ValueError(f"port {letter} is already wired")
            for q in ports:
                kinds[q] = kind
        # No devices is valid: the built-in block (port 22) is present even
        # with nothing wired, so the on-brain 3-wire screen has a device.
        return cls(tuple(kinds), tuple(shafts))

    @classmethod
    def empty(cls) -> "AdiWiring":
        """The built-in block with nothing wired."""
        return cls((None,) * PORTS)

    def describe_ports(self, kind: str) -> list[str]:
        return [chr(ord("A") + p) for p, k in enumerate(self.kinds) if k == kind]


@dataclass(frozen=True)
class World:
    front_gap_m: float = math.inf     # robot front to the wall it faces
    travel_m: float = 0.0             # tracking wheel distance, signed
    speed_m_s: float = 0.0
    accel_m_s2: float = 0.0           # forward
    heading_deg: float = 0.0          # clockwise, unwrapped
    shaft_deg: dict[int, float] = field(default_factory=dict)  # motor port -> angle


def _adc(value: float) -> int:
    return max(0, min(ADC_MAX, round(value)))


def _analog(kind: str | None, shaft: int | None, w: World) -> int:
    if kind in SHAFT_KINDS:
        return _adc(w.shaft_deg.get(shaft, 0.0) / SHAFT_KINDS[kind] * 4096)
    if kind == "line":
        return _adc((100 - FLOOR_REFLECT_PCT) * 30)
    if kind == "light":
        return _adc((100 - ROOM_LIGHT_PCT) * 9)
    if kind == "accel":
        return _adc(2048 + w.accel_m_s2 / STANDARD_G * ACCEL_COUNTS_PER_G)
    return 0


def adi_sample(wiring: AdiWiring, w: World) -> AdiSample:
    ticks_per_m = ENCODER_TICKS_PER_REV / (math.pi * TRACKING_WHEEL_DIAMETER_M)
    pressed = w.front_gap_m <= BUMPER_TRAVEL_M
    lo, hi = SONAR_RANGE_M
    echo = (round(w.front_gap_m * 1000 * US_PER_MM)
            if lo <= w.front_gap_m <= hi else NO_ECHO)
    digital = tuple(k in ("bumper", "limit") and pressed for k in wiring.kinds)
    encoders = tuple((round(w.travel_m * ticks_per_m), round(w.speed_m_s * ticks_per_m))
                     if wiring.kinds[2 * k] == "encoder" else (0, 0)
                     for k in range(PORTS // 2))
    echoes = tuple(echo if wiring.kinds[2 * k] == "sonar" else NO_ECHO
                   for k in range(PORTS // 2))
    analog = tuple(_analog(k, sh, w) for k, sh in zip(wiring.kinds, wiring.shafts))
    return AdiSample(analog=analog, digital=digital, encoders=encoders, echo_us=echoes,
                     heading_tenths=round(w.heading_deg * 10))
