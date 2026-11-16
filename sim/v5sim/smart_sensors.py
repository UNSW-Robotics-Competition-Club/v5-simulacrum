"""V5 Rotation Sensor (type 4) and Distance Sensor (type 7) firmware models,
notes/sensor-protocol.md. Link layer in smart_device.py.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from .smart_device import SmartDevice

TEMPERATURE_C = 25

# ---- rotation ---------------------------------------------------------

ENC_COUNTS = 4096                       # 12-bit encoder
VELOCITY_WINDOW_S = 0.040               # device: delta over 40 updates
VELOCITY_CLAMP = 32760
ROT_CMD = 0x50
ROT_ZERO = 0xFFFF
ROT_SET_OFFSET = 0x0001
ROT_POSITION_TO_ANGLE = 0x0010


@dataclass(frozen=True)
class RotationSample:
    shaft_deg: float = 0.0              # mechanical angle, unbounded


def _wrap_cdeg(delta: int) -> int:
    return (delta + 18000) % 36000 - 18000


class SmartRotation(SmartDevice[RotationSample]):
    DEVICE_TYPE = 0x04
    FW_VERSION = 0x08
    FRAME_LEN = 16
    ACK_COMMANDS = True

    def __init__(self, serial: int = 0x00002000):
        super().__init__(serial)
        self.offset12 = 0
        self._raw12 = 0
        self._angle_cdeg: int | None = None
        self.position_cdeg = 0
        self.velocity = 0
        self._vel_ref: tuple[float, int] | None = None

    def _update(self, now: float, s: RotationSample) -> None:
        self._raw12 = round(s.shaft_deg / 360 * ENC_COUNTS) % ENC_COUNTS
        angle = ((self._raw12 - self.offset12) % ENC_COUNTS) * 36000 // ENC_COUNTS
        if self._angle_cdeg is None:
            self.position_cdeg = angle
        else:
            self.position_cdeg += _wrap_cdeg(angle - self._angle_cdeg)
        self._angle_cdeg = angle
        if self._vel_ref is None:
            self._vel_ref = (now, self.position_cdeg)
        elif now - self._vel_ref[0] >= VELOCITY_WINDOW_S:
            t0, p0 = self._vel_ref
            dps = (self.position_cdeg - p0) / 100 / (now - t0)
            self.velocity = max(-VELOCITY_CLAMP, min(VELOCITY_CLAMP, round(dps)))
            self._vel_ref = (now, self.position_cdeg)

    def sample(self, now: float, s: RotationSample) -> None:
        """Called every physics step so position tracks every wrap."""
        self._update(now, s)

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] != ROT_CMD:
            return
        flags = int.from_bytes(frame[2:4], "little")
        if flags == ROT_ZERO:
            self.offset12 = self._raw12
            self._angle_cdeg = 0
            return
        if flags & ROT_SET_OFFSET:
            self.offset12 = int.from_bytes(frame[12:14], "little") & 0xFFF
        if flags & ROT_POSITION_TO_ANGLE and self._angle_cdeg is not None:
            self.position_cdeg = self._angle_cdeg

    def data_payload(self, now: float, s: RotationSample) -> bytes:
        """Reports the state sample() keeps up to date every physics step."""
        return (bytes([0xAA, TEMPERATURE_C]) + bytes(2)
                + struct.pack("<ihhH",
                              (self.position_cdeg + 2**31) % 2**32 - 2**31,  # s32
                              self.velocity,
                              self.offset12, self._angle_cdeg or 0))


# ---- distance ---------------------------------------------------------

DISTANCE_MAX_MM = 2000
CONFIDENCE_MAX = 63
SIGNAL_B = 10000


@dataclass(frozen=True)
class DistanceSample:
    distance_mm: float | None = None   # None: nothing in range
    object_mm: float = 400.0           # apparent object size


class SmartDistance(SmartDevice[DistanceSample]):
    DEVICE_TYPE = 0x07
    FW_VERSION = 0x02
    FRAME_LEN = 32
    ACK_COMMANDS = True

    def __init__(self, serial: int = 0x00002000):
        super().__init__(serial)

    def data_payload(self, now: float, s: DistanceSample) -> bytes:
        d = s.distance_mm
        if d is None or d <= 0 or d > DISTANCE_MAX_MM:
            return bytes([0x94, 0, 0, 0, 0, 2])
        mm = round(d)
        # SDK: size = 0.32 * dist / sqrt(A / B), capped at 400.
        ratio = (0.32 * mm / max(s.object_mm, 1.0)) ** 2
        a = min(0xFFFFFFFF, round(SIGNAL_B * ratio))
        return (bytes([0x94, 0]) + struct.pack("<HBB", mm, CONFIDENCE_MAX, 2)
                + bytes(2) + struct.pack("<II", a, SIGNAL_B))


def ray_to_square_wall(x: float, y: float, heading_ccw: float,
                       half_side: float) -> float:
    """Distance from (x, y) along heading to the walls of a square field
    centred on the origin."""
    dx, dy = math.cos(heading_ccw), math.sin(heading_ccw)
    hits = []
    for pos, d in ((x, dx), (y, dy)):
        if abs(d) > 1e-9:
            hits.append(((half_side if d > 0 else -half_side) - pos) / d)
    return min(t for t in hits if t >= 0) if hits else math.inf
