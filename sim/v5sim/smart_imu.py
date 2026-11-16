"""V5 Inertial Sensor firmware model (DEVFRM06), notes/imu-protocol.md.

Link layer as in smart_device.py, announcing 64-byte frames. In data mode
it sends 9D status every 10 ms. Brain 50 config: bit 0 = reset
(recalibrate, heading back to 0), 0x80 = accel range.

Calibration holds for 401 samples at 200 Hz (2.005 s) after data mode
starts and after each reset, with the quaternion fixed at identity; when
it ends the filter starts from identity, so heading counts from the
orientation the robot had then.

Orientation reported as 8 ("detected, Z up"): for a desired quaternion
(W', X', Y', Z') the packet carries (-X', W', Z', -Y'), which the SDK's
remap for that orientation turns back into (W', X', Y', Z').
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from .smart_device import LinkMode, SmartDevice

__all__ = ["CALIBRATION_S", "ImuSample", "LinkMode", "SmartImu", "quaternion"]

CALIBRATION_S = 401 / 200
STATUS_CALIBRATING = 0x01
STATUS_ORIENT_Z_UP = 8 << 1
TEMPERATURE_C = 25.0
DEADBANDS = bytes([8, 8, 8])
ACCEL_RANGE_SCALE = (0.5, 1.0, 2.0, 4.0)     # g per 8192 LSB
GYRO_DPS_PER_LSB = 1000 / 32767

CFG_RESET = 0x0001
CFG_ACCEL_RANGE = 0x0080


@dataclass(frozen=True)
class ImuSample:
    """What the sensor experiences. Heading is clockwise-positive, as VEX
    reports it; pitch and roll in degrees."""

    heading_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    gyro_dps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_g: tuple[float, float, float] = (0.0, 0.0, 1.0)


def quaternion(yaw_deg: float, pitch_deg: float, roll_deg: float
               ) -> tuple[float, float, float, float]:
    """(W, X, Y, Z) for ZYX Euler angles, matching the SDK's inverse."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def _s16(v: float) -> int:
    return max(-32768, min(32767, round(v)))


class SmartImu(SmartDevice[ImuSample]):
    DEVICE_TYPE = 0x06
    FW_VERSION = 0x0F
    FRAME_LEN = 64

    def __init__(self, serial: int = 0x12345678):
        super().__init__(serial)
        self.accel_range = 1
        self._calibrating_until: float | None = None
        self._heading_zero: float | None = None   # heading at last zero

    def calibrating(self, now: float) -> bool:
        return self._calibrating_until is not None and now < self._calibrating_until

    def _start_calibration(self, now: float) -> None:
        self._calibrating_until = now + CALIBRATION_S
        self._heading_zero = None

    def on_data_start(self, now: float) -> None:
        self._start_calibration(now)

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] != 0x50:
            return
        flags = int.from_bytes(frame[2:4], "little")
        if flags & CFG_ACCEL_RANGE and len(frame) > 0x18 + 2:   # 64 B only
            self.accel_range = frame[0x18] & 3
        if flags & CFG_RESET:
            self._start_calibration(now)

    def status_frame(self, now: float, s: ImuSample) -> bytes:
        return self.data_frame(now, s)

    def data_payload(self, now: float, s: ImuSample) -> bytes:
        status = STATUS_ORIENT_Z_UP
        if self.calibrating(now):
            status |= STATUS_CALIBRATING
            w, x, y, z = 1.0, 0.0, 0.0, 0.0
        else:
            if self._heading_zero is None:
                self._heading_zero = s.heading_deg
            w, x, y, z = quaternion(s.heading_deg - self._heading_zero,
                                    s.pitch_deg, s.roll_deg)
        accel_lsb = 8192 / ACCEL_RANGE_SCALE[self.accel_range]
        return (bytes([0x9D, status])
                + struct.pack("<h", _s16(TEMPERATURE_C * 256))
                # "+ 0.0" turns -0.0 into 0.0, as the hardware sends
                + struct.pack("<4f", -x + 0.0, w + 0.0, z + 0.0, -y + 0.0)
                + bytes(4)
                + struct.pack("<3h", *(_s16(a * accel_lsb) for a in s.accel_g))
                + struct.pack("<3h", *(_s16(g / GYRO_DPS_PER_LSB) for g in s.gyro_dps))
                + bytes(12) + DEADBANDS + bytes([self.accel_range]))
