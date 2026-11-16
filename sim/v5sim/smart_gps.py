"""V5 GPS Sensor, device type 20 (notes/gps-optical-protocol.md section 3).

64-byte frames after 02 13; data frame 0xAE carries the pose the sensor's
camera solves from the field strip: x, y (m from the field centre), yaw,
pitch, roll (rad), two error estimates, a quality code (0 = good fix),
plus the IMU quaternion and raw accel/gyro. The device sends at most every
20 ms whatever 04 asks for.

The SDK keeps origin offset and rotation to itself; the only brain command
with a device effect here is 0x50 mask 0x4000, seeding the pose
(InitialPositionSet), kept in .seed for the world to honour.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from .smart_device import SmartDevice

MIN_PERIOD_S = 0.020
ORIENTATION_FLAT = 1          # status byte 1 bits 1..4; 1 = no SDK remap
ACCEL_1G = 8192


@dataclass(frozen=True)
class GpsSample:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0           # as the device sends it (SDK wraps 0..360)
    pitch_rad: float = 0.0
    roll_rad: float = 0.0
    error_m: float = 0.0
    quality: int = 0               # 0 = good fix, else a failure code
    temperature_c: int = 25
    gyro_z_dps: float = 0.0


class SmartGps(SmartDevice[GpsSample]):
    DEVICE_TYPE = 0x14
    FW_VERSION = 0x0A
    FRAME_LEN = 64
    ACK_COMMANDS = True
    ACK_FLAGS = True                 # pktBuildHeader: bytes 2 and 10 are 1
    MIN_INTERVAL_S = MIN_PERIOD_S

    def __init__(self, serial: int = 0):
        super().__init__(serial)
        self.orientation = ORIENTATION_FLAT
        self.seed: tuple[float, float, float] | None = None
        self._counter = 0

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] != 0x50:
            return
        m = frame[2] | frame[3] << 8
        if m & 0x4000:
            self.seed = struct.unpack_from("<3f", frame, 36)
        if m & 0x0020:
            self.orientation = frame[1] & 0xF
        if m & 0x0004 and not m & 0x0080:
            self.orientation = ORIENTATION_FLAT

    def data_payload(self, now: float, s: GpsSample) -> bytes:
        self._counter = (self._counter + 1) & 0xFF
        half = s.yaw_rad / 2
        quat = (math.cos(half), 0.0, 0.0, math.sin(half))
        gyro_z = round(max(-32767, min(32767, s.gyro_z_dps * 32767 / 1000)))
        return (bytes([0xAE, (self.orientation & 0xF) << 1, 0,
                       s.temperature_c & 0xFF])
                + struct.pack("<4f", *quat)
                + struct.pack("<6h", 0, 0, ACCEL_1G, 0, 0, gyro_z)
                + struct.pack("<3f", s.x_m, s.y_m, s.error_m)
                + struct.pack("<3f", s.yaw_rad, s.pitch_rad, s.roll_rad)
                + struct.pack("<f", s.error_m)
                + bytes([s.quality & 0xFF, 0]))
