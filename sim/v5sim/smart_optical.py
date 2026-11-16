"""V5 Optical Sensor, device type 16 (notes/gps-optical-protocol.md sec. 2).

Same link layer as the other sensors (32-byte frames after 02 13); data
frame 0xAB every 20 ms carries raw clear/red/green/blue counts, proximity
and the device's register shadows. The SDK turns the raw counts into hue,
saturation and brightness itself, so the sim sends counts: raw_for_rgb()
inverts the SDK's colour matrix for a wanted colour.

Gestures (notes/vision-and-extras-protocol.md C): request_gesture() makes
the next frames report raw type 1 up, 2 down, 3 right, 4 left in byte 1,
with a FIFO count in byte 21, for 160 ms; only in gesture mode.

Brain 0x50 commands set the LED (mask 0x01), mode (0x10, 0x4F = gesture),
integration time code (0x20), gain code (0x40) and raw registers.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from .smart_device import SmartDevice

STATUS_AVALID = 0x01
STATUS_PVALID = 0x02
MODE_GESTURE = 0x40
GESTURES = {"up": 1, "down": 2, "right": 3, "left": 4}
GESTURE_HOLD_S = 0.160          # 16 ticks of 10 ms
GESTURE_COUNT = 12              # FIFO samples of a quick swipe
REG = {0x80: "mode", 0x81: "atime", 0x8F: "gain_code", 0x8E: "ppulse", 0x90: "config2"}


@dataclass(frozen=True)
class OpticalSample:
    clear: int = 0
    red: int = 0
    green: int = 0
    blue: int = 0
    proximity: int = 0           # 0..255, 255 = touching


def full_scale(atime: int) -> int:
    return min((256 - atime) * 1024 + 1, 65535)


def raw_for_rgb(rgb: tuple[float, float, float], brightness: float,
                atime: int) -> tuple[int, int, int, int]:
    """Raw (clear, red, green, blue) counts the SDK reads back as this
    colour's hue and this brightness (0..1); inverse of section 2.4."""
    r, g, b = rgb
    scale = brightness * full_scale(atime) / max(max(rgb), 1e-9)
    r, g, b = r * scale, g * scale, b * scale
    red = r / 0.7
    g2, b2 = g - 0.1 * red, b + 0.1 * red
    green = (1.05 * g2 + 0.2 * b2) / 1.01
    blue = (0.2 * g2 + b2) / 1.01

    def u16(v: float) -> int:
        return max(0, min(65535, round(v)))

    return (u16(brightness * full_scale(atime)), u16(red), u16(green), u16(blue))


class SmartOptical(SmartDevice[OpticalSample]):
    DEVICE_TYPE = 0x10
    FW_VERSION = 0x04
    FRAME_LEN = 32
    ACK_COMMANDS = True

    def __init__(self, serial: int = 0x00002000):
        super().__init__(serial)
        self.led = 0
        self.mode = 0x07
        self.atime = 0xDB
        self.gain_code = 0x09
        self.ppulse = 0x87
        self.config2 = 0x01
        self._reg_read = (0, 0)  # (value, address) of the last register read
        self._gesture_pending: int | None = None
        self._gesture = (0, 0.0)  # (raw type, reported until)

    def request_gesture(self, name: str) -> None:
        """A hand swipe in front of the sensor (seen only in gesture mode)."""
        if name not in GESTURES:
            raise ValueError(f"gesture must be one of {', '.join(GESTURES)}")
        if self.mode & MODE_GESTURE:
            self._gesture_pending = GESTURES[name]

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] != 0x50:
            return
        m = frame[2] | frame[3] << 8
        if m & 0x0001:
            self.led = min(frame[4], 100)
        if m & 0x0010:
            self._set_mode(frame[5] & 0x7F)
        if m & 0x0020:
            self.atime = frame[6]
        if m & 0x0040:
            self.gain_code = frame[7]
        if m & 0x0080:
            self.ppulse = frame[8]
        if m & 0x0100:
            self.config2 = frame[9]
        if m & 0x8000 and frame[28] in REG:
            setattr(self, REG[frame[28]], frame[29])
        if m & 0x4000:
            name = REG.get(frame[28])
            self._reg_read = (getattr(self, name) if name else 0, frame[28])

    def _set_mode(self, mode: int) -> None:
        if mode != self.mode:
            self.ppulse = 0x89 if mode & MODE_GESTURE else 0x87
        self.mode = mode

    def data_payload(self, now: float, s: OpticalSample) -> bytes:
        if self._gesture_pending is not None:
            self._gesture = (self._gesture_pending, now + GESTURE_HOLD_S)
            self._gesture_pending = None
        kind, until = self._gesture
        if now >= until:
            kind = 0
        status = STATUS_AVALID | (STATUS_PVALID if s.proximity else 0)
        return (bytes([0xAB, kind, status, 0, 0, 0])
                + struct.pack("<4H", s.clear, s.red, s.green, s.blue)
                + bytes([s.proximity & 0xFF, self.led, self.mode, self.atime,
                         self.gain_code, self.ppulse, self.config2])
                + bytes([GESTURE_COUNT if kind else 0]) + bytes(6)   # gesture + debug
                + bytes(self._reg_read))
