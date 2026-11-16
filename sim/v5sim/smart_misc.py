"""Electromagnet (type 17), Light Tower (27), CTE Pneumatics (30) and LED (3)
(notes/misc-devices-protocol.md sections 2-5).

All four use the shared link layer unchanged and ack every 50..6F command;
they sense nothing from the world (the tower's button aside), so their data
frames echo what the brain set plus a small sim-defined model: magnet
current and temperature, pneumatic pressure and actuation counters.
"""
from __future__ import annotations

import math
import struct

from .smart_device import LinkMode, SmartDevice

DEVICE_SERIAL = 0x00002000


def _s8(b: int) -> int:
    return b - 256 if b > 127 else b


# ---- electromagnet ----------------------------------------------------

MAGNET_POWER_LIMIT = 100
MAGNET_FLAG_DRIVE_OFF = 0x01
PICKUP_MA = 400                  # current at the SDK's pickup power byte 0x1F
PICKUP_BYTE = 0x1F
AMBIENT_C = 25.0
DEG_C_PER_MA = 0.015             # steady rise above ambient (400 mA -> 31 C)
THERMAL_TAU_S = 20.0
MA_PER_ADC = 3.3 / 2.2 * 1000 / 4096


def temperature_adc(temp_c: float) -> int:
    """Thermistor ADC count the device reads back as temp_c (beta 3380)."""
    r = 10000 * math.exp(3380 * (1 / (temp_c + 273.15) - 1 / 298.15))
    return round(4096 * r / (r + 10000))


class SmartMagnet(SmartDevice[None]):
    DEVICE_TYPE = 0x11
    FW_VERSION = 3
    ACK_COMMANDS = True

    def __init__(self, serial: int = DEVICE_SERIAL):
        super().__init__(serial)
        self.power = 0                   # s8 power byte, (power% * 5) >> 3
        self.deadline: float | None = None
        self.temp_c = AMBIENT_C
        self._last: float | None = None

    @property
    def current_ma(self) -> int:
        return round(PICKUP_MA * abs(self.power) / PICKUP_BYTE)

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] != 0x51:
            return                       # 0x50 config is vex-only: ack, ignore
        power = _s8(frame[1])
        if abs(power) > MAGNET_POWER_LIMIT:
            return
        ms = int.from_bytes(frame[2:4], "little")
        self.power = power
        self.deadline = now + ms / 1000 if ms else None

    def _on_link(self, frame: bytes, mode: LinkMode, default_ms: int,
                 cmd: int, now: float) -> list[bytes]:
        if mode == LinkMode.IDLE:        # mode 0x21: magnetSetDriveMode(3)
            self.power, self.deadline = 0, None
        return super()._on_link(frame, mode, default_ms, cmd, now)

    def _advance(self, now: float) -> None:
        if self.deadline is not None and now >= self.deadline:
            self.power, self.deadline = 0, None
        dt = 0.0 if self._last is None else max(0.0, now - self._last)
        self._last = now
        target = AMBIENT_C + DEG_C_PER_MA * self.current_ma
        self.temp_c += (target - self.temp_c) * (1 - math.exp(-dt / THERMAL_TAU_S))

    def data_payload(self, now: float, sample: None) -> bytes:
        self._advance(now)
        ma = self.current_ma
        raw = round(ma / MA_PER_ADC)
        loop = max(0, int((raw * 0.0080997504 - 0.8179672109) * 1000))
        flags = MAGNET_FLAG_DRIVE_OFF if self.power == 0 else 0
        return (bytes([0xAD, self.power & 0xFF])
                + struct.pack("<5H", temperature_adc(self.temp_c), 0, raw, ma,
                              min(loop, 0xFFFF))
                + bytes([int(self.temp_c + 0.5), flags]))


# ---- light tower ------------------------------------------------------

TOWER_CHANNELS = ("red", "green", "blue", "white", "yellow")
BLINK_DEFAULT_MS = 500


class SmartLightTower(SmartDevice[bool | None]):
    DEVICE_TYPE = 0x1B
    FW_VERSION = 2
    ACK_COMMANDS = True

    def __init__(self, serial: int = DEVICE_SERIAL):
        super().__init__(serial)
        self.values = (0,) * len(TOWER_CHANNELS)
        self.blink_mask = 0
        self.on_ms = 0
        self.off_ms = 0
        self._blink_t0 = 0.0
        self.pressed = False

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] != 0x52:
            return
        mask = frame[2]
        self.values = tuple(frame[3 + i] if mask >> i & 1 else v
                            for i, v in enumerate(self.values))
        blink = (frame[8], frame[9] * 10 or BLINK_DEFAULT_MS)
        blink += (frame[10] * 10 or blink[1],)
        if blink != (self.blink_mask, self.on_ms, self.off_ms):
            self._blink_t0 = now         # colour-only updates keep the phase
        self.blink_mask, self.on_ms, self.off_ms = blink

    def lit(self, now: float) -> tuple[int, ...]:
        """Channel brightness as seen now, blinking channels in phase."""
        period = (self.on_ms + self.off_ms) / 1000
        on = period <= 0 or (now - self._blink_t0) % period < self.on_ms / 1000
        return tuple(v if on or not self.blink_mask >> i & 1 else 0
                     for i, v in enumerate(self.values))

    def data_payload(self, now: float, sample: bool | None) -> bytes:
        if sample is not None:           # the robot holds the button state
            self.pressed = sample
        return bytes([0x70, int(self.pressed), *self.values, self.blink_mask,
                      self.on_ms & 0xFF, self.off_ms & 0xFF])


# ---- CTE pneumatics ---------------------------------------------------

CYLINDERS = 4
PUMP_ON, PUMP_OFF = 0x0001, 0x0002
PRESSURE_IDLE = 6200             # 0.01 psi counts [I]
PRESSURE_FULL = 9000
PUMP_RATE_PER_S = 400
ACTUATION_DROP = 150


class SmartPneumatic(SmartDevice[None]):
    DEVICE_TYPE = 0x1E
    FW_VERSION = 3
    FRAME_LEN = 32
    ACK_COMMANDS = True

    def __init__(self, serial: int = DEVICE_SERIAL):
        super().__init__(serial)
        self.extended = (False,) * CYLINDERS
        self.counts = (0,) * CYLINDERS
        self.pump = False
        self.pressure = PRESSURE_IDLE
        self._last: float | None = None
        self._decay_at = [0.0] * CYLINDERS   # next -1 of each counter

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] != 0x50:
            return
        flags = int.from_bytes(frame[10:12], "little")
        if flags & PUMP_ON:
            self.pump = True
        elif flags & PUMP_OFF:
            self.pump = False
        ext, counts = list(self.extended), list(self.counts)
        for i in range(CYLINDERS):
            pair = flags >> (2 + 2 * i) & 0x3
            if not pair:
                continue
            want = bool(pair & 0x1)
            if want != ext[i]:
                ext[i] = want
                if not counts[i]:
                    self._decay_at[i] = now + 1.0
                counts[i] = min(counts[i] + 1, 0xFF)
                self.pressure = max(0, self.pressure - ACTUATION_DROP)
        self.extended, self.counts = tuple(ext), tuple(counts)

    def _advance(self, now: float) -> None:
        dt = 0.0 if self._last is None else max(0.0, now - self._last)
        self._last = now
        if self.pump:
            self.pressure = min(PRESSURE_FULL,
                                self.pressure + round(PUMP_RATE_PER_S * dt))
        # each actuation counter drops by one per second
        counts = list(self.counts)
        for i in range(CYLINDERS):
            while counts[i] and now >= self._decay_at[i]:
                counts[i] -= 1
                self._decay_at[i] += 1.0
        self.counts = tuple(counts)

    def status(self) -> int:
        bits = PUMP_ON if self.pump else PUMP_OFF
        for i, ext in enumerate(self.extended):
            bits |= (0x04 if ext else 0x08) << (2 * i)
        return bits

    def data_payload(self, now: float, sample: None) -> bytes:
        self._advance(now)
        return (bytes([0x70, 0]) + struct.pack("<H", self.status())
                + bytes(self.counts) + bytes(20) + struct.pack("<H", self.pressure))


# ---- LED --------------------------------------------------------------

class SmartLed(SmartDevice[None]):
    DEVICE_TYPE = 0x03
    FW_VERSION = 4
    ACK_COMMANDS = True

    def __init__(self, serial: int = DEVICE_SERIAL):
        super().__init__(serial)
        self.rgb = (0, 0, 0)

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] not in (0x51, 0x52):
            return                       # 0x50 palette index: not extracted
        bright = frame[7]
        self.rgb = tuple(min(255, b * bright // 100) for b in frame[1:4])

    def data_payload(self, now: float, sample: None) -> bytes:
        return bytes([0x70, 0])
