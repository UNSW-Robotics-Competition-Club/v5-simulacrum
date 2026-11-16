"""V5 3-wire (ADI) ports: device type 12 (notes/adi-protocol.md).

The brain's built-in ports A..H are this device on smart port 22; the
3-wire expander is the same device on any other port.

  device 0A, brain 03, device 13 20 <cfg A..H>, brain 02 13 -> 32-byte frames
  brain 04 <ms>   data mode: 73 every <ms> with 8 x 3 bytes of port data
  brain 50        port config bytes (analog/digital in, digital out, PWM,
                  encoder and sonar pairs, gyro), acked 02 50
  brain 51        output byte per port, for ports whose config has bit 0
                  (digital out 0/1, PWM/servo value+127), acked 02 51
  brain 53        addressable-LED data in chunks, acked 02 53; decoded
                  into led_strip(port) when the last chunk arrives

The world feeds AdiSample (pin levels, ADC counts, encoder counts, echo
times, heading); the robot reads outputs back with digital_out(), pwm() and
led_strip(). Gyro ports calibrate and integrate on the device
(notes/vision-and-extras-protocol.md D.3): 0x8000 for 1.024 s after their
config, then the angle since, in 0.1 degree, opposite to VEX's heading and
wrapped at +-3600.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from . import protocol
from .smart_device import SmartDevice

PORTS = 8
SUPPLY_ADC_5V = 3414            # bytes 28..29: (adc * 3000) >> 11 = 5000 mV
SONAR_MAX_US = 0x8CA0           # the device caps echo times here
NO_ECHO = 0xFFFF

CFG_GYRO = 0x1C
CFG_DIGITAL_IN = (0x02, 0x04, 0x0E)
CFG_PWM = (0x25, 0x29)          # legacy servo, legacy PWM (motor)
CFG_ENCODER_EVEN = (0x2E, 0x36, 0x3E, 0x46)
CFG_ENCODER_ODD = (0x32, 0x3A, 0x42, 0x4A)
CFG_SONAR_EVEN = (0x4E, 0x56, 0x5E, 0x66)
CFG_SONAR_ODD = (0x53, 0x5B, 0x63, 0x6B)
PWM_NEUTRAL = 0x7F
GYRO_CALIBRATING = -0x8000
GYRO_CAL_S = 1.024              # 1024 one-ms samples
GYRO_WRAP = 3600
GYRO_RESTART_ALL = 0x80         # 0x50 byte 10
LED_STRIP_MAX = 64
LED_BUFFER = 96


@dataclass(frozen=True)
class AdiSample:
    analog: tuple[int, ...] = (0,) * PORTS            # 12-bit ADC per port
    # Level the device reports: True = switch closed (pressed). The device
    # sends its MCU pin as is (adiPollPorts) and VEXcode's bumper/limit
    # pressing() is that value, so the input stage inverts the pulled-up
    # connector pin; an open input reads 0. [I, checked against VEXcode]
    digital: tuple[bool, ...] = (False,) * PORTS
    encoders: tuple[tuple[int, int], ...] = ((0, 0),) * 4  # (counts, counts/s) per pair
    echo_us: tuple[int, ...] = (NO_ECHO,) * 4         # sonar echo per pair
    heading_tenths: int = 0       # robot heading, 0.1 deg clockwise, unwrapped


def _output_default(cfg: int) -> int:
    return PWM_NEUTRAL if cfg in CFG_PWM or cfg == 0x01 else 0


class SmartAdi(SmartDevice[AdiSample]):
    DEVICE_TYPE = 0x0C
    FW_VERSION = 0x0B
    FRAME_LEN = 32
    ACK_COMMANDS = True

    def __init__(self, serial: int = 0x00002000):
        super().__init__(serial)
        self.config = bytes(PORTS)
        self.outputs = bytes(PORTS)
        self._enc_seen = [0] * 4      # last world counts, per pair
        self._enc_zero = [0] * 4      # world counts when the config was applied
        self._heading = 0             # last world heading (tenths)
        self._gyro_until: list[float | None] = [None] * PORTS
        self._gyro_zero: list[int | None] = [None] * PORTS
        self._led_buf = [bytearray(LED_BUFFER) for _ in range(PORTS)]
        self._strips = [((0, 0, 0),) * LED_STRIP_MAX for _ in range(PORTS)]

    def ready_frame(self) -> bytes:
        return protocol.build(bytes([0x13, self.FRAME_LEN]) + self.config)

    # ---- brain commands ----------------------------------------------

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] == 0x50:
            old, self.config = self.config, bytes(frame[1:1 + PORTS])
            self.outputs = bytes(_output_default(c) for c in self.config)
            self._enc_zero = list(self._enc_seen)
            restart = frame[10] & GYRO_RESTART_ALL
            self._calibrate(now, [p for p in range(PORTS)
                                  if restart or old[p] != self.config[p]])
        elif frame[0] == 0x51:
            self.outputs = bytes(
                frame[1 + p] if self.config[p] & 1 else self.outputs[p]
                for p in range(PORTS))
            self._calibrate(now, [p for p in range(PORTS) if frame[12] >> p & 1])
        elif frame[0] == 0x53:
            self._on_led_chunk(frame)

    def _calibrate(self, now: float, ports: list[int]) -> None:
        for p in ports:
            if self.config[p] == CFG_GYRO:
                self._gyro_until[p], self._gyro_zero[p] = now + GYRO_CAL_S, None

    def _on_led_chunk(self, frame: bytes) -> None:
        port, offset, length, total, start = frame[1:6]
        if port >= PORTS or offset + length > LED_BUFFER or 6 + length > len(frame) - 2:
            return
        buf = self._led_buf[port]
        buf[offset:offset + length] = frame[6:6 + length]
        if not total:
            return                    # more chunks to come
        count = min(total * 2 // 3, LED_STRIP_MAX - min(start, LED_STRIP_MAX))
        strip = list(self._strips[port])
        for i in range(count):
            b = buf[i // 2 * 3:i // 2 * 3 + 3]
            if i % 2 == 0:
                g, r, bl = b[0] >> 4, b[0] & 15, b[1] >> 4
            else:
                g, r, bl = b[1] & 15, b[2] >> 4, b[2] & 15
            strip[start + i] = (r * 17, g * 17, bl * 17)
        self._strips[port] = tuple(strip)

    def led_strip(self, port: int) -> tuple[tuple[int, int, int], ...]:
        """The 64 (r, g, b) colours last sent to an addressable strip."""
        return self._strips[port]

    def digital_out(self, port: int) -> bool | None:
        """Level of a digital-out port (a pneumatic solenoid), else None."""
        return bool(self.outputs[port]) if self.config[port] == 0x03 else None

    def pwm(self, port: int) -> int | None:
        """-127..128 command of a legacy PWM/servo port, else None."""
        return self.outputs[port] - PWM_NEUTRAL if self.config[port] in CFG_PWM else None

    # ---- data ---------------------------------------------------------

    def due_frames(self, now: float, sample: AdiSample) -> list[bytes]:
        # The robot calls this every loop: keep the counts a 0x50 zeroes at
        # live, not as of the last data frame.
        self._enc_seen = [c for c, _ in sample.encoders]
        self._track_gyros(now, sample.heading_tenths)
        return super().due_frames(now, sample)

    def _track_gyros(self, now: float, heading: int) -> None:
        self._heading = heading
        for p, until in enumerate(self._gyro_until):
            if until is not None and now >= until and self._gyro_zero[p] is None:
                self._gyro_zero[p] = heading      # integration starts here

    def _gyro_value(self, p: int) -> int:
        zero = self._gyro_zero[p]
        if zero is None:
            return GYRO_CALIBRATING
        q = -(self._heading - zero)
        return q if abs(q) <= GYRO_WRAP else int(math.copysign(abs(q) % GYRO_WRAP, q))

    def data_payload(self, now: float, s: AdiSample) -> bytes:
        self._enc_seen = [c for c, _ in s.encoders]
        self._track_gyros(now, s.heading_tenths)
        body = bytearray(3 * PORTS)
        for p, cfg in enumerate(self.config):
            at = 3 * p
            if cfg in CFG_ENCODER_EVEN:
                k = p // 2
                count, rate = s.encoders[k]
                rate = max(-0x8000, min(0x7FFF, rate))
                body[at:at + 6] = struct.pack(
                    "<ih", (count - self._enc_zero[k] + 2**31) % 2**32 - 2**31, rate)
            elif cfg in CFG_SONAR_EVEN:
                echo = s.echo_us[p // 2]
                body[at:at + 2] = (echo if echo == NO_ECHO
                                   else min(echo, SONAR_MAX_US)).to_bytes(2, "little")
            elif cfg in CFG_ENCODER_ODD or cfg in CFG_SONAR_ODD:
                continue                          # carried by the even port
            elif cfg & 1:                         # outputs read back
                body[at] = self.outputs[p]
            elif cfg == CFG_GYRO:
                body[at:at + 2] = struct.pack("<h", self._gyro_value(p))
            elif cfg in CFG_DIGITAL_IN:
                body[at] = int(s.digital[p])
            else:                                 # analog in
                body[at:at + 2] = (s.analog[p] & 0xFFF).to_bytes(2, "little")
        return (bytes([0x73]) + bytes(body) + bytes(3)
                + SUPPLY_ADC_5V.to_bytes(2, "little"))
