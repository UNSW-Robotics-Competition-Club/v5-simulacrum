"""Legacy V5 Vision Sensor, device type 11 (notes/vision-and-extras-protocol.md A).

64-byte frames after 02 13; the brain asks for a report every 100 ms.
Each report is a 0x8F status (mode, WiFi, brightness, white balance, LED:
echoing what the brain set, or the SDK's getters flip back) followed by
object packets: 0x90 first / 0x91 middle / 0x92 last, 7 records of 8
bytes each (x and width halved), at most 32 objects; a lone 0x92 with
count 0 means nothing seen. The brain answers one frame at a time, so the
packets go out one per brain heartbeat after the status.

Brain 0x50 55 11 sets a colour signature (id 1..7: u/v chroma ranges and
a range factor, as the Vision Utility makes them); 0x51 sets WiFi (sub 0),
brightness (1), white balance (2), mode and LED (4).

The world supplies what is in view as chroma (u, v) and a box in the
316x212 image; the sensor reports the views whose chroma falls inside a
signature's box, widened by its range. The chroma scale is the Pixy2 one
the sensor is built on: (R-G) and (B-G) over R+G+B, times 2^14 [I: fits
the note's red signature; not measured].
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from . import protocol
from .smart_device import LinkMode, SmartDevice

WIDTH, HEIGHT = 316, 212
MAX_OBJECTS = 32
PER_PACKET = 7
CHROMA_SCALE = 1 << 14
RANGE_NOMINAL = 2.5             # range factor at which [min, max] is as given [I]
WIFI_OFF, WIFI_ON = 1, 2
LED_DEFAULT = (0x80, 0x80, 0x80)
LED_BRIGHTNESS_DEFAULT = 50
BRIGHTNESS_DEFAULT = 50


def uv_for_rgb(rgb: tuple[int, int, int]) -> tuple[int, int]:
    """Chroma the sensor computes for a colour [I]."""
    r, g, b = rgb
    total = max(r + g + b, 1)
    return round((r - g) * CHROMA_SCALE / total), round((b - g) * CHROMA_SCALE / total)


@dataclass(frozen=True)
class VisionView:
    """Something in view: its chroma and box (316x212, top-left)."""
    uv: tuple[int, int]
    x: int
    y: int
    w: int
    h: int


@dataclass(frozen=True)
class VisionSample:
    in_view: tuple[VisionView, ...] = ()


@dataclass(frozen=True)
class VisionObject:
    sig: int
    x: int
    y: int
    w: int
    h: int
    angle: int


@dataclass(frozen=True)
class Signature:
    range: float
    u_min: int
    u_max: int
    u_mean: int
    v_min: int
    v_max: int
    v_mean: int
    raw: bytes                  # struct bytes 4..0x27, as the brain sent them

    def matches(self, uv: tuple[int, int]) -> bool:
        k = max(self.range, 0.0) / RANGE_NOMINAL
        u, v = uv
        return (self.u_mean - (self.u_mean - self.u_min) * k <= u
                <= self.u_mean + (self.u_max - self.u_mean) * k
                and self.v_mean - (self.v_mean - self.v_min) * k <= v
                <= self.v_mean + (self.v_max - self.v_mean) * k)


class SmartVision(SmartDevice[VisionSample]):
    DEVICE_TYPE = 0x0B
    FW_VERSION = 0x12
    FRAME_LEN = 64
    ACK_COMMANDS = True
    ACK_FLAGS = True            # like the AI Vision; seen to work end to end

    def __init__(self, serial: int = 0):
        super().__init__(serial)
        self.mode = 0
        self.wifi = WIFI_OFF
        self.brightness = BRIGHTNESS_DEFAULT
        self.wb_manual = False
        self.wb = (0, 0, 0)
        self.led_manual = False
        self.led_brightness = LED_BRIGHTNESS_DEFAULT
        self.led_rgb = LED_DEFAULT
        self.signatures: dict[int, Signature] = {}
        self._seq = 0
        self._queue: list[bytes] = []

    # ---- brain commands ----------------------------------------------

    def on_command(self, f: bytes, now: float) -> None:
        if f[0] == 0x50 and f[1:3] == b"\x55\x11" and 1 <= f[4] <= 7:
            rng, *vals = struct.unpack_from("<f6i", f, 8)
            self.signatures[f[4]] = Signature(rng, *vals, raw=bytes(f[8:44]))
        elif f[0] == 0x51:
            sub = f[3]
            if sub == 0:
                self.wifi = f[2] if f[2] in (WIFI_OFF, WIFI_ON) else self.wifi
            elif sub == 1:
                self.brightness = f[4]
            elif sub == 2:
                self.wb_manual, self.wb = f[5] == 2, (f[6], f[7], f[8])
            elif sub == 4:
                self.mode = f[1]
                self.led_manual = f[9] == 1
                self.led_brightness, self.led_rgb = f[10], (f[11], f[12], f[13])

    # ---- detection -----------------------------------------------------

    def detect(self, sample: VisionSample) -> list[VisionObject]:
        found = []
        for view in sample.in_view:
            for sig_id, sig in sorted(self.signatures.items()):
                if sig.matches(view.uv):
                    found.append(VisionObject(sig_id, view.x, view.y, view.w, view.h, 0))
                    break
        found.sort(key=lambda o: o.w * o.h, reverse=True)       # largest first [I]
        return found[:MAX_OBJECTS]

    # ---- frames ----------------------------------------------------------

    def status_frame(self) -> bytes:
        p = bytearray(14)
        p[0:3] = bytes([0x8F, self.mode, self.wifi])
        p[4], p[5] = self.brightness, int(self.wb_manual)
        p[6:9] = bytes(self.wb)
        p[9], p[10] = int(self.led_manual), self.led_brightness
        p[11:14] = bytes(self.led_rgb)
        return self.frame(bytes(p))        # byte 0x14 = 0: no signature echo

    def object_frames(self, objects: list[VisionObject], seq: int) -> list[bytes]:
        objects = objects[:MAX_OBJECTS]
        groups = [objects[i:i + PER_PACKET]
                  for i in range(0, len(objects), PER_PACKET)] or [[]]
        frames = []
        for n, group in enumerate(groups):
            kind = 0x92 if n == len(groups) - 1 else (0x90 if n == 0 else 0x91)
            recs = b"".join(
                struct.pack("<HBBBBH", o.sig, _u8(o.x // 2), _u8(o.y), _u8(o.w // 2),
                            _u8(o.h), o.angle & 0xFFFF) for o in group)
            frames.append(self.frame(bytes([kind, len(group)])
                                     + struct.pack("<H", seq & 0xFFFF) + recs))
        return frames

    def on_data_start(self, now: float) -> None:
        self._queue = []                 # a re-pair drops a half-sent report

    def data_frame(self, now: float, sample: VisionSample) -> bytes:
        """A report: the status now, each object packet in reply to the
        brain's next heartbeat (it answers one frame at a time, and sends
        queued SDK packets such as SignatureSet in reply to the status)."""
        self._queue = self.object_frames(self.detect(sample), self._seq)
        self._seq = (self._seq + 1) & 0xFFFF
        return self.status_frame()

    def on_brain_frame(self, frame: bytes, now: float) -> list[bytes]:
        replies = super().on_brain_frame(frame, now)
        if (protocol.is_valid(frame) and frame[0] == 0x02 and frame[1] != 0x13
                and self.link == LinkMode.DATA and self._queue and not replies):
            replies.append(self._queue.pop(0))
        return replies

def _u8(v: int) -> int:
    return max(0, min(255, int(v)))
