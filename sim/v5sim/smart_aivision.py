"""V5 AI Vision Sensor, device type 0x1D (notes/aivision-protocol.md).

64-byte frames after 02 13. Every 20 ms in data mode the sensor reports
what it detects: object chunks 0x90 (4 objects of 15 bytes each, first
chunk flag 0x01, last 0x80) sent one per brain ack (02 90), then a 0x8F
status frame (temperature, resolutions, counts, one class name per frame).
With nothing detected it sends only the status; the brain lets its object
count time out to 0.

The camera decides what it reports: colour objects need a colour
signature (brain 0x51) whose hue matches; AI-model objects come from the
default model's classes (Ball/Ring/Cube x Blue/Green/Red); the brain's
0x50 "status byte" enables the pipelines (bit set = off: bit0 tags, bit1
colour, bit2 AI; default 5 = colour only). So the world supplies what is
in view (InView), and detect() applies the sensor's own configuration.
Boxes are 320x240 pixels, top-left origin.

AprilTags (notes/vision-and-extras-protocol.md B): with tag detection on
(status bit 0 clear), tags of the selected family (0x50 flag 0x20, byte 9)
are reported first, as four signed 12-bit corners. Colour codes (0x51
format 2) merge adjacent colour blobs, within 7 px, whose colours run in
the code's order along the group's axis (either way round) into one type 2
object: the union box and the angle of the first-to-last member line.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from . import protocol
from .smart_device import LinkMode, SmartDevice

COLOR, CODE, AI, TAG = 1, 2, 4, 8
CFG_DEFAULT = 0x05
CFG_TAGS_OFF, CFG_COLOR_OFF, CFG_AI_OFF = 0x01, 0x02, 0x04
CLASS_NAMES = ("Ball(Blue)", "Ball(Green)", "Ball(Red)", "Ring(Blue)",
               "Ring(Green)", "Ring(Red)", "Cube(Blue)", "Cube(Green)", "Cube(Red)")
CUBE_CLASS = {"blue": 6, "green": 7, "red": 8}
AI_SCORE = 90                     # percent
MAX_OBJECTS = 24
SLOTS_PER_CHUNK = 4
SLOT_LEN = 15
TEMPERATURE_C = 36.0
COLOR_FPS = 30
AI_FPS = 15
MAX_TAGS = 30
MAX_COLOURS_WITH_TAGS = 16
CODE_GAP_PX = 7
CODE_MAX_MEMBERS = 7
TAG_FAMILIES = 4


@dataclass(frozen=True)
class TagInView:
    """An AprilTag in the image: id and corners BL, BR, TR, TL (pixels)."""
    id: int
    corners: tuple[tuple[int, int], ...]
    family: int = 0


@dataclass(frozen=True)
class InView:
    """Something the camera sees: its colour and box (320x240, top-left)."""
    rgb: tuple[int, int, int]
    x: int
    y: int
    w: int
    h: int
    color: str                    # "red" / "blue" / ...: for the AI classes


@dataclass(frozen=True)
class AiVisionSample:
    in_view: tuple[InView, ...] = ()
    tags: tuple[TagInView, ...] = ()


@dataclass(frozen=True)
class AiObject:
    type: int
    id: int
    x: int
    y: int
    w: int
    h: int
    extra: int                    # colour angle (centideg) or AI score (%)
    corners: tuple[tuple[int, int], ...] = ()   # tags: BL, BR, TR, TL


def hue_direction(rgb: tuple[int, int, int]) -> float:
    """Hue angle (rad) from the sensor's opponent axes (2R-G-B, G-B)."""
    r, g, b = rgb
    return math.atan2(math.sqrt(3) * (g - b), 2 * r - g - b)


def saturation(rgb: tuple[int, int, int]) -> float:
    """(R+G+B - 3 min) / (R+G+B): 0 for greys, 1 for pure colours."""
    total = sum(rgb)
    return (total - 3 * min(rgb)) / total if total else 0.0


MIN_SATURATION = 0.10            # below this a colour has no usable hue


def matches(rgb: tuple[int, int, int], sig_rgb: tuple[int, int, int],
            hangle_deg: float, hdsat: float) -> bool:
    """Hue within hangle of the signature's. Greys never match. The note
    reads hdsat as a saturation band [I]; it isn't applied, so a dull and
    a vivid red both match a red signature."""
    if saturation(rgb) < MIN_SATURATION:
        return False
    return (_angle_between(hue_direction(rgb), hue_direction(sig_rgb))
            <= math.radians(hangle_deg))


def _angle_between(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def _pack12(a: int, b: int) -> bytes:
    a, b = (max(-0x800, min(0x7FF, v)) & 0xFFF for v in (a, b))   # signed 12-bit
    return bytes([a & 0xFF, a >> 8 | (b & 0xF) << 4, b >> 4])


def _slot(o: AiObject, seq: int) -> bytes:
    if o.type == TAG:
        return (bytes([o.id & 0xFF, TAG]) + b"".join(_pack12(x, y) for x, y in o.corners)
                + bytes([seq & 0xFF]))
    x, y, w, h = (max(0, min(0xFFF, int(v))) for v in (o.x, o.y, o.w, o.h))
    return (bytes([o.id & 0xFF, o.type,
                   x & 0xFF, (x >> 8) | (y & 0xF) << 4, y >> 4,
                   w & 0xFF, (w >> 8) | (h & 0xF) << 4, h >> 4])
            + struct.pack("<H", o.extra & 0xFFFF) + bytes(4) + bytes([seq & 0xFF]))


def _gap(a: AiObject, b: AiObject) -> tuple[int, int]:
    return (max(a.x, b.x) - min(a.x + a.w, b.x + b.w),
            max(a.y, b.y) - min(a.y + a.h, b.y + b.h))


def _touching_groups(blobs: list[AiObject]) -> list[list[AiObject]]:
    """Blobs whose boxes are within CODE_GAP_PX on both axes, transitively."""
    groups: list[list[AiObject]] = []
    for blob in blobs:
        near = [g for g in groups
                if any(max(_gap(blob, o)) <= CODE_GAP_PX for o in g)]
        merged = [blob] + [o for g in near for o in g]
        groups = [g for g in groups if g not in near] + [merged]
    return groups


def _centre(o: AiObject) -> tuple[float, float]:
    return o.x + o.w / 2, o.y + o.h / 2


def _along_axis(group: list[AiObject]) -> list[AiObject]:
    """Members in order along the principal axis of their centres."""
    pts = [_centre(o) for o in group]
    mx = sum(x for x, _ in pts) / len(pts)
    my = sum(y for _, y in pts) / len(pts)
    sxx = sum((x - mx) ** 2 for x, _ in pts)
    syy = sum((y - my) ** 2 for _, y in pts)
    sxy = sum((x - mx) * (y - my) for x, y in pts)
    theta = math.atan2(2 * sxy, sxx - syy) / 2
    return sorted(group, key=lambda o: ((_centre(o)[0] - mx) * math.cos(theta)
                                        + (_centre(o)[1] - my) * math.sin(theta)))


def _code_object(code_id: int, group: list[AiObject], first: AiObject,
                 last: AiObject) -> AiObject:
    x0, y0 = min(o.x for o in group), min(o.y for o in group)
    x1 = max(o.x + o.w for o in group)
    y1 = max(o.y + o.h for o in group)
    (fx, fy), (lx, ly) = _centre(first), _centre(last)
    a = math.degrees(math.atan2(ly - fy, lx - fx))      # image y down
    v = -a if -a >= 0 else 360 - a                      # blobPackResults [I sign]
    return AiObject(CODE, code_id, x0, y0, x1 - x0, y1 - y0, round(v * 100) % 36000)


class SmartAiVision(SmartDevice[AiVisionSample]):
    DEVICE_TYPE = 0x1D
    FW_VERSION = 0x15
    FRAME_LEN = 0x40
    ACK_COMMANDS = True
    ACK_FLAGS = True
    MIN_INTERVAL_S = 0.020

    def __init__(self, serial: int = 0x00200000):
        super().__init__(serial)
        self.cfg_byte = CFG_DEFAULT
        self._echo = {0x10: 0, 0x12: 0, 0x13: 0, 0x14: 0, 0x15: 0}
        self.signatures: dict[int, tuple[tuple[int, int, int], float, float]] = {}
        self.codes: dict[int, tuple[int, ...]] = {}   # code id -> colour ids
        self.tag_family = 0
        self._queue: list[bytes | None] = []
        self._name_idx = 0
        self._seq = 0
        self._connected = False   # the first data frame must be a status
        self._counts = (0, 0, 0)  # colour/code, tag, AI in the last report

    # ---- brain commands ----------------------------------------------

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] == 0x50:
            self._on_config(frame)
        elif frame[0] == 0x51:
            self._on_signature(frame)

    def _on_config(self, p: bytes) -> None:
        flags = p[1]
        if flags & 0x80:
            self.cfg_byte = CFG_DEFAULT
        for bit, at, echo in ((0x01, 2, 0x10), (0x04, 4, 0x12), (0x08, 5, 0x13),
                              (0x10, 6, 0x14), (0x20, 9, 0x15)):
            if flags & bit:
                self._echo[echo] = p[at]
        if flags & 0x02:
            self.cfg_byte = p[3]
        if flags & 0x20 and p[9] & 7 < TAG_FAMILIES:
            self.tag_family = p[9] & 7

    def _on_signature(self, p: bytes) -> None:
        sig_id, fmt = p[4], p[3]
        if fmt == 1:
            if sig_id == 0xFF:
                self.signatures.clear()
            elif sig_id != 0xFE:
                rgb = (p[5], p[6], p[7])
                if rgb == (0, 0, 0):
                    self.signatures.pop(sig_id, None)
                else:
                    hangle, hdsat = struct.unpack_from("<ff", p, 8)
                    self.signatures[sig_id] = (rgb, hangle, hdsat)
        elif fmt == 2:
            if sig_id == 0xFF:
                self.codes.clear()
            else:
                n = min(p[5], 5)
                # VEXcode always sends 5, unused ones 0 (seen on the wire)
                members = tuple(c for c in struct.unpack_from(f"<{n}H", p, 6) if c)
                if members:
                    self.codes[sig_id] = members
                else:
                    self.codes.pop(sig_id, None)

    # ---- detection -----------------------------------------------------

    def detect(self, sample: AiVisionSample) -> list[AiObject]:
        """What the sensor reports for this view (no side effects)."""
        colours, models = [], []
        for v in sample.in_view:
            if not self.cfg_byte & CFG_COLOR_OFF:
                for sig_id, (rgb, hangle, hdsat) in sorted(self.signatures.items()):
                    if matches(v.rgb, rgb, hangle, hdsat):
                        colours.append(AiObject(COLOR, sig_id, v.x, v.y, v.w, v.h, 0))
                        break
            if not self.cfg_byte & CFG_AI_OFF and v.color in CUBE_CLASS:
                models.append(AiObject(AI, CUBE_CLASS[v.color], v.x, v.y, v.w, v.h,
                                       AI_SCORE))
        colours.sort(key=lambda o: o.w * o.h, reverse=True)   # largest first [I]
        codes = self._codes_in(colours)
        tags = self._tags_in(sample)
        if tags:
            colours = colours[:MAX_COLOURS_WITH_TAGS]
        return (tags + codes + colours + models)[:MAX_OBJECTS]

    def _tags_in(self, sample: AiVisionSample) -> list[AiObject]:
        if self.cfg_byte & CFG_TAGS_OFF:
            return []
        out = []
        for t in sample.tags:
            if t.family != self.tag_family or len(t.corners) != 4:
                continue
            xs, ys = [x for x, _ in t.corners], [y for _, y in t.corners]
            out.append(AiObject(TAG, t.id & 0xFF, min(xs), min(ys), max(xs) - min(xs),
                                max(ys) - min(ys), 0, tuple(t.corners)))
        return out[:MAX_TAGS]

    def _codes_in(self, colours: list[AiObject]) -> list[AiObject]:
        out = []
        for group in _touching_groups(colours):
            if not 2 <= len(group) <= CODE_MAX_MEMBERS:
                continue
            ordered = _along_axis(group)
            ids = tuple(o.id for o in ordered)
            for code_id, members in sorted(self.codes.items()):
                if ids == members:
                    first, last = ordered[0], ordered[-1]
                elif ids[::-1] == members:
                    first, last = ordered[-1], ordered[0]
                else:
                    continue
                out.append(_code_object(code_id, group, first, last))
                break
        return out


    # ---- frames ----------------------------------------------------------

    def on_data_start(self, now: float) -> None:
        self._queue = []                 # a re-pair drops a half-sent report
        self._connected = False          # and starts with a status again

    def chunk_frames(self, objects: list[AiObject], seq: int) -> list[bytes]:
        groups = [objects[i:i + SLOTS_PER_CHUNK]
                  for i in range(0, len(objects), SLOTS_PER_CHUNK)]
        frames = []
        for n, group in enumerate(groups):
            flags = (0x01 if n == 0 else 0) | (0x80 if n == len(groups) - 1 else 0)
            slots = b"".join(_slot(o, seq) for o in group)
            body = slots + b"\xFF" * (SLOTS_PER_CHUNK * SLOT_LEN - len(slots))
            frames.append(self.frame(bytes([0x90, flags]) + body))
        return frames

    def status_frame(self, colour_count: int | None = None) -> bytes:
        """The 0x8F status; each one sent carries the next class name."""
        colours, tags, models = self._counts
        if colour_count is not None:
            colours = colour_count
        p = bytearray(self.FRAME_LEN - 2)
        p[0], p[1] = 0x8F, self.cfg_byte
        struct.pack_into("<H6H", p, 2, int(TEMPERATURE_C * 256),
                         320, 240, 640, 480, 320, 320)
        p[0x11] = self.cfg_byte
        for at, value in self._echo.items():
            p[at] = value
        p[0x18:0x1B] = bytes([colours, tags, models])
        p[0x1D] = 0 if self.cfg_byte & CFG_AI_OFF else AI_FPS
        p[0x1E] = 0 if self.cfg_byte & CFG_COLOR_OFF else COLOR_FPS
        p[0x20], p[0x21] = len(self.signatures), len(self.codes)
        p[0x22], p[0x23] = len(CLASS_NAMES), self._name_idx
        p[0x24], p[0x29] = 2, 1
        name = CLASS_NAMES[self._name_idx].encode()
        p[0x2E:0x2E + len(name)] = name
        self._name_idx = (self._name_idx + 1) % len(CLASS_NAMES)
        return self.frame(bytes(p))

    def data_frame(self, now: float, sample: AiVisionSample) -> bytes:
        """A report: the first chunk now, the rest on each 02 90, then the
        status (built when it goes out, so class names only advance on
        frames the brain actually gets)."""
        objects = self.detect(sample)
        self._counts = (sum(o.type in (COLOR, CODE) for o in objects),
                        sum(o.type == TAG for o in objects),
                        sum(o.type == AI for o in objects))
        if not self._connected or not objects:
            self._connected = True
            self._queue = []
            return self.status_frame()
        self._seq = (self._seq + 1) & 0xFF
        chunks = self.chunk_frames(objects, self._seq)
        self._queue = chunks[1:] + [None]          # None: the status
        return chunks[0]

    def on_brain_frame(self, frame: bytes, now: float) -> list[bytes]:
        replies = super().on_brain_frame(frame, now)
        if (protocol.is_valid(frame) and frame[0] == 0x02 and frame[1] == 0x90
                and self.link == LinkMode.DATA and self._queue):
            nxt = self._queue.pop(0)
            replies.append(self.status_frame() if nxt is None else nxt)
        return replies

