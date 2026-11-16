"""V5 Controller, tethered to a smart port (device type 9),
notes/controller-protocol.md.

  announce   0A info (type 9, version 0x4B) every 50 ms until the brain
             answers anything
  ready      13 20 <id>: the port will use 32-byte frames; resent every
             200 ms until the brain's 02 13 arrives
  stream     5 ms after that, A0 frames (32 B) every 20 ms with sticks,
             buttons and the competition byte. Each valid A0 marks the
             controller present (B33c 0x10000), which also enables motors.

Axes use the SDK's -127..127 scale (vexControllerGet); the wire carries
0x7F - value. Brain replies need no answer.

Screen text and rumble (notes/controller-screen.md) arrive as 0x54 replies:
54 <n> 00 <seq> + n bytes of a stream of inner frames
AA 5A 56 <len> 33 <line> <col> <n> <text> <crc16, init 0>; line 1..3 text
(n = 0 clears it), 4 = rumble pattern, 0x0F with n = 0 clears the screen.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum

from . import protocol
from .radio_signal import RadioSignal, SignalSample

FW_VERSION = 0x4B
FW_ID = 0x4000
DEVICE_TYPE = 0x09
FRAME_LEN = 32
INFO_PERIOD_S = 0.050
READY_RETRY_S = 0.200
BRAIN_SILENCE_S = 2.0             # then start over (controller-protocol.md 8)
STREAM_DELAY_S = 0.005
DATA_PERIOD_S = 0.020

SCREEN_LINES = 3
SCREEN_COLS = 40                  # firmware buffer; 19 are visible [I]
SCREEN_VISIBLE = 19
LINE_RUMBLE = 4
LINE_CLEAR_ALL = 0x0F
CMD_SCREEN = 0x33
PRIMARY = 0x5A                    # inner frame byte 1; 0x5B = partner

LINK_TETHERED = 0x01
BATTERY_PERCENT = 100
BATTERY_RAW = 190                 # raw * 10 + 2100 = 4000 mV

# vex::controller axis name -> A0 byte (SDK channel in comments)
AXIS_BYTES = {"Axis4": 0x11,      # ch0 left X
              "Axis3": 0x10,      # ch1 left Y
              "Axis1": 0x13,      # ch2 right X
              "Axis2": 0x12}      # ch3 right Y
BUTTONS = {"L1": 0x2000, "L2": 0x1000, "R1": 0x0010, "R2": 0x0020,
           "Up": 0x0004, "Down": 0x0001, "Left": 0x0008, "Right": 0x0002,
           "X": 0x0800, "B": 0x0200, "Y": 0x0100, "A": 0x0400}


class Competition(Enum):
    """Byte 6 of A0: what a tether (or competition switch) reports."""

    NONE = 0x00                   # plain driver control, enabled
    DRIVER = 0x01                 # competition control present, driver
    AUTONOMOUS = 0x11
    DISABLED = 0x09


@dataclass(frozen=True)
class ControllerState:
    axes: dict[str, int] = field(default_factory=dict)    # -127..127
    buttons: frozenset[str] = frozenset()
    competition: Competition = Competition.NONE

    def with_input(self, axes: dict[str, int] | None = None,
                   buttons: frozenset[str] | None = None,
                   competition: Competition | None = None) -> "ControllerState":
        return replace(self,
                       axes=self.axes if axes is None else axes,
                       buttons=self.buttons if buttons is None else buttons,
                       competition=self.competition if competition is None else competition)


def clamp_axis(v: float) -> int:
    return max(-127, min(127, round(v)))


class ControllerScreen:
    """The controller's text lines and rumble, fed the brain's AA byte
    stream (0x54 frames on a tether, 0x55 through the radio)."""

    def __init__(self) -> None:
        self._screen = [[" "] * SCREEN_COLS for _ in range(SCREEN_LINES)]
        self.rumble = ""
        self._rx = bytearray()

    def lines(self) -> list[str]:
        """The three lines as the controller shows them."""
        return ["".join(row[:SCREEN_VISIBLE]).rstrip() for row in self._screen]

    def reset_stream(self) -> None:
        self._rx.clear()

    def feed(self, data: bytes) -> None:
        """Parse AA frames out of the text byte stream. The stream also
        carries other traffic, so an AA that doesn't start a plausible,
        CRC-valid frame is skipped one byte at a time."""
        self._rx += data
        while True:
            start = self._rx.find(0xAA)
            if start < 0:
                self._rx.clear()
                return
            del self._rx[:start]
            if len(self._rx) < 4:
                return
            size = self._rx[3] + 4
            if self._rx[1] not in (PRIMARY, PRIMARY + 1) or not 7 <= size <= 0x84:
                del self._rx[:1]                     # not a frame start
                continue
            if len(self._rx) < size:
                return                               # wait for the rest
            frame = bytes(self._rx[:size])
            if protocol.crc16_ccitt_false(frame, init=0) != 0:
                del self._rx[:1]
                continue
            del self._rx[:size]
            if frame[1] == PRIMARY and size >= 8:
                self._on_screen_command(frame[4], frame[5:-2])

    def _on_screen_command(self, cmd: int, p: bytes) -> None:
        if cmd != CMD_SCREEN or len(p) < 3:
            return
        line, col, n = p[0], p[1], p[2]
        text = p[3:3 + n].decode("ascii", "replace")
        if line == LINE_CLEAR_ALL and n == 0:
            self._screen = [[" "] * SCREEN_COLS for _ in range(SCREEN_LINES)]
        elif line == LINE_RUMBLE:
            self.rumble = text[:8]
        elif 1 <= line <= SCREEN_LINES:
            row = self._screen[line - 1]
            if n == 0:
                row[:] = [" "] * SCREEN_COLS
            elif col < SCREEN_COLS:
                text = text[:SCREEN_COLS - col]
                row[col:col + len(text)] = list(text)


class Stage(Enum):
    ANNOUNCE = 0
    READY = 1
    STREAM = 2


@dataclass(frozen=True)
class RobotStatus:
    """What the controller knows about its robot over VEXnet: the robot
    brain's 52 status (DEVFRM09 copies byte 4 to A7 byte 11 and byte 0x0B,
    through ctrlSetPendingMode, to A7 byte 14 = the field's "program")."""
    linked: bool = True
    battery: int = BATTERY_PERCENT
    flags: int = 0x01             # 52 byte 4; bit 0 = program running
    program: int = 1              # 52 byte 0x0B; 0 = no program


FIELD_STATUS = 0xA7              # what a controller streams to a field brain
FIELD_STATE = 0x53               # field app -> controller: byte 1 = state
FIELD_REPLY = 0xA3               # controller -> field app: CDC reply bytes
FIELD_DISABLED = 0xC9            # until the field app says otherwise
TEAM_LEN = 10

# The field app asks the controller for its radio's firmware once per link,
# a CDC request tunnelled in a 54 frame; it shows "Ctrl radio" and only
# turns the station green when the answer is > 0x0100002F
# (notes/field-control-protocol.md 8-10).
FIELD_CDC = bytes([0xC9, 0x36, 0xB8, 0x50])  # field app -> controller CDC header
FIELD_QUERY = FIELD_CDC + b"\x58"           # + <ext>: for the controller itself
USER_CDC = 0x56                             # + <ext>: tunnelled to the robot brain
QUERY_CTRL_RADIO = 0x39
# The V5 radio firmware (DEVFRM08, header @0x500 `00 02 08 30`: type 8,
# version 0x30) runs in the controller's radio too; the app shows
# 0x01000000 | version as "1.0.0b48"
RADIO_FW_VERSION = 0x30
CTRL_RADIO_VERSION = 0x01000000 | RADIO_FW_VERSION
CDC_ACK = 0x76
A3_DATA = FRAME_LEN - 6                     # CDC bytes per A3 frame


def a3_frames(cdc: bytes) -> list[bytes]:
    """CDC reply bytes for the field app, as A3 frames: A3 <n> 00 00 <n
    bytes>; the app feeds them to its CDC parser in order."""
    return [protocol.build(bytes([FIELD_REPLY, len(chunk), 0, 0]) + chunk, FRAME_LEN)
            for chunk in (cdc[i:i + A3_DATA] for i in range(0, len(cdc), A3_DATA))]


def field_reply(ext: int, data: bytes) -> bytes:
    """An A3 frame carrying the CDC reply AA 55 58 <len> <ext> <data>
    <CRC16 XMODEM, BE>."""
    body = bytes([ext]) + data
    inner = bytes([0xAA, 0x55, 0x58, len(body) + 2]) + body
    inner += protocol.crc16_ccitt_false(inner, init=0).to_bytes(2, "big")
    (frame,) = a3_frames(inner)
    return frame


def cdc_requests(payload: bytes, header: bytes, cmd: int) -> list[bytes]:
    """Whole CDC2 extended requests <header> <cmd> <ext> <len> <data> <crc>
    in a 54 frame's payload (len is one byte, or two with bit 7 set)."""
    found, at = [], payload.find(header + bytes([cmd]))
    while 0 <= at:
        p = at + len(header) + 2
        if p < len(payload):
            n, p = payload[p], p + 1
            if n & 0x80 and p < len(payload):
                n, p = ((n & 0x7F) << 8) | payload[p], p + 1
            end = p + n + 2
            if end <= len(payload):
                found.append(bytes(payload[at:end]))
        at = payload.find(header + bytes([cmd]), at + 1)
    return found


def ctrl_radio_reply(version: int = CTRL_RADIO_VERSION) -> bytes:
    """Answer to 58 39; the app reads d[15] | (u16 d[12..13] & 0x3FFF) << 8
    | (u16 d[12..13] & 0xC000) << 10."""
    d = bytearray(16)
    d[0] = CDC_ACK
    hi = ((version >> 8) & 0x3FFF) | ((version >> 10) & 0xC000)
    d[12:14] = hi.to_bytes(2, "little")
    d[15] = version & 0xFF
    return field_reply(QUERY_CTRL_RADIO, bytes(d))


class SmartController:
    """A tethered V5 Controller. With `field_team` it is a robot's controller
    smart-cabled to a field-control brain (notes/field-control-protocol.md):
    it streams 0xA7 status frames (robot linked over VEXnet, batteries,
    team name) instead of 0xA0, and records the field app's 0x53 state."""

    def __init__(self, serial: int = 0x12345678, field_team: str | None = None,
                 signal: RadioSignal | None = None):
        self.serial = serial
        self.field_team = field_team
        # the controller radio's link to the robot (RSSI, quality, channel);
        # a bridge shares one with the robot's radio so both ends agree
        self.signal = signal or RadioSignal(seed=serial)
        self.field_state = FIELD_DISABLED
        self.robot = RobotStatus()
        self._replies: list[bytes] = []   # field-app CDC answers, sent first
        # the field app's user-CDC (56) requests for the robot brain, in
        # order; whoever carries them over VEXnet (field_link) answers with
        # relay_reply()
        self.robot_requests: list[bytes] = []
        self.stage = Stage.ANNOUNCE
        self.frame_len = protocol.FRAME_LEN
        self._next_tx = 0.0
        self._last_brain: float | None = None
        self.screen = ControllerScreen()

    def screen_lines(self) -> list[str]:
        """The three lines as the controller shows them."""
        return self.screen.lines()

    @property
    def rumble(self) -> str:
        return self.screen.rumble

    @property
    def link(self) -> Stage:            # for status lines, like the others
        return self.stage

    def info_frame(self) -> bytes:
        return protocol.build(
            bytes([0x0A, FW_VERSION, 0, 0, DEVICE_TYPE, 0])
            + FW_ID.to_bytes(2, "little") + self.serial.to_bytes(4, "little"))

    def ready_frame(self) -> bytes:
        return protocol.build(bytes([0x13, FRAME_LEN, 0, 0])
                              + self.serial.to_bytes(4, "little"))

    def data_frame(self, s: ControllerState) -> bytes:
        body = bytearray(FRAME_LEN - 2)
        body[0] = 0xA0
        body[2] = LINK_TETHERED
        body[6] = s.competition.value
        body[0x0A], body[0x0B] = BATTERY_PERCENT, BATTERY_RAW
        for b in (0x10, 0x11, 0x12, 0x13, 0x16, 0x17, 0x18, 0x19):
            body[b] = 0x7F
        for name, value in s.axes.items():
            body[AXIS_BYTES[name]] = (0x7F - clamp_axis(value)) & 0xFF
        mask = 0
        for name in s.buttons:
            mask |= BUTTONS[name]
        body[0x14:0x16] = mask.to_bytes(2, "little")
        return protocol.build(bytes(body), FRAME_LEN)

    def field_status_frame(self, sig: SignalSample | None = None) -> bytes:
        """0xA7 (ctrlBuildStatusPacketA7): robot link and status, the radio
        link (RSSI, link status, channel / mode, signal), the field state
        echoed back, batteries, VEXnet, the team name."""
        r = self.robot
        sig = sig or self.signal.sample
        body = bytearray(FRAME_LEN - 2)
        body[0] = FIELD_STATUS
        body[2] = 0x01 if r.linked else 0x00    # robot linked over VEXnet
        body[6] = self.field_state
        if r.linked:
            body[1] = sig.rssi_dbm & 0xFF       # "RSSI", signed dBm
            body[4:6] = sig.link_status.to_bytes(2, "little")   # "Radio Status"
            body[8] = r.battery                 # robot battery
            body[11] = r.flags                  # "Brain Status"
            body[14] = r.program                # "Program"; 0 = None
            body[29] = sig.quality              # "Signal", percent
        body[9] = BATTERY_PERCENT               # controller battery
        body[15] = 1                      # radio type VEXnet
        body[16] = sig.channel            # 54..81 shows "Comp", else "Pit"
        body[17] = sig.mode               # shown as mode + 1
        team = (self.field_team or "").encode("ascii", "replace")[:TEAM_LEN]
        body[18:18 + len(team)] = team
        return protocol.build(bytes(body), FRAME_LEN)

    def on_brain_frame(self, frame: bytes, now: float) -> list[bytes]:
        if not protocol.is_valid(frame):
            return []
        self._last_brain = now
        if self.stage == Stage.ANNOUNCE:
            self.stage = Stage.READY
            self._next_tx = now + READY_RETRY_S
            return [self.ready_frame()]
        if self.stage == Stage.READY and frame[0] == 0x02 and frame[1] == 0x13:
            self.stage = Stage.STREAM
            self.frame_len = FRAME_LEN
            self._next_tx = now + STREAM_DELAY_S
            if self.field_team is not None:
                # the app asks 58 39 once per link and keeps its reply
                # callback registered, so answer up front: a missed query
                # would leave "Ctrl radio" 0.0.0 (and the station orange)
                self._replies.append(ctrl_radio_reply())
        elif (self.stage == Stage.STREAM and self.field_team is not None
              and frame[0] == FIELD_STATE and len(frame) == FRAME_LEN):
            self.field_state = frame[1]
        elif self.stage == Stage.STREAM and frame[0] == 0x54 and len(frame) == FRAME_LEN:
            n = min(frame[1] & 0x3F, FRAME_LEN - 6)
            payload = frame[4:4 + n]
            if self.field_team is not None and FIELD_CDC in payload:
                self._on_field_cdc(payload)
            else:
                self.screen.feed(payload)
        return []

    def _on_field_cdc(self, payload: bytes) -> None:
        for req in cdc_requests(payload, FIELD_CDC, 0x58):
            if req[5] == QUERY_CTRL_RADIO:
                self._replies.append(ctrl_radio_reply())
        # DEVFRM09 ctrlDispatchCommand: 58 is the controller's own, every
        # other command goes to the robot brain unchanged (56 22, 20, f6 ...)
        self.robot_requests += cdc_requests(payload, FIELD_CDC, USER_CDC)

    def relay_reply(self, cdc: bytes) -> None:
        """The robot brain's reply to a relayed request, back to the field."""
        self._replies += a3_frames(cdc)

    def reboot(self, now: float) -> None:
        """Start over: announce again with 16-byte frames."""
        self.stage = Stage.ANNOUNCE
        self.frame_len = protocol.FRAME_LEN
        self.screen.reset_stream()
        self._replies.clear()
        self.robot_requests.clear()
        self._last_brain = None
        self._next_tx = now

    def due_frames(self, now: float, s: ControllerState) -> list[bytes]:
        if (self.stage != Stage.ANNOUNCE and self._last_brain is not None
                and now - self._last_brain > BRAIN_SILENCE_S):
            self.reboot(now)
        if now < self._next_tx:
            return []
        if self.stage == Stage.ANNOUNCE:
            self._next_tx = now + INFO_PERIOD_S
            return [self.info_frame()]
        if self.stage == Stage.READY:
            self._next_tx = now + READY_RETRY_S
            return [self.ready_frame()]
        self._next_tx = max(self._next_tx + DATA_PERIOD_S, now)
        if self.field_team is not None:
            if self._replies:
                return [self._replies.pop(0)]
            return [self.field_status_frame(self.signal.update(now))]
        return [self.data_frame(s)]
