"""V5 Robot Radio, device type 8, with a wireless controller behind it
(notes/radio-protocol.md).

Same link layer as the sensors (32-byte frames after 02 13). In data mode
the radio sends one frame per 20 ms in the rotation 80, 85, 85, 85, 85,
with an 86 in place of every 25th:

  80  link status: LS 0x0005 = linked + controller active (0x0002 when
      searching, and then no 85s); keeps the brain's field link alive
  85  inputs of both controllers, same body as the tether's A0: sticks,
      buttons, batteries, byte 6 the competition status (switch 0x01 /
      field 0x80, autonomous 0x10, disabled 0x08, driver 0x20) and byte 7
      the seconds counter the brain's header timer shows
  86  peer info: the controller's firmware id and version

Text and rumble for the controller arrive in 0x55 frames, the same AA
stream as the tether's 0x54, and must be acked (02 55) before the next
data frame. A brain 05 (a second radio plugged in) reboots the radio.
The brain's 52 robot status (every 86) says whether a program is running;
each start restarts the controller's competition-switch clock (byte 7).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from . import protocol
from .smart_controller import (AXIS_BYTES, BATTERY_PERCENT, BATTERY_RAW,
                               BUTTONS, Competition, ControllerScreen,
                               ControllerState, RADIO_FW_VERSION, clamp_axis)
from .radio_signal import RadioSignal
from .smart_device import SmartDevice

LS_LINKED = 0x0005               # any link + controller active
LS_SEARCHING = 0x0002
LINK_PARTNER = 0x0004            # 85 bytes 2..3
LINK_KIND_VEXNET = 1
PARTNER_OFFSET = 6               # partner axes/buttons sit 6 bytes after
CONTROLLER_FW_ID = 0x4000
CONTROLLER_FW_VERSION = 0x4B
PEER_TIME_SLOT, PEER_CHANNEL = 3, 7
ROTATION = 5                     # one 80 per this many frames
PEER_EVERY = 25
PROGRAM_RUNNING = 0x01           # 52 byte 4 bit 0
CDC_CHANNEL = 1                  # 81 / 55 channel of the brain's CDC dispatcher
CDC_CHUNK = 26                   # data bytes per 81 frame

SWITCH, FIELD = 0x01, 0x80
AUTON_BIT, DISABLED_BIT, DRIVER_BIT = 0x10, 0x08, 0x20


def status_byte(phase: Competition, field_control: bool, auton_mode: bool) -> int:
    """85 byte 6 for a match phase. auton_mode: disabled while the match is
    in its autonomous half (before it, or the pause after it)."""
    if phase == Competition.NONE:
        return 0
    base = FIELD | SWITCH if field_control else SWITCH
    if phase == Competition.AUTONOMOUS:
        return base | AUTON_BIT
    if phase == Competition.DRIVER:
        return base | DRIVER_BIT
    return base | DISABLED_BIT | (AUTON_BIT if auton_mode else DRIVER_BIT)


@dataclass(frozen=True)
class RadioSample:
    controller: ControllerState = field(default_factory=ControllerState)
    partner: ControllerState | None = None
    status: int = 0              # 85 byte 6
    match_secs: int = 0          # 85 byte 7
    linked: bool = True


class SmartRadio(SmartDevice[RadioSample]):
    DEVICE_TYPE = 0x08
    FW_VERSION = RADIO_FW_VERSION
    FRAME_LEN = 32
    ACK_COMMANDS = True
    KEEPALIVE = b"\x11\x01"

    def __init__(self, serial: int = 0x12345678, signal: RadioSignal | None = None):
        super().__init__(serial)
        # the VEXnet link's RSSI / quality, reported in the 80 frame; a
        # bridge shares one with the controller so both ends agree
        self.signal = signal or RadioSignal(seed=serial)
        self.screen = ControllerScreen()
        self._n = 0
        self._text_seq: int | None = None
        self._program_running = False
        self.program_started: float | None = None    # last program start (52)
        # the brain's last 52 robot status: 52 00 <batt%> 00 <flags> .. <code>
        self.robot_status: bytes | None = None
        # CDC to/from the robot brain's general dispatcher over the radio
        # 81 frames out, 55 channel-1 replies in
        self._cdc_out = bytearray()
        self._cdc_seq = 0
        self._cdc_reply_seq: int | None = None
        self.cdc_in = bytearray()

    def send_cdc(self, data: bytes) -> None:
        """Queue CDC bytes for the robot brain; replies collect in cdc_in."""
        self._cdc_out += data

    def screen_lines(self) -> list[str]:
        return self.screen.lines()

    @property
    def program_running(self) -> bool:
        """The robot brain's last 52 status says a program is running."""
        return self._program_running

    @property
    def rumble(self) -> str:
        return self.screen.rumble

    # ---- frames -------------------------------------------------------

    def info_frame(self) -> bytes:
        return protocol.build(
            bytes([0x0A, self.FW_VERSION, 0, 0, self.DEVICE_TYPE, 0])
            + self.FW_ID.to_bytes(2, "little") + self.serial.to_bytes(4, "little")
            + b"\x01")

    def ready_frame(self) -> bytes:
        return protocol.build(bytes([0x13, self.FRAME_LEN])
                              + self.serial.to_bytes(4, "little"))

    def link_frame(self, linked: bool) -> bytes:
        p = bytearray(0x18)
        p[0] = 0x80
        p[0x0E:0x10] = self.FW_ID.to_bytes(2, "little")
        p[0x10], p[0x11] = LINK_KIND_VEXNET, self.FW_VERSION
        p[0x16:0x18] = (LS_LINKED if linked else LS_SEARCHING).to_bytes(2, "little")
        if linked:
            sig = self.signal.sample
            p[3] = sig.rssi_dbm & 0xFF                    # RSSI, signed dBm
            p[8:12] = sig.quality.to_bytes(4, "little")   # link quality %
        return self.frame(bytes(p))

    def inputs_frame(self, s: RadioSample) -> bytes:
        p = bytearray(self.FRAME_LEN - 2)
        p[0] = 0x85
        p[2:4] = (LINK_PARTNER if s.partner else 0).to_bytes(2, "little")
        p[4:6] = LS_LINKED.to_bytes(2, "little")
        p[6], p[7] = s.status, s.match_secs & 0xFF
        p[0x0A], p[0x0B] = BATTERY_PERCENT, BATTERY_RAW
        if s.partner:
            p[0x0C], p[0x0D] = 90, 180
        p[0x0F] = LINK_KIND_VEXNET
        _put_controller(p, s.controller, 0)
        _put_controller(p, s.partner or ControllerState(), PARTNER_OFFSET)
        return self.frame(bytes(p))

    def peer_info_frame(self) -> bytes:
        p = bytearray(0x0F)
        p[0], p[6], p[7] = 0x86, PEER_TIME_SLOT, PEER_CHANNEL
        p[0x0C:0x0E] = CONTROLLER_FW_ID.to_bytes(2, "little")
        p[0x0E] = CONTROLLER_FW_VERSION
        return self.frame(bytes(p))

    def cdc_frame(self) -> bytes:
        """81: [1] count, [2] channel 1 (CDC dispatcher), [3] seq, data."""
        chunk = bytes(self._cdc_out[:CDC_CHUNK])
        del self._cdc_out[:CDC_CHUNK]
        self._cdc_seq = (self._cdc_seq + 1) & 0xFF
        return self.frame(bytes([0x81, len(chunk), CDC_CHANNEL, self._cdc_seq]) + chunk)

    def data_frame(self, now: float, s: RadioSample) -> bytes:
        n, self._n = self._n, self._n + 1
        self.signal.update(now)
        if not s.linked or n % ROTATION == 0:
            return self.link_frame(s.linked)
        if self._cdc_out:
            return self.cdc_frame()
        if n % PEER_EVERY == PEER_EVERY - 1:
            return self.peer_info_frame()
        return self.inputs_frame(s)

    # ---- brain -> radio ----------------------------------------------

    def on_brain_frame(self, frame: bytes, now: float) -> list[bytes]:
        if protocol.is_valid(frame) and frame[0] == 0x05:
            ack = self.ack_frame(0x05)       # radioHwInit acks, then restarts
            self.reboot(now)
            return [ack]
        return super().on_brain_frame(frame, now)

    def reboot(self, now: float) -> None:
        """The radio restarts; the controller behind it keeps its screen
        and its clock."""
        screen, started, signal = self.screen, self.program_started, self.signal
        super().reboot(now)
        self.screen, self.program_started, self.signal = screen, started, signal

    @property
    def program_running(self) -> bool:
        """The robot brain's last 52 status says a program is running."""
        return self._program_running

    def on_command(self, frame: bytes, now: float) -> None:
        if frame[0] == 0x52:
            self.robot_status = bytes(frame[:0x0C])
            running = bool(frame[4] & PROGRAM_RUNNING)
            if running and not self._program_running:
                self.program_started = now
            self._program_running = running
            return
        # CDC replies; the brain re-sends a frame until it is acked (02 55)
        if (frame[0] == 0x55 and frame[2] == CDC_CHANNEL
                and frame[3] != self._cdc_reply_seq):
            self._cdc_reply_seq = frame[3]
            n = min(frame[1] & 0x3F, len(frame) - 6)
            self.cdc_in += frame[4:4 + n]
            return
        # channel 0 only; the brain re-sends a frame it thinks was missed
        if frame[0] == 0x55 and frame[2] == 0 and frame[3] != self._text_seq:
            self._text_seq = frame[3]
            n = min(frame[1] & 0x3F, len(frame) - 6)
            self.screen.feed(frame[4:4 + n])


def _put_controller(p: bytearray, c: ControllerState, offset: int) -> None:
    for b in AXIS_BYTES.values():
        p[b + offset] = 0x7F
    for name, value in c.axes.items():
        p[AXIS_BYTES[name] + offset] = (0x7F - clamp_axis(value)) & 0xFF
    mask = 0
    for name in c.buttons:
        mask |= BUTTONS[name]
    p[0x14 + offset:0x16 + offset] = mask.to_bytes(2, "little")
