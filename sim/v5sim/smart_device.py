"""Link layer shared by V5 smart sensors (IMU, rotation, distance, ...).

Every smart device firmware runs the same mode machine (notes/
imu-protocol.md 1.4, sensor-protocol.md 1.3):

  0x20 power-up  send 0A info every 50 ms (16-byte frames)
  brain 03       first time: reply 13 <frame length>, else 02 03;
                 mode 0x21: 11 keepalive every <03 bytes 2..3> ms
  brain 02 13    switch RX and TX to the announced frame length
  brain 04       reply 02 04 (13 if still at power-up); mode 0x22: data
                 frame every <04 bytes 2..3> ms; in mode 0x22 a later 04
                 only changes the period
  brain 50..6F   device commands (subclass), acked with 02 <cmd> by
                 devices that do so
  brain silent   after the watchdog (02 13 byte 13 s, default 2) the
                 device starts over at 0x20
                 (real firmware reboots), so a port the brain reset after a
                 stall comes back

Subclasses set the class constants and implement data_payload().
"""
from __future__ import annotations

from enum import IntEnum
from typing import Generic, TypeVar

from . import protocol

INFO_PERIOD_S = 0.050
INFO_SERIAL = 0x12345678
BRAIN_SILENCE_S = 2.0

Sample = TypeVar("Sample")


class LinkMode(IntEnum):
    INFO = 0x20
    IDLE = 0x21
    DATA = 0x22


class SmartDevice(Generic[Sample]):
    DEVICE_TYPE = 0
    FW_VERSION = 0
    FW_ID = 0x4000
    FRAME_LEN = protocol.FRAME_LEN        # announced in 13, used after 02 13
    ACK_COMMANDS = False                  # reply 02 <cmd> to 50..6F
    ACK_FLAGS = False                     # acks carry 1s at bytes 2 and 10
    MIN_INTERVAL_S = 0.0                  # device floor on 03/04 periods;
                                          # with one, a period of 0 means it
    KEEPALIVE = b"\x11"                   # mode 0x21 frame

    def __init__(self, serial: int = INFO_SERIAL):
        self.serial = serial
        self.link = LinkMode.INFO
        self.interval_s = INFO_PERIOD_S
        self.frame_len = protocol.FRAME_LEN
        self._next_tx = 0.0
        self._last_brain: float | None = None
        self.watchdog_s = BRAIN_SILENCE_S

    # ---- frames -------------------------------------------------------

    def frame(self, payload: bytes) -> bytes:
        return protocol.build(payload, self.frame_len)

    def info_frame(self) -> bytes:
        return protocol.build(
            bytes([0x0A, self.FW_VERSION, 0, 0, self.DEVICE_TYPE, 0])
            + self.FW_ID.to_bytes(2, "little") + self.serial.to_bytes(4, "little"))

    def ready_frame(self) -> bytes:
        """13 <length>: always 16 bytes, sent before the switch."""
        return protocol.build(bytes([0x13, self.FRAME_LEN]))

    def ack_frame(self, cmd: int) -> bytes:
        """02 <cmd>: the device's ack of a brain command."""
        if self.ACK_FLAGS:
            body = bytearray(12)
            body[0:3], body[10] = (0x02, cmd, 1), 1
            return self.frame(bytes(body))
        return self.frame(bytes([0x02, cmd]))

    def _period_s(self, frame: bytes, default_ms: int) -> float:
        ms = int.from_bytes(frame[2:4], "little")
        if self.MIN_INTERVAL_S:
            return max(ms / 1000, self.MIN_INTERVAL_S)
        return (ms or default_ms) / 1000

    def data_payload(self, now: float, sample: Sample) -> bytes:
        raise NotImplementedError

    def data_frame(self, now: float, sample: Sample) -> bytes:
        return self.frame(self.data_payload(now, sample))

    # ---- hooks --------------------------------------------------------

    def on_data_start(self, now: float) -> None:
        """Entered data mode (first 04)."""

    def on_command(self, frame: bytes, now: float) -> None:
        """A 50..6F command from the brain (SDK config, reset, ...)."""

    # ---- brain -> device ---------------------------------------------

    def on_brain_frame(self, frame: bytes, now: float) -> list[bytes]:
        if not protocol.is_valid(frame):
            return []
        self._last_brain = now
        cmd = frame[0] & 0x7F
        if cmd == 0x03:
            return self._on_link(frame, LinkMode.IDLE, 1, 0x03, now)
        if cmd == 0x04:
            if self.link == LinkMode.DATA:
                self.interval_s = self._period_s(frame, 100)
                return [self.ack_frame(0x04)] if self.ACK_COMMANDS else []
            reply = self._on_link(frame, LinkMode.DATA, 100, 0x04, now)
            self.on_data_start(now)
            return reply
        if cmd == 0x02 and frame[1] == 0x13:
            self.frame_len = self.FRAME_LEN
            # byte 13: the brain's watchdog in seconds (0 = 2 s)
            self.watchdog_s = float(frame[13]) or BRAIN_SILENCE_S
        elif 0x50 <= cmd <= 0x6F:
            self.on_command(frame, now)
            if self.ACK_COMMANDS:
                return [self.ack_frame(cmd)]
        return []

    def _on_link(self, frame: bytes, mode: LinkMode, default_ms: int,
                 cmd: int, now: float) -> list[bytes]:
        reply = (self.ready_frame() if self.link == LinkMode.INFO
                 else self.ack_frame(cmd))
        self.link = mode
        self.interval_s = self._period_s(frame, default_ms)
        self._next_tx = now + self.interval_s
        return [reply]

    # ---- periodic output ---------------------------------------------

    def reboot(self, now: float) -> None:
        """Start over at power-up: 0A announces, 16-byte frames, and every
        setting back to its default (the subclass's own __init__)."""
        self.__init__(self.serial)
        self._next_tx = now

    def due_frames(self, now: float, sample: Sample) -> list[bytes]:
        # The brain only answers device frames, so a slow data rate is
        # quiet too; real firmware times out on the brain's watchdog.
        silence = max(self.watchdog_s, 2 * self.interval_s)
        if (self.link != LinkMode.INFO and self._last_brain is not None
                and now - self._last_brain > silence):
            self.reboot(now)
        if now < self._next_tx:
            return []
        self._next_tx = max(self._next_tx + self.interval_s, now)
        if self.link == LinkMode.INFO:
            return [self.info_frame()]
        if self.link == LinkMode.IDLE:
            return [self.frame(self.KEEPALIVE)]
        return [self.data_frame(now, sample)]
