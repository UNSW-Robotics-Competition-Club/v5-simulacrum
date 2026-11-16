"""A robot's V5 Controller between a field-control brain and the robot brain.

    FIELD brain port P  --smart cable-->  CONTROLLER  --VEXnet-->  ROBOT brain port R

Field side: a tethered SmartController in field mode (notes/field-control-
protocol.md) streams 0xA7 status and records the field app's 0x53 state.
Robot side: a SmartRadio (notes/radio-protocol.md) carries that state to the
robot as 0x85 byte 6 (field control) and reads back the robot's 0x52 status,
so the field sees whether the robot is linked and running a program. The
field app's user-CDC (56) requests, e.g. the robot's VEXos version, go to
the robot brain's CDC dispatcher over the radio and its replies come back.

  python -m v5sim.field_link FIELD_SOCK ROBOT_SOCK [--team NAME]

e.g. field brain instance 0 port 15 (blue2), robot instance 1 radio port 11:
  python -m v5sim.field_link /tmp/v5port15.sock /tmp/v5sim1/v5port11.sock --team 1234A
"""
from __future__ import annotations

import argparse
import select
import socket
import sys
import time
from dataclasses import dataclass, field

from . import protocol
from .smart_controller import ControllerState, RobotStatus, SmartController
from .radio_signal import RadioSignal
from .smart_device import LinkMode
from .smart_radio import RadioSample, SmartRadio

CONNECT_TIMEOUT_S = 30.0
POLL_S = 0.001
STATUS_PERIOD_S = 1.0


def connect(path: str) -> socket.socket:
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while True:
        s = socket.socket(socket.AF_UNIX)
        try:
            s.connect(path)
            s.setblocking(False)
            return s
        except (FileNotFoundError, ConnectionRefusedError):
            s.close()
            if time.monotonic() > deadline:
                raise
            time.sleep(0.2)


@dataclass
class Link:
    """One device on one brain port: bytes in -> frames -> device -> bytes out."""
    sock: socket.socket
    device: SmartController | SmartRadio
    reader: protocol.FrameReader = field(default_factory=protocol.FrameReader)
    trace: frozenset[int] = frozenset()   # brain frame cmds to print

    def receive(self, now: float) -> bool:
        try:
            data = self.sock.recv(4096)
        except BlockingIOError:
            return True
        if not data:
            return False
        self.reader.push(data)
        while True:
            self.reader.length = self.device.frame_len
            frame = self.reader.next_frame()
            if frame is None:
                return True
            if frame[0] in self.trace:
                print(f"{type(self.device).__name__} < {frame.hex(' ')}", flush=True)
            for out in self.device.on_brain_frame(frame, now):
                if out[0] in self.trace:
                    print(f"{type(self.device).__name__} > {out.hex(' ')}", flush=True)
                self._send(out)

    def send_due(self, now: float, sample: object) -> None:
        for out in self.device.due_frames(now, sample):
            if out[0] in self.trace:
                print(f"{type(self.device).__name__} > {out.hex(' ')}", flush=True)
            self._send(out)

    def _send(self, out: bytes) -> None:
        # A stalled brain (QEMU busy, e.g. while another instance boots) fills
        # the socket buffer. Drop the frame, as a lossy smart-port link would,
        # instead of crashing the bridge; the device re-sends on its period.
        try:
            self.sock.sendall(out)
        except BlockingIOError:
            pass


ROBOT_CDC = bytes([0xC9, 0x36, 0xB8, 0x47])   # host -> brain CDC header
REPLY_HEADER = bytes([0xAA, 0x55])
RELAY_TIMEOUT_S = 1.0


def to_robot(request: bytes) -> bytes:
    """A field-app request (header C9 36 B8 50) as the robot brain takes it:
    the host header and the CDC CRC (XMODEM, BE) redone over it."""
    body = ROBOT_CDC + request[len(ROBOT_CDC):-2]
    return body + protocol.crc16_ccitt_false(body, init=0).to_bytes(2, "big")


def take_reply(buf: bytearray) -> bytes | None:
    """Cut one whole CDC2 reply AA 55 <cmd> <len> .. off the front of buf."""
    at = buf.find(REPLY_HEADER)
    if at < 0 or len(buf) < at + 4:
        return None
    n, hdr = buf[at + 3], 4
    if n & 0x80:
        if len(buf) < at + 5:
            return None
        n, hdr = ((n & 0x7F) << 8) | buf[at + 4], 5
    end = at + hdr + n
    if len(buf) < end:
        return None
    reply = bytes(buf[at:end])
    del buf[:end]
    return reply


class Relay:
    """The controller carrying the field app's 56 requests to the robot
    brain over VEXnet and its replies back, one request at a time."""

    def __init__(self, trace: bool = False) -> None:
        self._sent_at: float | None = None
        self._trace = trace

    def _log(self, tag: str, data: bytes) -> None:
        if self._trace:
            print(f"relay {tag} {data.hex(' ')}", flush=True)

    def step(self, now: float, radio: SmartRadio, ctrl: SmartController) -> None:
        if radio.link != LinkMode.DATA:
            self._sent_at = None
            return
        if self._sent_at is not None:
            reply = take_reply(radio.cdc_in)
            if reply is not None:
                self._log("robot >", reply)
                ctrl.relay_reply(reply)
            elif now - self._sent_at < RELAY_TIMEOUT_S:
                return
            else:
                self._log("timeout, robot sent", bytes(radio.cdc_in))
            self._sent_at = None             # answered, or lost: next one
        if ctrl.robot_requests:
            request = to_robot(ctrl.robot_requests.pop(0))
            self._log("robot <", request)
            radio.cdc_in.clear()
            radio.send_cdc(request)
            self._sent_at = now


def radio_sample(ctrl: SmartController) -> RadioSample:
    """In field-tether mode controllerGetButtonFlags returns 0x40 | 53[1],
    so the field app's C9 / D1 / E1 reach the robot's 85 byte 6 as is (the
    brain reads 0x80 field, 0x10 auton, 0x08 disabled, 0x20 driver)."""
    return RadioSample(status=ctrl.field_state | 0x40)


TRACE_FIELD = frozenset({0x53, 0x54, 0xA3})   # field state, queries, replies


def run(field_sock: str, robot_sock: str, team: str, trace: bool = False) -> int:
    signal = RadioSignal()                 # one VEXnet link, seen from both ends
    ctrl = SmartController(field_team=team, signal=signal)
    radio = SmartRadio(signal=signal)
    field_link = Link(connect(field_sock), ctrl,
                      trace=TRACE_FIELD if trace else frozenset())
    robot_link = Link(connect(robot_sock), radio)
    idle = ControllerState()
    relay = Relay(trace)
    next_status = 0.0
    while True:
        select.select([field_link.sock, robot_link.sock], [], [], POLL_S)
        now = time.monotonic()
        for link in (field_link, robot_link):
            if not link.receive(now):
                print(f"{link.device.__class__.__name__}: brain closed the port",
                      file=sys.stderr)
                return 0
        ctrl.robot = radio_robot_status(radio)
        relay.step(now, radio, ctrl)
        field_link.send_due(now, idle)
        robot_link.send_due(now, radio_sample(ctrl))
        if now >= next_status:
            next_status = now + STATUS_PERIOD_S
            r = ctrl.robot
            sig = signal.sample
            print(f"field {ctrl.stage.name} state {ctrl.field_state:02x} | robot "
                  f"radio {radio.link.name} linked {r.linked} flags {r.flags:02x} "
                  f"program {r.program} | rssi {sig.rssi_dbm} dBm signal "
                  f"{sig.quality}%", flush=True)


def radio_robot_status(radio: SmartRadio) -> RobotStatus:
    """The robot's 52 status as DEVFRM09 forwards it into A7."""
    s = radio.robot_status
    if radio.link != LinkMode.DATA or s is None:
        return RobotStatus(linked=False)
    return RobotStatus(linked=True, battery=s[2], flags=s[4], program=s[0x0B])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("field_sock", help="the field brain's port socket")
    ap.add_argument("robot_sock", help="the robot brain's (free) radio port socket")
    ap.add_argument("--team", default="SIM", help="team name, up to 10 characters")
    ap.add_argument("--trace", action="store_true",
                    help="print the field app's 53 / 54 frames and the A3 replies")
    args = ap.parse_args(argv)
    if len(args.team) > 10 or not args.team.isascii():
        ap.error("--team: up to 10 ASCII characters")
    return run(args.field_sock, args.robot_sock, args.team, args.trace)


if __name__ == "__main__":
    sys.exit(main())
