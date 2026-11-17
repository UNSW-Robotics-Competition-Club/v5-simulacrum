"""A smart motor that runs VEX's own firmware in QEMU (machine vex-motor).

The motor's LPC824 runs DEVFRM02 (11 W) or DEVFRM05 (5.5 W) unchanged in
its own QEMU process; this class stands in for SmartMotor on the robot's
side (notes/motor-emulation.md):

  brain <-> port socket       smart-port bytes, relayed as they come
  physics <-> plant socket    every step: ADC samples (current, terminal
                              and battery voltage, temperature) and the
                              encoder go in; the H-bridge pins come back
                              and become the Drive for motor.py

The firmware does its own control, current limiting and faults. The ADC
scales below are the firmware's own conversions
(motorGetPositionDelta, motorSetFloatScaled, motorMonitorTick,
motorTempFromAdc in DEVFRM02).

A board that falls silent (QEMU stopped, hung or paused in gdb) counts as
unplugged: it drives nothing and the brain's frames go nowhere, instead of
the shaft running on at the last bridge output. When it talks again it is
plugged back in, which power-cycles it: QEMU resets the board and the
firmware boots and announces itself afresh.
"""
from __future__ import annotations

import atexit
import ctypes
import math
import os
import signal
import socket
import struct
import subprocess
import shutil
import sys
import tempfile
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

from . import protocol
from .motor import Cartridge, Drive, MotorState
from .smart_motor import (COUNTS_PER_SHAFT_REV, FIRMWARE_DIR, Command, LinkMode,
                          apply_control)

QEMU = os.environ.get("V5SIM_QEMU",
                      str(Path.home() / "re/qemu/build/qemu-system-arm"))
CONNECT_TIMEOUT_S = 10.0

NUM_ADC = 12
NUM_SCT_OUT = 6
PLANT_IN = struct.Struct(f"<c{NUM_ADC}HddI")
PLANT_OUT = struct.Struct(f"<cII{NUM_SCT_OUT}B{NUM_SCT_OUT}fQ")
PLANT_RESET = b"R"           # power-cycle the board; QEMU echoes it once done
PIN_UNASSIGNED = 0xFF

ADC_MAX = 0xFFF


@dataclass(frozen=True)
class Board:
    """Where a motor board's analogue signals sit and how its firmware
    scales them (each firmware's ADC ISR, motorGetPositionDelta,
    motorSetFloatScaled, motorMonitorTick and motorTempFromAdc)."""
    image: str
    ch_current: int
    ch_terminal_a: int      # status volts = (B - A) / terminal_counts_per_mv
    ch_terminal_b: int
    ch_battery: int
    ch_temperature: int
    terminal_counts_per_mv: float
    battery_counts_per_mv: float
    current_counts_per_ma: float
    # 11 W: the current is sensed while the PWM is off and the firmware
    # divides its reading by the off fraction (x1.25) once that is > 2%
    off_time_current: bool
    temp_table: tuple[int, ...]
    temp_rising: bool       # table order: 11 W rises with temperature
    # H-bridge inputs and driver enable (motorSetOutputMode); the PWM
    # (SCT_OUT4) is switched onto one input. Encoder B (PININT0 ISR).
    pin_in1: int
    pin_in2: int
    pin_enable: int
    pin_encoder_b: int
    # a fault input the main loop reads on every pass: QEMU idles the core
    # there until the next interrupt (lpc82x idle-pin)
    pin_idle_poll: int
    # encoder counts per output revolution; None: 50 per internal motor
    # revolution, so it depends on the cartridge (11 W)
    counts_per_output_rev: float | None = None


BOARD_11W = Board(
    image="DEVFRM02.BIN", ch_current=2, ch_terminal_a=3, ch_terminal_b=4,
    ch_battery=5, ch_temperature=7,
    terminal_counts_per_mv=0.16234, battery_counts_per_mv=0.16234,
    current_counts_per_ma=0.6552, off_time_current=True,
    # temperature (idx + 1) * 5 degC when the reading is below table[idx]
    temp_table=(1372, 1562, 1756, 1950, 2141, 2325, 2501, 2665, 2818, 2958,
                3085, 3199, 3300, 3390, 3471, 3542, 3604, 3659, 3708, 3751,
                3788, 3821, 3850, 3876, 3895),
    temp_rising=True, pin_in1=24, pin_in2=25, pin_enable=15, pin_encoder_b=7,
    pin_idle_poll=8)

BOARD_5W5 = Board(
    image="DEVFRM05.BIN", ch_current=8, ch_terminal_a=10, ch_terminal_b=3,
    ch_battery=4, ch_temperature=2,
    terminal_counts_per_mv=0.12366, battery_counts_per_mv=0.11833,
    current_counts_per_ma=1.016, off_time_current=False,
    # temperature idx * 5 degC for the first table[idx] below the reading
    temp_table=(2782, 2586, 2385, 2185, 1989, 1801, 1622, 1456, 1302, 1161,
                1035, 920, 819, 728, 648, 577, 514, 459, 410, 367, 329, 296,
                266, 240),
    temp_rising=False, pin_in1=26, pin_in2=27, pin_enable=16, pin_encoder_b=20,
    pin_idle_poll=10,
    # it reports 15/8 of these, i.e. green-cartridge units (motor55-protocol.md)
    counts_per_output_rev=480)

OFF_TIME_GAIN = 1.25
MIN_OFF_FRACTION = 0.02

PWM_OUTPUT = 4
BATTERY_V = 12.8
# A healthy board sends a frame at least every ~57 ms, linked or not (both
# images, measured); this much silence means it has stopped.
UNPLUG_S = 0.5
_LINK_BY_FRAME = {0x0A: LinkMode.INFO, 0x13: LinkMode.IDLE, 0x11: LinkMode.IDLE,
                  0x76: LinkMode.DATA}


class Plug(Enum):
    IN = auto()              # on the brain's port
    OUT = auto()             # silent too long: unplugged, unpowered
    POWERING_UP = auto()     # talking again: QEMU is resetting the board


@dataclass(frozen=True)
class PlantOut:
    """What the motor board drives: pin levels and the SCT outputs."""
    levels: int
    directions: int
    sct_pins: tuple[int, ...]
    sct_duty: tuple[float, ...]
    vm_ns: int

    def pin_fraction(self, pin: int) -> float:
        """Fraction of the time the pin drives high. A pin that is neither
        an SCT output nor a GPIO output floats, which the driver reads as
        low (input pins report their pull-up in levels)."""
        for out, assigned in enumerate(self.sct_pins):
            if assigned == pin:
                return self.sct_duty[out]
        if not (self.directions >> pin) & 1:
            return 0.0
        return float((self.levels >> pin) & 1)


def encode_plant_in(adc: list[int], position: float, velocity: float,
                    gpio_in: int = 0xFFFFFFFF) -> bytes:
    if len(adc) != NUM_ADC:
        raise ValueError(f"need {NUM_ADC} ADC samples, got {len(adc)}")
    return PLANT_IN.pack(b"I", *(max(0, min(ADC_MAX, a)) for a in adc),
                         position, velocity, gpio_in & 0xFFFFFFFF)


def decode_plant_out(msg: bytes) -> PlantOut:
    fields = PLANT_OUT.unpack(msg)
    if fields[0] != b"O":
        raise ValueError(f"not a plant reply: {msg[:1]!r}")
    pins = fields[3:3 + NUM_SCT_OUT]
    duty = fields[3 + NUM_SCT_OUT:3 + 2 * NUM_SCT_OUT]
    return PlantOut(fields[1], fields[2], tuple(pins), tuple(duty), fields[-1])


def bridge_drive(board: Board, out: PlantOut, battery_v: float) -> Drive:
    """Average bridge voltage, or None when the driver is off (coast).

    IN1/IN2 follow the usual IN/IN truth table: 01 forward, 10 reverse,
    11 brake, 00 coast. With one input held and the other PWMed, the time
    average is (IN2 - IN1) x battery whichever input is held. Forward is
    the direction the firmware means by a positive voltage command."""
    if not out.pin_fraction(board.pin_enable):
        return None
    a = out.pin_fraction(board.pin_in1)
    b = out.pin_fraction(board.pin_in2)
    if a == 0.0 and b == 0.0:
        return None
    return (b - a) * battery_v


def temperature_counts(board: Board, temp_c: float) -> int:
    """An ADC reading the board's motorTempFromAdc turns into temp_c
    (5 degC steps)."""
    t = board.temp_table
    if board.temp_rising:
        k = min(max(round(temp_c / 5), 1), len(t))
        return t[0] - 50 if k == 1 else (t[k - 2] + t[k - 1]) // 2
    k = min(max(round(temp_c / 5), 0), len(t) - 1)
    return t[0] + 50 if k == 0 else (t[k - 1] + t[k]) // 2


def current_counts(board: Board, current_a: float, pwm_high: float) -> int:
    sensed = abs(current_a) * 1000 * board.current_counts_per_ma
    off = min(1.0, (1.0 - pwm_high) * OFF_TIME_GAIN)
    if board.off_time_current and pwm_high > 0 and off > MIN_OFF_FRACTION:
        sensed *= off
    return round(sensed)


def adc_samples(board: Board, state: MotorState, battery_v: float,
                pwm_high: float) -> list[int]:
    adc = [ADC_MAX // 2] * NUM_ADC
    v_mv = state.voltage_v * 1000
    adc[board.ch_terminal_a] = round(max(-v_mv, 0.0) * board.terminal_counts_per_mv)
    adc[board.ch_terminal_b] = round(max(v_mv, 0.0) * board.terminal_counts_per_mv)
    adc[board.ch_battery] = round(battery_v * 1000 * board.battery_counts_per_mv)
    adc[board.ch_current] = current_counts(board, state.current_a, pwm_high)
    adc[board.ch_temperature] = temperature_counts(board, state.temperature_c)
    return adc


def _child_setup() -> None:
    """Child side of Popen (Linux): get SIGTERM when the robot sim exits, and
    fire timers on time. The encoder edges are QEMU timers and the firmware
    times them; Linux's default 50 us timer slack on QEMU's poll made its
    velocity readings jump by several percent. Both settings survive exec."""
    try:
        libc = ctypes.CDLL("libc.so.6")
        libc.prctl(1, signal.SIGTERM)       # PR_SET_PDEATHSIG
        libc.prctl(29, 1)                   # PR_SET_TIMERSLACK, 1 ns
    except OSError:
        pass


# Without Linux's parent-death signal (macOS), the motor QEMUs are stopped
# when the robot sim exits: robot.main turns SIGTERM/SIGHUP into SystemExit,
# so its own cleanup and this atexit hook run.
_LIVE: set["FirmwareMotor"] = set()


@atexit.register
def _close_all() -> None:
    for m in list(_LIVE):
        m.close()


def _connect(path: Path, deadline: float, proc: subprocess.Popen) -> socket.socket:
    while True:
        s = socket.socket(socket.AF_UNIX)
        try:
            s.connect(str(path))
            s.setblocking(False)
            return s
        except OSError:
            s.close()
            if proc.poll() is not None or time.monotonic() > deadline:
                raise
            time.sleep(0.05)


class _Channel:
    """A non-blocking socket to QEMU. Sends queue whole messages and never
    block the physics loop; past the backlog new messages are dropped whole,
    so the stream is never torn. A closed socket (QEMU gone) raises
    ConnectionError, which ends the robot sim like a lost brain."""

    def __init__(self, sock: socket.socket, name: str, backlog: int):
        self.sock, self.name, self.backlog = sock, name, backlog
        self._out = b""

    def send(self, data: bytes, force: bool = False) -> bool:
        self.flush()
        if len(self._out) > self.backlog and not force:
            return False
        self._out += data
        self.flush()
        return True

    def flush(self) -> None:
        if not self._out:
            return
        try:
            sent = self.sock.send(self._out)
        except BlockingIOError:
            return
        except OSError as e:
            raise ConnectionError(f"motor QEMU {self.name} socket: {e}") from e
        self._out = self._out[sent:]

    def recv(self) -> bytes:
        data = b""
        while True:
            try:
                chunk = self.sock.recv(4096)
            except BlockingIOError:
                return data
            except OSError as e:
                raise ConnectionError(f"motor QEMU {self.name} socket: {e}") from e
            if not chunk:
                raise ConnectionError(f"motor QEMU closed its {self.name} socket")
            data += chunk

    def close(self) -> None:
        self.sock.close()


class FirmwareMotor:
    """Drop-in for SmartMotor backed by the real firmware in QEMU."""

    frame_len = protocol.FRAME_LEN
    firmware = None                  # the image is what QEMU runs

    def __init__(self, cartridge: Cartridge = Cartridge.GREEN_18,
                 name: str = "motor", workdir: Path | None = None):
        self.is_5_5w = cartridge == Cartridge.MOTOR_5_5W
        self.board = BOARD_5W5 if self.is_5_5w else BOARD_11W
        # the plant: the 5.5 W motor as its firmware's voltage cap drives it
        self.cartridge = Cartridge.MOTOR_5_5W_FW if self.is_5_5w else cartridge
        # what the brain's counts mean: the 5.5 W reports green units
        self.brain_counts_per_rev = (900 if self.is_5_5w else
                                     COUNTS_PER_SHAFT_REV * cartridge.ratio)
        self.battery_v = BATTERY_V
        self.command = Command()         # the brain's last 0x56 (status, view)
        self.link = LinkMode.INFO        # from the frames it sends
        self.out: PlantOut | None = None
        self.name = name
        self.plug = Plug.IN
        self._last_heard: float | None = None
        self._fault_byte = 0
        image = FIRMWARE_DIR / self.board.image
        if not image.is_file():
            raise FileNotFoundError(f"motor firmware {image} (unpack VEXos first)")
        self._own_dir = workdir is None
        # /tmp: QEMU's sockets live here, and macOS's own temp folder
        # (/var/folders/...) is long for a unix socket path (104 bytes)
        self._dir = Path(workdir or tempfile.mkdtemp(prefix=f"v5{name}-", dir="/tmp"))
        self._port: _Channel | None = None
        self._plant: _Channel | None = None
        port_sock, plant_sock = self._dir / "port.sock", self._dir / "plant.sock"
        with open(self._dir / "qemu.log", "wb") as log:
            self._proc = subprocess.Popen(
                [QEMU, "-M", "vex-motor", "-bios", str(image),
                 "-display", "none", "-monitor", "none", "-serial", "none",
                 "-chardev", f"socket,id=port,path={port_sock},server=on,wait=off",
                 "-chardev", f"socket,id=plant,path={plant_sock},server=on,wait=off",
                 "-global", f"lpc82x-periph.enc-b-pin={self.board.pin_encoder_b}",
                 "-global", f"lpc82x-periph.idle-pin={self.board.pin_idle_poll}",
                 *os.environ.get("V5SIM_MOTOR_QEMU_ARGS", "").split()],
                stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                preexec_fn=_child_setup if sys.platform.startswith("linux") else None)
        _LIVE.add(self)
        try:
            deadline = time.monotonic() + CONNECT_TIMEOUT_S
            self._port = _Channel(_connect(port_sock, deadline, self._proc), "port", 4096)
            self._plant = _Channel(_connect(plant_sock, deadline, self._proc), "plant",
                                   PLANT_IN.size * 64)
        except OSError as e:
            self.close(keep_log=True)
            raise RuntimeError(f"motor QEMU did not start: {e} "
                               f"(log: {self._dir / 'qemu.log'})") from e
        self._reader = protocol.FrameReader()
        self._plant_buf = b""

    # ---- SmartMotor interface -----------------------------------------

    @property
    def effective_current_limit_a(self) -> float:
        return math.inf              # the firmware limits current itself

    def faults(self, state: MotorState) -> int:
        """Byte 13 of the firmware's last status frame."""
        return self._fault_byte

    def on_brain_frame(self, frame: bytes, now: float) -> list[bytes]:
        if self.plug is not Plug.IN:
            return []                # nothing on the other end of the cable
        if frame[0] == 0x56:
            self.command = apply_control(self.command, frame)
        self._port.send(frame)
        return []

    def due_frames(self, now: float, state: MotorState) -> list[bytes]:
        self._port.flush()
        data = self._port.recv()
        if self.plug is not Plug.IN:
            if data and self.plug is Plug.OUT:
                self._plug_in(now)
            return []                # an old life's frames: the brain never sees them
        self._reader.push(data)
        frames = []
        while (frame := self._reader.next_frame()) is not None:
            frames.append(frame)
            self.link = _LINK_BY_FRAME.get(frame[0], self.link)
            if frame[0] == 0x76:
                self._fault_byte = frame[13]
        if frames or self._last_heard is None:
            self._last_heard = now
        elif now - self._last_heard > UNPLUG_S:
            self._unplug(now)
        return frames

    def drive(self, state: MotorState, now: float, dt: float) -> Drive:
        self._exchange(state, now)
        if self.plug is not Plug.IN or self.out is None:
            return None
        return bridge_drive(self.board, self.out, self.battery_v)

    def reboot(self, now: float) -> None:
        """The brain went away; the real motor keeps running and
        re-announces on its own after its link watchdog."""

    def close(self, keep_log: bool = False) -> None:
        _LIVE.discard(self)
        for ch in (self._port, self._plant):
            if ch is not None:
                ch.close()
        if self._proc.poll() is None:
            self._proc.terminate()   # QEMU quits cleanly on SIGTERM
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
        if self._own_dir and not keep_log:
            shutil.rmtree(self._dir, ignore_errors=True)

    # ---- unplugging -----------------------------------------------------

    def _unplug(self, now: float) -> None:
        self.plug = Plug.OUT
        self.link, self.command, self._fault_byte = LinkMode.INFO, Command(), 0
        print(f"{now:7.2f}s {self.name}: motor QEMU silent for {UNPLUG_S:g} s, "
              "unplugged", file=sys.stderr, flush=True)

    def _plug_in(self, now: float) -> None:
        """It talks again: plug it back in, which powers it up from reset.
        Until QEMU confirms the reset, all it says is from before."""
        self.plug = Plug.POWERING_UP
        self._plant.send(PLANT_RESET, force=True)
        print(f"{now:7.2f}s {self.name}: motor QEMU back, plugged in again "
              "(board reset)", file=sys.stderr, flush=True)

    def _powered_up(self, now: float) -> None:
        self.plug = Plug.IN
        self.out = None
        self._reader = protocol.FrameReader()
        self._last_heard = now

    # ---- plant link ---------------------------------------------------

    def _exchange(self, state: MotorState, now: float) -> None:
        """Send this step's analogue world, take the newest reply. The
        reply to this message arrives during the next step: one step of
        lag, so the physics never waits on QEMU."""
        pwm = self.out.sct_duty[PWM_OUTPUT] if self.out else 0.0
        per_rad = encoder_counts_per_rad(self.board, self.cartridge)
        self._plant.send(encode_plant_in(adc_samples(self.board, state, self.battery_v, pwm),
                                         state.position_rad * per_rad,
                                         state.velocity_rad_s * per_rad))
        self._plant_buf += self._plant.recv()
        n = PLANT_OUT.size
        while self._plant_buf:
            if self._plant_buf[:1] == PLANT_RESET:
                self._plant_buf = self._plant_buf[1:]
                if self.plug is Plug.POWERING_UP:
                    self._powered_up(now)
                continue
            if len(self._plant_buf) < n:
                break
            if self._plant_buf[:1] != b"O":
                self._plant_buf = self._plant_buf[1:]
                continue
            self.out = decode_plant_out(self._plant_buf[:n])
            self._plant_buf = self._plant_buf[n:]


def encoder_counts_per_rad(board: Board, cartridge: Cartridge) -> float:
    """Encoder counts per radian of the output shaft."""
    per_rev = board.counts_per_output_rev or COUNTS_PER_SHAFT_REV * cartridge.ratio
    return per_rev / (2 * math.pi)
