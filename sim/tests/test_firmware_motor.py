"""The emulated-firmware motor (firmware_motor.py, notes/motor-emulation.md).

The pure parts are checked against replicas of the firmware's own
conversions; the last tests boot DEVFRM02 / DEVFRM05 in QEMU (skipped when
the vex-motor QEMU or the images aren't there).
"""
import math
import struct
import time
from pathlib import Path

import pytest

from v5sim import motor, protocol
from v5sim.firmware_motor import (BOARD_5W5, BOARD_11W, PLANT_OUT, QEMU,
                                  FirmwareMotor, PlantOut, adc_samples,
                                  bridge_drive, current_counts,
                                  decode_plant_out, encode_plant_in,
                                  encoder_counts_per_rad, temperature_counts)
from v5sim.motor import Cartridge, MotorState
from v5sim.smart_motor import FIRMWARE_DIR


def plant_out(levels: int = 0, pwm_pin: int = 0xFF, duty: float = 0.0,
              outputs: int = 0xFFFFFFFF) -> PlantOut:
    pins = (0xFF, 0xFF, 0xFF, 0xFF, pwm_pin, 0xFF)
    return PlantOut(levels, outputs, pins, (0.0, 0.0, 0.0, 0.0, duty, 0.0), 0)


# ---- the firmware's lookups, as the disassembly reads ----------------------

def fw_temp_11w(adc: int) -> int:
    """DEVFRM02 motorTempFromAdc 0x25EC."""
    for idx, limit in enumerate(BOARD_11W.temp_table):
        if adc < limit:
            return (idx + 1) * 5
    return 24 + 0x65


def fw_temp_5w5(adc: int) -> int:
    """DEVFRM05 motorTempFromAdc 0x2604."""
    for idx, limit in enumerate(BOARD_5W5.temp_table):
        if limit < adc:
            return idx * 5
    return 24 + 0x65


def fw_current_11w(adc: int, pwm_high: float) -> float:
    """DEVFRM02 motorMonitorTick: mA from the current channel."""
    value = int(adc / 0.6552 + 0.5)
    if value < 25:
        return 0
    off = min(1.0, (1.0 - pwm_high) * 1.25)
    return value / off if pwm_high > 0 and off > 0.02 else value


# ---- pure ------------------------------------------------------------------

@pytest.mark.parametrize("board, lookup", [(BOARD_11W, fw_temp_11w),
                                           (BOARD_5W5, fw_temp_5w5)])
@pytest.mark.parametrize("temp_c", [5, 25, 40, 55, 70, 100])
def test_temperature_reads_back_through_the_firmware_table(board, lookup, temp_c):
    assert lookup(temperature_counts(board, temp_c)) == temp_c


@pytest.mark.parametrize("amps, pwm", [(0.5, 0.0), (1.0, 0.48), (1.8, 0.08), (2.4, 0.3)])
def test_11w_current_survives_the_off_time_correction(amps, pwm):
    got = fw_current_11w(current_counts(BOARD_11W, amps, pwm), pwm)
    assert got == pytest.approx(amps * 1000, rel=0.01)


def test_5w5_current_is_plain():
    assert current_counts(BOARD_5W5, 1.0, 0.4) == round(1000 * 1.016)


def test_adc_samples_put_each_signal_on_its_channel():
    st = MotorState(voltage_v=-6.0, current_a=1.0, temperature_c=25.0)
    adc = adc_samples(BOARD_11W, st, 12.8, 0.0)
    assert adc[BOARD_11W.ch_terminal_a] == round(6000 * 0.16234)
    assert adc[BOARD_11W.ch_terminal_b] == 0
    assert adc[BOARD_11W.ch_battery] == round(12800 * 0.16234)
    adc = adc_samples(BOARD_5W5, st, 12.8, 0.0)
    assert adc[BOARD_5W5.ch_battery] == round(12800 * 0.11833)
    assert adc[BOARD_5W5.ch_current] == round(1000 * 1.016)


@pytest.mark.parametrize("board", [BOARD_11W, BOARD_5W5])
def test_bridge_truth_table(board):
    en = 1 << board.pin_enable
    in1, in2 = 1 << board.pin_in1, 1 << board.pin_in2
    assert bridge_drive(board, plant_out(en), 12.0) is None                # 00 coast
    assert bridge_drive(board, plant_out(en | in1 | in2), 12.0) == 0.0     # 11 brake
    assert bridge_drive(board, plant_out(en | in2), 12.0) == 12.0          # 01 forward
    assert bridge_drive(board, plant_out(en | in1), 12.0) == -12.0         # 10 reverse
    assert bridge_drive(board, plant_out(in2), 12.0) is None               # driver off
    # PWM on IN1 with IN2 held: forward for the low part of the period
    pwm = plant_out(en | in2, pwm_pin=board.pin_in1, duty=0.25)
    assert bridge_drive(board, pwm, 12.0) == pytest.approx(9.0)


@pytest.mark.parametrize("board", [BOARD_11W, BOARD_5W5])
def test_undriven_pins_float_so_the_bridge_coasts(board):
    """Before the firmware sets its pins up they are inputs reading their
    pull-ups: that is a dead driver, not a brake."""
    assert bridge_drive(board, plant_out(0xFFFFFFFF, outputs=0), 12.0) is None
    # the PWM'd input is driven by the SCT whatever its GPIO direction
    en, in2 = 1 << board.pin_enable, 1 << board.pin_in2
    pwm = plant_out(en | in2, pwm_pin=board.pin_in1, duty=0.5, outputs=en | in2)
    assert bridge_drive(board, pwm, 12.0) == pytest.approx(6.0)


def test_plant_messages():
    msg = encode_plant_in(list(range(12)), 12.5, -3.0, 0x80000001)
    assert msg[:1] == b"I" and len(msg) == 1 + 24 + 16 + 4
    assert struct.unpack("<dd", msg[25:41]) == (12.5, -3.0)
    raw = PLANT_OUT.pack(b"O", 1 << 15, 1 << 15, 1, 2, 3, 4, 24, 0xFF,
                         0, 0, 0, 0, 0.5, 0, 99)
    out = decode_plant_out(raw)
    assert out.pin_fraction(24) == 0.5 and out.pin_fraction(15) == 1.0
    assert out.vm_ns == 99


def test_encoder_resolution():
    # 11 W: 50 counts per motor revolution; 5.5 W: 480 per output revolution
    assert encoder_counts_per_rad(BOARD_11W, Cartridge.GREEN_18) * 2 * math.pi == 900
    assert encoder_counts_per_rad(BOARD_5W5, Cartridge.MOTOR_5_5W_FW) * 2 * math.pi == 480


def test_5w5_firmware_plant_matches_vex_at_the_firmware_voltage_cap():
    cart, volts = Cartridge.MOTOR_5_5W_FW, 0.6 * 12.8
    free = 200 * 2 * math.pi / 60
    current = motor.motor_current(volts, free, cart, math.inf)
    assert motor.net_output_torque(current, free, cart) == pytest.approx(0, abs=2e-3)
    stall = (motor.output_torque(motor.motor_current(volts, 0.0, cart, math.inf), cart)
             - motor.gear_friction_nm(cart)[0])
    assert stall == pytest.approx(0.5, rel=0.02)


def test_channel_drops_whole_messages_when_qemu_stops_reading():
    import socket
    from v5sim.firmware_motor import _Channel
    a, b = socket.socketpair()
    a.setblocking(False)
    ch = _Channel(a, "plant", backlog=1000)
    sent = sum(ch.send(bytes(45)) for _ in range(20000))    # nobody reads
    assert 0 < sent < 20000                                 # never raises
    got = b""
    b.setblocking(False)
    while True:                                             # QEMU catches up
        try:
            got += b.recv(1 << 16)
        except BlockingIOError:
            if not ch._out:
                break
            ch.flush()
    assert len(got) == 45 * sent                            # whole messages only
    b.close()
    with pytest.raises(ConnectionError):
        ch.recv()
    a.close()


# ---- the real firmware in QEMU -----------------------------------------------

needs_qemu = pytest.mark.skipif(
    not Path(QEMU).is_file() or not (FIRMWARE_DIR / BOARD_11W.image).is_file(),
    reason="needs the vex-motor QEMU (scripts/qemu_install_vex.sh) and VEXos images")


def frame(*body: int) -> bytes:
    return protocol.build(bytes(body))


def run_brain(m: FirmwareMotor, script: dict[float, bytes], seconds: float):
    """Answer the motor like the brain (one frame per device frame) while a
    free shaft follows the drive, in real time. Yields (t, state, status)."""
    state, status, queue = MotorState(), None, []
    t0, step = time.monotonic(), 0
    while (now := step * 0.001) < seconds:
        queue += [f for t, f in list(script.items()) if now >= t]
        script = {t: f for t, f in script.items() if now < t}
        for f in m.due_frames(now, state):
            reply = {0x0A: frame(0x03, 1, 100, 0, 0, 0, 0, 0x0A),
                     0x13: frame(0x04, 0, 5, 0)}.get(f[0])
            if reply is None:
                reply = queue.pop(0) if queue else frame(0x02, f[0], 1, 1, 0xFF)
            if f[0] == 0x76:
                status = f
            m.on_brain_frame(reply, now)
        drive = m.drive(state, now, 0.001)
        state = motor.step_shaft(state, drive, 0.001, m.cartridge, limit_a=math.inf)
        yield now, state, status
        step += 1
        time.sleep(max(0.0, t0 + now - time.monotonic()))


def velocity_cmd(counts_s: int) -> bytes:
    return protocol.build(b"\x56\x02" + struct.pack("<h", counts_s) + bytes(6)
                          + bytes([1, 0, 1, 0]))


def voltage_cmd(percent: int) -> bytes:
    return protocol.build(b"\x56\x03" + bytes(6) + struct.pack("<b", percent)
                          + bytes([0, 8, 0, 1, 0]))


@needs_qemu
@pytest.mark.parametrize("cart", [Cartridge.GREEN_18, Cartridge.MOTOR_5_5W])
def test_real_firmware_runs_the_motor(cart):
    m = FirmwareMotor(cart, name="pytest")
    try:
        samples, speeds, last = {}, [], None
        for now, state, status in run_brain(
                m, {0.2: voltage_cmd(50), 1.2: velocity_cmd(1500)}, 2.4):
            for t in (1.1, 2.3):
                if t not in samples and now >= t:
                    samples[t] = (state, status)
            if now >= 2.0 and status is not last and status is not None:
                speeds.append(struct.unpack("<h", status[8:10])[0])
            last = status
    finally:
        m.close()
    state, status = samples[1.1]
    assert status is not None, "no 0x76 status: handshake failed"
    assert 5.6 < state.voltage_v < 6.6                  # 50 % of ~12 V
    millivolts = struct.unpack("<h", status[10:12])[0]
    assert millivolts == pytest.approx(state.voltage_v * 1000, rel=0.05)
    state, status = samples[2.3]
    # 1500 counts/s in green units is 100 rpm; the firmware's own PID holds it
    assert state.velocity_rpm == pytest.approx(100, abs=4)
    # single readings jitter a little with host timing (motor-emulation.md)
    assert sum(speeds) / len(speeds) == pytest.approx(1500, abs=30)


@needs_qemu
def test_a_stalled_board_is_unplugged_and_comes_back_from_reset():
    """QEMU stops (SIGSTOP) while the motor drives: past UNPLUG_S the motor
    coasts and the brain hears nothing; when QEMU runs again the board is
    power-cycled, announces itself and links up again."""
    import os
    import signal
    from v5sim.firmware_motor import UNPLUG_S, Plug
    from v5sim.smart_motor import LinkMode
    m = FirmwareMotor(Cartridge.GREEN_18, name="pytest-unplug")
    stop, cont = 0.8, 0.8 + UNPLUG_S + 0.5
    seen = {}
    try:
        for now, state, status in run_brain(
                m, {0.2: voltage_cmd(50), cont + 0.4: voltage_cmd(50)}, cont + 1.2):
            if now >= stop and "stop" not in seen:
                seen["stop"] = state
                os.kill(m._proc.pid, signal.SIGSTOP)
            if now >= cont - 0.05 and "out" not in seen:
                seen["out"] = (m.plug, state, m.link)
            if now >= cont and "cont" not in seen:
                seen["cont"] = True
                os.kill(m._proc.pid, signal.SIGCONT)
            if now >= cont + 0.3 and "back" not in seen:
                seen["back"] = (m.plug, m.link)
        end = (state, m.link)
    finally:
        os.kill(m._proc.pid, signal.SIGCONT)
        m.close()
    assert seen["stop"].velocity_rpm > 80               # driving when it stalls
    plug, state, link = seen["out"]
    assert plug is Plug.OUT and link.name == "INFO"
    assert state.voltage_v == 0 and state.velocity_rpm < 5      # coasted down
    assert seen["back"] == (Plug.IN, LinkMode.DATA)     # rebooted, linked again
    state, link = end
    assert link is LinkMode.DATA and state.velocity_rpm > 80


@needs_qemu
def test_robot_relays_the_brain_to_the_real_firmware():
    """robot.py with --motor-fw: brain frames reach the firmware through the
    robot's port, and the firmware's drive moves the robot's free shaft."""
    import socket
    from v5sim.robot import Port, Robot

    brain, dev = socket.socketpair()
    m = FirmwareMotor(Cartridge.GREEN_18, name="pytest-robot")
    robot = Robot([Port(7, dev, protocol.FrameReader(), m)], None)
    brain.setblocking(False)
    reader, sent_cmd, statuses = protocol.FrameReader(), False, 0
    t0, step = time.monotonic(), 0
    try:
        while (now := step * 0.001) < 1.5:
            robot.service_io(now, 0.0)
            robot.step(now)
            try:
                reader.push(brain.recv(4096))
            except BlockingIOError:
                pass
            while (f := reader.next_frame()) is not None:
                statuses += f[0] == 0x76
                if f[0] == 0x0A:
                    reply = frame(0x03, 7, 100, 0, 0, 0, 0, 0x0A)
                elif f[0] == 0x13:
                    reply = frame(0x04, 0, 5, 0)
                elif f[0] == 0x76 and not sent_cmd and now > 0.2:
                    reply, sent_cmd = voltage_cmd(50), True
                else:
                    reply = frame(0x02, f[0], 1, 7, 0xFF)
                brain.sendall(reply)
            step += 1
            time.sleep(max(0.0, t0 + now - time.monotonic()))
        _, state = robot.port_states()[0]
        assert statuses > 100 and m.link.name == "DATA"
        assert state.velocity_rpm == pytest.approx(98, abs=6)
        assert "p7[DATA]" in robot.describe()
        (snap,) = robot.device_snapshot(1.5)          # the dashboard's view
        assert snap["mode"] == "voltage" and snap["percent"] == 50
        assert snap["faults"] == 0
    finally:
        m.close()
        brain.close()
        dev.close()
