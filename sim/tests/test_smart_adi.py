"""SmartAdi against the reference frames in notes/adi-protocol.md section 7."""
import pytest

from v5sim import protocol
from v5sim.smart_adi import AdiSample, SmartAdi
from v5sim.smart_device import LinkMode


def hexframe(s: str) -> bytes:
    """Note syntax: hex bytes, `00*N` = N zero bytes."""
    out = []
    for tok in s.split():
        if "*" in tok:
            b, n = tok.split("*")
            out += [int(b, 16)] * int(n)
        else:
            out.append(int(tok, 16))
    return bytes(out)


CFG = "00 02 03 29 3E 42 66 6B"   # A analog, B dig in, C dig out, D PWM, E/F enc, G/H sonar


def linked(cfg: str | None = None) -> SmartAdi:
    adi = SmartAdi()
    adi.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"), 0.0)
    adi.on_brain_frame(hexframe("02 13 01 00 00 00 00 00 00 00 00 00 00 00 85 CC"), 0.0)
    adi.on_brain_frame(hexframe("04 00 0A 00 00*26 73 90"), 0.0)
    if cfg:
        adi.on_brain_frame(protocol.build(hexframe(f"50 {cfg} 00 00 01"), 32), 0.0)
    return adi


def test_info_and_ready_frames_match_reference():
    adi = SmartAdi()
    assert adi.info_frame() == hexframe("0A 0B 00 00 0C 00 00 40 00 20 00 00 00 00 42 1A")
    assert adi.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"),
                              0.0) == [hexframe("13 20 00 00 00 00 00 00 00 00 00 00 00 00 CF 56")]


def test_bring_up_switches_to_32_bytes_and_acks():
    adi = SmartAdi()
    adi.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"), 0.0)
    adi.on_brain_frame(hexframe("02 13 01 00 00 00 00 00 00 00 00 00 00 00 85 CC"), 0.0)
    assert adi.frame_len == 32 and adi.link == LinkMode.IDLE
    assert adi.due_frames(1.0, AdiSample()) == [hexframe("11 00*29 B2 AA")]
    assert adi.on_brain_frame(hexframe("03 16 64 00 00 00 00 0A 00*22 EA AD"), 1.0) == \
        [hexframe("02 03 00*28 39 70")]
    assert adi.on_brain_frame(hexframe("04 00 32 00 00*26 FC 17"), 1.0) == \
        [hexframe("02 04 00*28 CB 80")]
    assert adi.link == LinkMode.DATA and adi.interval_s == pytest.approx(0.050)
    assert adi.on_brain_frame(hexframe("04 00 0A 00 00*26 73 90"), 1.0) == \
        [hexframe("02 04 00*28 CB 80")]
    assert adi.interval_s == pytest.approx(0.010)


def test_ready_frame_reports_current_config():
    adi = linked(CFG)
    assert adi.ready_frame() == hexframe("13 20 00 02 03 29 3E 42 66 6B 00 00 00 00 F9 12")


def test_command_acks_match_reference():
    adi = linked()
    assert adi.on_brain_frame(hexframe("50 00 02 03 29 3E 42 66 6B 00 00 01 00*18 4E 9D"),
                              0.0) == [hexframe("02 50 00*28 09 AB")]
    assert adi.on_brain_frame(hexframe("51 00 00 01 B1 00*25 CF 72"), 0.0) == \
        [hexframe("02 51 00*28 F9 4D")]
    assert adi.on_brain_frame(hexframe("53 02 00 18 0C 00 01 02 03 04 05 06 07 08 09 0A 0B "
                                       "0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 F6 98"),
                              0.0) == [hexframe("02 53 00*28 08 A0")]


def test_idle_data_frame_reports_5v_supply():
    assert linked().data_frame(0, AdiSample()) == hexframe("73 00*27 56 0D 81 63")


def test_analog_and_digital_inputs():
    adi = linked("00 00 02 00 00 00 00 00")
    s = AdiSample(analog=(2048, 4095) + (0,) * 6,
                  digital=(False, False, True) + (False,) * 5)
    assert adi.data_frame(0, s) == hexframe("73 00 08 00 FF 0F 00 01 00*20 56 0D 7D DB")


def test_encoder_sonar_and_digital_layout():
    adi = linked("2E 32 56 5B 02 00 00 00")
    s = AdiSample(encoders=((12345, -200),) + ((0, 0),) * 3,
                  echo_us=(0xFFFF, 2900, 0xFFFF, 0xFFFF),
                  digital=(False,) * 4 + (True,) + (False,) * 3)
    assert adi.data_frame(0, s) == hexframe(
        "73 39 30 00 00 38 FF 54 0B 00 00 00 00 01 00*14 56 0D 5F B2")


def test_gyro_port_reports_device_angle_after_calibration():
    adi = linked("00 00 00 00 00 00 00 1C")
    assert adi.data_frame(0.5, AdiSample(heading_tenths=200))[22:24] == b"\x00\x80"
    adi.due_frames(1.1, AdiSample(heading_tenths=200))      # calibrated: zero here
    # the device's angle runs the other way to VEX's clockwise heading
    assert adi.data_frame(1.2, AdiSample(heading_tenths=200 - 900)) == \
        hexframe("73 00*21 84 03 00 00 00 00 56 0D AB 0D")


D1_CFG = "50 10 14 18 20 1C 29 25 03 00 80 01 00*18 77 02"
D2_ANALOG = (2048, 3000, 400, 2323, 0, 0, 0, 0)


def test_analog_and_gyro_reference_frames():
    adi = linked()
    adi.on_brain_frame(hexframe(D1_CFG), 0.0)
    s = AdiSample(analog=D2_ANALOG, heading_tenths=0)
    assert adi.data_frame(0.5, s) == hexframe(
        "73 00 08 00 B8 0B 00 90 01 00 13 09 00 00 80 00 7F 00 00 7F 00 00*7 56 0D 80 8E")
    adi.due_frames(1.1, s)
    assert adi.data_frame(1.2, AdiSample(analog=D2_ANALOG, heading_tenths=900)) == hexframe(
        "73 00 08 00 B8 0B 00 90 01 00 13 09 00 7C FC 00 7F 00 00 7F 00 00*7 56 0D 66 70")


def test_gyro_wraps_like_the_device_and_restarts():
    adi = linked("1C 00 00 00 00 00 00 00")
    adi.due_frames(1.1, AdiSample())

    def angle(f):
        return int.from_bytes(f[1:3], "little", signed=True)

    assert angle(adi.data_frame(1.2, AdiSample(heading_tenths=-4000))) == 400
    assert angle(adi.data_frame(1.2, AdiSample(heading_tenths=4000))) == -400
    # 0x51 byte 12 bit p restarts that gyro's calibration
    adi.on_brain_frame(protocol.build(bytes([0x51] + [0] * 11 + [0x01]), 32), 2.0)
    assert adi.data_frame(2.5, AdiSample(heading_tenths=4000))[1:3] == b"\x00\x80"
    adi.due_frames(3.1, AdiSample(heading_tenths=4000))
    assert angle(adi.data_frame(3.2, AdiSample(heading_tenths=4100))) == -100


def test_addressable_led_strip_decode():
    adi = linked("00 00 03 00 00 00 00 00")
    assert adi.on_brain_frame(hexframe("53 02 00 06 06 00 0F 0F 00 00 FF FF 00*18 78 29"),
                              0.0) == [adi.ack_frame(0x53)]
    strip = adi.led_strip(2)
    assert strip[:4] == ((255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255))
    assert strip[4] == (0, 0, 0) and len(strip) == 64


def test_addressable_led_chunks_and_start_offset():
    adi = linked("03 00 00 00 00 00 00 00")
    data = bytes([0xF0, 0x0F]) * 15                                # 20 LEDs
    adi.on_brain_frame(protocol.build(bytes([0x53, 0, 0, 24, 0, 0]) + data[:24], 32), 0.0)
    assert adi.led_strip(0)[0] == (0, 0, 0)                        # not committed yet
    adi.on_brain_frame(protocol.build(bytes([0x53, 0, 24, 6, 30, 10]) + data[24:], 32), 0.0)
    strip = adi.led_strip(0)
    assert strip[9] == (0, 0, 0) and strip[10] == (0, 255, 0) and strip[29] != (0, 0, 0)


def test_outputs_read_back_and_drive_the_world():
    adi = linked(CFG)
    assert adi.outputs[2] == 0 and adi.outputs[3] == 0x7F     # PWM neutral after 50
    adi.on_brain_frame(hexframe("51 00 00 01 B1 00*25 CF 72"), 0.0)
    assert adi.digital_out(2) is True and adi.pwm(3) == 50
    f = adi.data_frame(0, AdiSample())
    assert f[7] == 0x01 and f[10] == 0xB1                      # C and D slots
    adi.on_brain_frame(hexframe("51 00*29 D5 28"), 0.0)
    assert adi.digital_out(2) is False and adi.pwm(3) == -127


def test_outputs_ignored_on_input_ports():
    adi = linked()                                             # all analog in
    adi.on_brain_frame(hexframe("51 00 00 01 B1 00*25 CF 72"), 0.0)
    assert adi.outputs == bytes(8)


def test_encoder_counts_from_config_time():
    adi = linked()
    s = AdiSample(encoders=((1000, 0),) + ((0, 0),) * 3)
    adi.data_frame(0, s)
    adi.on_brain_frame(protocol.build(hexframe("50 2E 32 00 00 00 00 00 00 00 00 01"), 32), 0.0)
    f = adi.data_frame(0, AdiSample(encoders=((1250, 0),) + ((0, 0),) * 3))
    assert int.from_bytes(f[1:5], "little", signed=True) == 250


def test_sonar_echo_capped_like_the_device():
    adi = linked("4E 53 00 00 00 00 00 00")
    f = adi.data_frame(0, AdiSample(echo_us=(50000, 0xFFFF, 0xFFFF, 0xFFFF)))
    assert int.from_bytes(f[1:3], "little") == 0x8CA0


def test_encoder_zero_uses_the_live_count_not_the_last_frame():
    adi = linked()
    adi.data_frame(0, AdiSample(encoders=((1000, 0),) + ((0, 0),) * 3))
    adi.due_frames(0.001, AdiSample(encoders=((1100, 0),) + ((0, 0),) * 3))
    adi.on_brain_frame(protocol.build(hexframe("50 2E 32 00 00 00 00 00 00 00 00 01"), 32), 0.0)
    f = adi.data_frame(0, AdiSample(encoders=((1150, 0),) + ((0, 0),) * 3))
    assert int.from_bytes(f[1:5], "little", signed=True) == 50


def test_led_chunk_longer_than_its_frame_is_ignored():
    adi = linked("03 00 00 00 00 00 00 00")
    adi.on_brain_frame(protocol.build(bytes([0x53, 0, 0, 40, 40, 0]) + b"\xF0" * 24, 32), 0.0)
    assert adi.led_strip(0) == ((0, 0, 0),) * 64
