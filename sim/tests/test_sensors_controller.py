import math
import struct

import pytest

from v5sim import protocol
from v5sim.smart_controller import (Competition, ControllerState,
                                    SmartController, Stage)
from v5sim.smart_device import LinkMode
from v5sim.smart_sensors import (DistanceSample, RotationSample, SmartDistance,
                                 SmartRotation, ray_to_square_wall)


def hexframe(s: str) -> bytes:
    return bytes.fromhex(s.replace("_", ""))


def brain(payload: str, length: int = 16) -> bytes:
    return protocol.build(bytes.fromhex(payload), length)


def link(dev, now=0.0, interval="14 00"):
    dev.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"), now)
    dev.on_brain_frame(hexframe("02 13 01 00 00 00 00 00 00 00 00 00 00 00 85 CC"), now)
    return dev.on_brain_frame(brain("04 00 " + interval, dev.frame_len), now)


# ---- rotation ---------------------------------------------------------

def test_rotation_link_frames_match_real_sensor():
    r = SmartRotation()
    assert r.info_frame() == hexframe("0A 08 00 00 04 00 00 40 00 20 00 00 00 00 C7 F4")
    assert r.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"),
                            0.0) == [hexframe("13 10 00 00 00 00 00 00 00 00 00 00 00 00 F4 8A")]
    r.on_brain_frame(hexframe("02 13 01 00 00 00 00 00 00 00 00 00 00 00 85 CC"), 0.0)
    assert r.frame_len == 16
    ack = r.on_brain_frame(hexframe("04 00 14 00 00 00 00 00 00 00 00 00 00 00 6E 4B"), 0.1)
    assert ack == [hexframe("02 04 00 00 00 00 00 00 00 00 00 00 00 00 5A 05")]
    assert r.link == LinkMode.DATA and r.interval_s == pytest.approx(0.020)


def test_rotation_status_frame_layout_matches_example():
    r = SmartRotation()                         # note's "AA idle" example
    r.position_cdeg, r._angle_cdeg = 12345, 12345
    assert r.data_frame(0, RotationSample()) == hexframe(
        "AA 19 00 00 39 30 00 00 00 00 00 00 39 30 D3 36")
    r.position_cdeg, r._angle_cdeg, r.velocity = -4500, 31500, -120
    f = r.data_frame(0, RotationSample())       # "AA reverse" body at 25 C
    assert f[2:14] == hexframe("00 00 6C EE FF FF 88 FF 00 00 0C 7B")


def test_rotation_position_wraps_like_an_s32():
    r = SmartRotation()
    r.position_cdeg = 2**31 + 5                 # ~60k turns of a long run
    f = r.data_frame(0, RotationSample())
    assert struct.unpack_from("<i", f, 4)[0] == -2**31 + 5


def test_rotation_tracks_position_across_wraps_and_velocity():
    r = SmartRotation()
    t = 0.0
    for i in range(1001):                      # 2.5 turns in 1 s
        r.sample(t, RotationSample(shaft_deg=900 * t))
        t += 0.001
    assert r.position_cdeg == pytest.approx(90000, abs=20)
    assert r.velocity == pytest.approx(900, abs=5)


def test_rotation_commands_zero_offset_and_reset_position():
    r = SmartRotation()
    r.sample(0.0, RotationSample(shaft_deg=100.0))
    acks = r.on_brain_frame(hexframe("50 00 FF FF 00 00 00 00 00 00 00 00 00 00 C0 DB"), 0.0)
    assert acks == [hexframe("02 50 00 00 00 00 00 00 00 00 00 00 00 00 13 CC")]
    r.sample(0.01, RotationSample(shaft_deg=130.0))
    angle = struct.unpack_from("<H", r.data_frame(0.01, RotationSample()), 12)[0]
    assert angle == pytest.approx(3000, abs=10)
    r.on_brain_frame(hexframe("50 00 10 00 00 00 00 00 00 00 00 00 00 00 73 72"), 0.02)
    assert r.position_cdeg == r._angle_cdeg


# ---- distance ---------------------------------------------------------

def test_distance_switches_to_32_byte_frames():
    d = SmartDistance()
    assert d.info_frame() == hexframe("0A 02 00 00 07 00 00 40 00 20 00 00 00 00 F7 A7")
    r = d.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"), 0.0)
    assert r == [hexframe("13 20 00 00 00 00 00 00 00 00 00 00 00 00 CF 56")]
    d.on_brain_frame(hexframe("02 13 01 00 00 00 00 00 00 00 00 00 00 00 85 CC"), 0.0)
    assert d.frame_len == 32
    ack = d.on_brain_frame(brain("04 00 14 00", 32), 0.1)[0]
    assert ack == protocol.build(b"\x02\x04", 32) and ack[-2:] == b"\xcb\x80"


def test_distance_no_object_frame_matches_example():
    d = SmartDistance()
    link(d)
    assert d.data_frame(0, DistanceSample()) == hexframe(
        "94 00 00 00 00 02" + " 00" * 24 + " DB A5")


def test_distance_reports_mm_and_sdk_object_size():
    d = SmartDistance()
    link(d)
    f = d.data_frame(0, DistanceSample(distance_mm=500, object_mm=80))
    mm, conf = struct.unpack_from("<HB", f, 2)
    a, b = struct.unpack_from("<II", f, 8)
    assert (mm, conf) == (500, 63)
    assert 0.32 * mm / math.sqrt(a / b) == pytest.approx(80, abs=0.5)


def test_ray_to_square_wall():
    assert ray_to_square_wall(0, 0, 0, 1.8) == pytest.approx(1.8)
    assert ray_to_square_wall(1.0, 0, math.pi, 1.8) == pytest.approx(2.8)
    assert ray_to_square_wall(0, 0, math.pi / 4, 1.0) == pytest.approx(math.sqrt(2))


# ---- controller --------------------------------------------------------

def test_controller_bring_up_and_neutral_frame():
    c = SmartController()
    assert c.due_frames(0.0, ControllerState()) == [
        hexframe("0A 4B 00 00 09 00 00 40 78 56 34 12 00 00 89 7C")]
    r = c.on_brain_frame(hexframe("03 01 28 00 00 00 00 00 00 00 00 00 00 00 14 A2"), 0.0)
    assert r == [hexframe("13 20 00 00 78 56 34 12 00 00 00 00 00 00 12 1A")]
    c.on_brain_frame(hexframe("02 13 01 01 00 00 00 00 00 00 00 00 00 00 5D 85"), 0.01)
    assert c.stage == Stage.STREAM
    assert c.due_frames(0.012, ControllerState()) == []        # +5 ms first
    assert c.due_frames(0.016, ControllerState()) == [hexframe(
        "A0 00 01 00 00 00 00 00 00 00 64 BE 00 00 00 00 7F 7F 7F 7F 00 00"
        " 7F 7F 7F 7F 00 00 00 00 8A E3")]


@pytest.mark.parametrize("state, expected", [
    (ControllerState(axes={"Axis3": 100}, buttons=frozenset({"A"})),
     "A0 00 01 00 00 00 00 00 00 00 64 BE 00 00 00 00 1B 7F 7F 7F 00 04"
     " 7F 7F 7F 7F 00 00 00 00 5D D9"),
    (ControllerState(axes={"Axis1": 127}, buttons=frozenset({"L1", "R1"})),
     "A0 00 01 00 00 00 00 00 00 00 64 BE 00 00 00 00 7F 7F 7F 00 10 20"
     " 7F 7F 7F 7F 00 00 00 00 89 28"),
    (ControllerState(competition=Competition.DISABLED),
     "A0 00 01 00 00 00 09 00 00 00 64 BE 00 00 00 00 7F 7F 7F 7F 00 00"
     " 7F 7F 7F 7F 00 00 00 00 15 D9"),
    (ControllerState(competition=Competition.AUTONOMOUS),
     "A0 00 01 00 00 00 11 00 00 00 64 BE 00 00 00 00 7F 7F 7F 7F 00 00"
     " 7F 7F 7F 7F 00 00 00 00 FD B3"),
])
def test_controller_frames_match_reference(state, expected):
    assert SmartController().data_frame(state) == hexframe(expected)


def test_controller_input_parsing_and_validation():
    from v5sim.robot import parse_controller_input
    s = parse_controller_input({"axes": {"Axis3": 254, "Axis1": -20.4},
                                "buttons": ["A", "L1"],
                                "competition": "AUTONOMOUS"}, ControllerState())
    assert s.axes == {"Axis3": 254, "Axis1": -20}
    assert s.buttons == {"A", "L1"} and s.competition == Competition.AUTONOMOUS
    f = SmartController().data_frame(s)
    assert f[0x10] == 0x7F - 127                 # clamped to 127 on the wire
    with pytest.raises(ValueError):
        parse_controller_input({"buttons": ["Turbo"]}, ControllerState())
    with pytest.raises(KeyError):
        parse_controller_input({"competition": "PAUSED"}, ControllerState())


@pytest.mark.parametrize("msg", [
    {"axes": {"Axis3": 1e999}},          # inf -> round() OverflowError
    {"axes": {"Axis3": float("nan")}},
    {"axes": [1]},                       # not a dict -> AttributeError
    {"axes": 5},
    {"buttons": "AB"},                   # string iterated as buttons
])
def test_controller_input_rejects_malformed(msg):
    from v5sim.robot import parse_controller_input
    with pytest.raises(ValueError):
        parse_controller_input(msg, ControllerState())


# ---- controller screen (notes/controller-screen.md section 5) ---------

SCREEN_FRAMES = {
    "hello": "54 0F 00 00 AA 5A 56 0B 33 01 00 05 48 65 6C 6C 6F E3 DC 00 00 00 00 00 00 00 00 00 00 00 FD 6C",
    "clear": "54 0A 00 01 AA 5A 56 06 33 0F 00 00 A5 B8 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 EB 17",
    "clear2": "54 0A 00 02 AA 5A 56 06 33 02 00 00 E7 E9 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D4 BD",
    "rumble": "54 0C 00 03 AA 5A 56 08 33 04 00 02 2E 2D A4 69 00 00 00 00 00 00 00 00 00 00 00 00 00 00 C3 CD",
    "battery": "54 15 00 04 AA 5A 56 11 33 03 02 0B 42 61 74 74 65 72 79 20 38 37 25 2A E5 00 00 00 00 00 51 41",
    "abc1": "54 1A 00 05 AA 5A 56 19 33 02 00 13 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F 50 51 52 E7 ED",
    "abc2": "54 03 00 06 53 52 BA 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 53 29",
}


def streaming_controller() -> SmartController:
    c = SmartController()
    c.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"), 0.0)
    c.on_brain_frame(hexframe("02 13 01 00 00 00 00 00 00 00 00 00 00 00 85 CC"), 0.0)
    return c


def feed(c: SmartController, *names: str) -> None:
    for n in names:
        c.on_brain_frame(hexframe(SCREEN_FRAMES[n]), 1.0)


def test_controller_screen_text_clear_and_rumble():
    c = streaming_controller()
    feed(c, "hello", "battery")
    assert c.screen_lines() == ["Hello", "", "  Battery 87%"]
    feed(c, "abc1")
    assert c.screen_lines()[1] == ""                 # half a frame: not yet
    feed(c, "abc2")
    assert c.screen_lines()[1] == "ABCDEFGHIJKLMNOPQRS"
    feed(c, "clear2")
    assert c.screen_lines() == ["Hello", "", "  Battery 87%"]
    feed(c, "rumble")
    assert c.rumble == ".-"
    feed(c, "clear")
    assert c.screen_lines() == ["", "", ""]


def test_controller_screen_rejects_bad_inner_crc_and_partner():
    c = streaming_controller()
    bad = bytearray(hexframe(SCREEN_FRAMES["hello"])[:30])
    bad[17] ^= 0xFF                                   # inner CRC byte
    c.on_brain_frame(protocol.build(bytes(bad), 32), 1.0)
    inner = bytes.fromhex("AA 5B 56 0B 33 01 00 05") + b"Hello"   # partner (0x5B)
    inner += protocol.crc16_ccitt_false(inner, init=0).to_bytes(2, "big")
    c.on_brain_frame(protocol.build(bytes([0x54, len(inner), 0, 0]) + inner, 32), 1.0)
    assert c.screen_lines() == ["", "", ""]


def test_sensor_and_controller_reannounce_after_brain_silence():
    r = SmartRotation()
    r.on_brain_frame(hexframe("03 00 64 00 00 00 00 0A 00 00 00 00 00 00 69 B3"), 0.0)
    r.on_brain_frame(hexframe("04 00 14 00 00 00 00 00 00 00 00 00 00 00 6E 4B"), 0.0)
    assert r.link == LinkMode.DATA
    assert r.due_frames(2.6, RotationSample()) == [r.info_frame()]
    assert r.link == LinkMode.INFO and r.frame_len == 16
    c = streaming_controller()
    assert c.stage.name == "STREAM"
    assert c.due_frames(2.6, ControllerState()) == [c.info_frame()]
    assert c.stage.name == "ANNOUNCE" and c.frame_len == 16


def test_controller_screen_resyncs_after_a_false_start():
    c = streaming_controller()
    junk = bytes.fromhex("AA 00 00 F0 01 02 03")              # stray AA, huge len
    hello = hexframe(SCREEN_FRAMES["hello"])[4:4 + 15]
    c.on_brain_frame(protocol.build(bytes([0x54, len(junk + hello), 0, 0]) + junk + hello, 32), 1.0)
    assert c.screen_lines()[0] == "Hello"
