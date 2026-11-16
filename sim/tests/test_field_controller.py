"""A robot's controller tethered to a field-control brain
(notes/field-control-protocol.md): 0xA7 status out, 0x53 field state in."""
import argparse

import pytest

from v5sim import field_link, protocol, robot
from v5sim.radio_signal import RadioSignal, SignalSample
from v5sim.smart_controller import (ControllerState, RobotStatus, SmartController,
                                    Stage, ctrl_radio_reply)
from v5sim.smart_device import LinkMode
from v5sim.smart_radio import SmartRadio


def linked(c: SmartController, drain: bool = True) -> SmartController:
    """Run the 0A / 03 / 13 / 02 13 bring-up to the streaming stage; with
    drain, also take a field controller's unprompted Ctrl radio reply."""
    c.due_frames(0.0, ControllerState())
    c.on_brain_frame(protocol.build(bytes([0x03, 15, 0x64])), 0.001)
    c.on_brain_frame(protocol.build(bytes([0x02, 0x13, 1])), 0.002)
    assert c.stage == Stage.STREAM
    if drain and c.field_team is not None:
        assert c.due_frames(0.05, ControllerState()) == [ctrl_radio_reply()]
    return c


def test_field_status_frame_layout() -> None:
    """notes 9's linked A7 plus section 12's radio fields: RSSI -55 dBm
    (byte 1), Radio Status 0005 (4..5), channel 81 / mode 3 = "81/4 Comp"
    (16..17), Signal 100 % (29)."""
    c = SmartController(field_team="1234A")
    assert c.field_status_frame(SignalSample()).hex(" ") == (
        "a7 c9 01 00 05 00 c9 00 64 64 00 01 00 00 01 01 51 03 "
        "31 32 33 34 41 00 00 00 00 00 00 64 1d f0")


def test_signal_stays_in_a_field_like_range_and_repeats() -> None:
    a, b = RadioSignal(seed=1), RadioSignal(seed=1)
    samples = [a.update(t * 0.5) for t in range(400)]
    assert samples == [b.update(t * 0.5) for t in range(400)]
    rssi = [s.rssi_dbm for s in samples]
    assert -75 <= min(rssi) and max(rssi) <= -35 and len(set(rssi)) > 5
    assert all(90 <= s.quality <= 100 for s in samples)
    assert sum(s.quality < 100 for s in samples) > 0


def test_signal_updates_at_most_every_half_second() -> None:
    s = RadioSignal(seed=3)
    first = s.update(0.0)
    assert s.update(0.2) is first and s.update(0.6) is not first


def test_unlinked_robot_reports_no_radio_signal() -> None:
    c = SmartController(field_team="T")
    c.robot = RobotStatus(linked=False)
    f = c.field_status_frame(SignalSample())
    assert (f[1], f[4], f[29]) == (0, 0, 0)


def test_bridge_shares_one_signal_between_both_ends() -> None:
    sig = RadioSignal(seed=9)
    c, radio = SmartController(field_team="T", signal=sig), SmartRadio(signal=sig)
    radio.data_frame(1.0, field_link.radio_sample(c))
    link = radio.link_frame(True)
    a7 = c.field_status_frame()
    assert link[3] == a7[1] and link[8] == a7[29]


def test_ctrl_radio_reply_matches_spec_example() -> None:
    assert ctrl_radio_reply().hex(" ") == (
        "a3 17 00 00 aa 55 58 13 39 76 00 00 00 00 00 00 00 00 00 00 "
        "00 00 40 00 30 5e b5 00 00 00 c0 de")


def test_ctrl_radio_query_is_answered_once_before_status() -> None:
    c = linked(SmartController(field_team="1234A"))
    query = bytes.fromhex("c9 36 b8 50 58 39 00 ee fe")
    c.on_brain_frame(protocol.build(bytes([0x54, len(query), 0, 1]) + query, 32), 0.01)
    first, = c.due_frames(0.1, ControllerState())
    second, = c.due_frames(0.2, ControllerState())
    assert first == ctrl_radio_reply() and second[0] == 0xA7


def test_every_user_cdc_request_is_relayed_to_the_robot() -> None:
    c = linked(SmartController(field_team="T"))
    reqs = bytes.fromhex("c9 36 b8 50 56 22 00 2a 76")
    sync = bytes.fromhex("c9 36 b8 50 56 f6 04 0c 17 1a 22 7f 07")
    for seq, req in enumerate((reqs, sync)):
        c.on_brain_frame(protocol.build(bytes([0x54, len(req), 0, seq]) + req, 32), 0.01)
    assert c.robot_requests == [reqs, sync]


def test_ctrl_radio_is_sent_unprompted_on_each_link() -> None:
    c = linked(SmartController(field_team="T"), drain=False)
    first, = c.due_frames(0.1, ControllerState())
    assert first == ctrl_radio_reply()
    c.reboot(1.0)
    c = linked(c, drain=False)
    again, = c.due_frames(1.1, ControllerState())
    assert again == ctrl_radio_reply()


def test_relayed_reply_is_split_into_a3_frames() -> None:
    c = linked(SmartController(field_team="T"))
    reply = bytes(range(40))
    c.relay_reply(reply)
    a, = c.due_frames(0.1, ControllerState())
    b, = c.due_frames(0.2, ControllerState())
    assert a[1] == 26 and b[1] == 14 and a[4:30] + b[4:18] == reply


def test_relay_rewrites_header_and_crc_for_the_robot() -> None:
    req = bytes.fromhex("c9 36 b8 50 56 22 00 2a 76")
    assert field_link.to_robot(req).hex(" ") == "c9 36 b8 47 56 22 00 60 fc"


def test_take_reply_cuts_one_whole_cdc_reply() -> None:
    buf = bytearray(bytes.fromhex("00 aa 55 56 04 f6 ff 78 ba aa 55"))
    assert field_link.take_reply(buf) == bytes.fromhex("aa 55 56 04 f6 ff 78 ba")
    assert field_link.take_reply(buf) is None and buf == bytearray(b"\xaa\x55")


def test_ctrl_radio_is_the_bundled_radio_firmware() -> None:
    assert SmartRadio.FW_VERSION == 0x30                 # DEVFRM08 header
    assert ctrl_radio_reply()[24] == 0x30


def test_robot_status_reaches_a7() -> None:
    c = SmartController(field_team="T")
    c.robot = RobotStatus(linked=True, battery=55, flags=0x21, program=3)
    f = c.field_status_frame()
    assert (f[2], f[8], f[11], f[14]) == (1, 55, 0x21, 3)
    c.robot = RobotStatus(linked=False)
    f = c.field_status_frame()
    assert (f[2], f[8], f[11], f[14]) == (0, 0, 0, 0)


@pytest.mark.parametrize("state, byte6", [(0xC9, 0xC9), (0xD1, 0xD1), (0xE1, 0xE1)])
def test_field_state_passes_through_to_radio(state: int, byte6: int) -> None:
    c = SmartController(field_team="T")
    c.field_state = state
    assert field_link.radio_sample(c).status == byte6


def test_robot_status_from_radio_52() -> None:
    radio = SmartRadio()
    assert not field_link.radio_robot_status(radio).linked
    radio.link = LinkMode.DATA
    radio.on_command(bytes([0x52, 0, 77, 0, 0x41, 0, 0, 0, 0, 0, 0, 2]) + bytes(18), 0.0)
    assert field_link.radio_robot_status(radio) == RobotStatus(
        linked=True, battery=77, flags=0x41, program=2)


def test_field_controller_streams_a7_and_tracks_field_state() -> None:
    c = linked(SmartController(field_team="1234A"))
    (frame,) = c.due_frames(0.1, ControllerState())
    assert frame[0] == 0xA7 and len(frame) == 32 and protocol.is_valid(frame)
    assert frame[6] == 0xC9                       # disabled until told

    driver = protocol.build(bytes([0x53, 0xE1, 0, 0, 0, 0, 0, 0, 1]), 32)
    assert c.on_brain_frame(driver, 0.11) == []
    assert c.field_state == 0xE1
    (frame,) = c.due_frames(0.2, ControllerState())
    assert frame[6] == 0xE1


def test_plain_controller_ignores_field_state_and_sends_a0() -> None:
    c = linked(SmartController())
    c.on_brain_frame(protocol.build(bytes([0x53, 0xE1]), 32), 0.01)
    assert c.field_state == 0xC9
    (frame,) = c.due_frames(0.1, ControllerState())
    assert frame[0] == 0xA0


def test_team_name_is_truncated_to_ten_chars() -> None:
    frame = SmartController(field_team="ABCDEFGHIJKL").field_status_frame()
    assert frame[18:28] == b"ABCDEFGHIJ" and frame[28] == 0


@pytest.mark.parametrize("text, expected", [("15", (15, "SIM15")),
                                            ("1:1234A", (1, "1234A"))])
def test_field_controller_arg(text: str, expected: tuple[int, str]) -> None:
    assert robot._field_controller_arg(text) == expected


@pytest.mark.parametrize("text", ["0", "23", "5:ELEVENCHARS", "x:1"])
def test_field_controller_arg_rejects(text: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        robot._field_controller_arg(text)


def test_no_builtin_adi_flag_parses() -> None:
    args = robot.parser().parse_args(["--field-controller", "15", "--no-builtin-adi"])
    assert args.no_builtin_adi and args.field_controller == [(15, "SIM15")]
