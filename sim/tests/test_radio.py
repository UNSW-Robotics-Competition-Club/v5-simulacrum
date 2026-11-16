"""SmartRadio (VEXnet robot radio, type 8) against notes/radio-protocol.md
section 6."""
import pytest

from v5sim import protocol
from v5sim.smart_controller import Competition, ControllerState
from v5sim.smart_device import LinkMode
from v5sim.smart_radio import RadioSample, SmartRadio, status_byte


def hexframe(s: str) -> bytes:
    return bytes(int(t, 16) for t in s.split())


def linked() -> SmartRadio:
    r = SmartRadio(serial=0x12345678)
    r.on_brain_frame(hexframe("03 01 64 00 00 00 00 0A 45 23 01 00 00 00 95 13"), 0.0)
    r.on_brain_frame(hexframe("02 13 01 01 00 00 00 00 00 00 01 00 00 00 2B 31"), 0.0)
    r.on_brain_frame(hexframe("04 00 14 00" + " 00" * 26 + " F5 6B"), 0.0)
    assert r.link == LinkMode.DATA and r.frame_len == 32
    return r


IDLE_85 = ("85 00 00 00 05 00 00 00 00 00 64 BE 00 00 00 01 7F 7F 7F 7F 00 00 "
           "7F 7F 7F 7F 00 00 00 00 23 8E")
LINKED_80 = ("80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 40 01 30 00 00 00 00 "
             "05 00 00 00 00 00 00 00 C7 4B")


def test_bring_up_frames():
    r = SmartRadio(serial=0x12345678)
    assert r.info_frame() == hexframe("0A 30 00 00 08 00 00 40 78 56 34 12 01 00 70 FB")
    assert r.on_brain_frame(hexframe("03 01 64 00 00 00 00 0A 45 23 01 00 00 00 95 13"),
                            0.0) == [hexframe("13 20 78 56 34 12 00 00 00 00 00 00 00 00 25 65")]
    r.on_brain_frame(hexframe("02 13 01 01 00 00 00 00 00 00 01 00 00 00 2B 31"), 0.0)
    (keepalive,) = r.due_frames(1.0, RadioSample())
    assert keepalive == hexframe("11 01" + " 00" * 28 + " 42 4C")
    assert r.on_brain_frame(hexframe("04 00 14 00" + " 00" * 26 + " F5 6B"), 1.0) == \
        [hexframe("02 04" + " 00" * 28 + " CB 80")]
    assert r.interval_s == pytest.approx(0.020)


def test_rotation_link_status_then_inputs_and_peer_info():
    r = linked()
    kinds = [r.data_frame(0.02 * i, RadioSample())[0] for i in range(50)]
    assert kinds[:5] == [0x80, 0x85, 0x85, 0x85, 0x85]
    assert kinds.count(0x86) == 2 and kinds.count(0x80) == 10
    r = linked()
    # notes section 6's 80 frame, plus the link's RSSI (byte 3) and quality
    # (bytes 8..11) from the signal model
    sig = r.signal.update(0.0)
    want = bytearray(hexframe(LINKED_80)[:-2])
    want[3], want[8:12] = sig.rssi_dbm & 0xFF, sig.quality.to_bytes(4, "little")
    assert r.data_frame(0.0, RadioSample()) == protocol.build(bytes(want), 32)
    assert r.data_frame(0.02, RadioSample()) == hexframe(IDLE_85)


def test_inputs_frame_matches_reference():
    r = linked()
    r.data_frame(0.0, RadioSample())
    s = RadioSample(ControllerState({"Axis3": 100, "Axis1": 127}, frozenset({"A", "L1"})))
    assert r.data_frame(0.02, s) == hexframe(
        "85 00 00 00 05 00 00 00 00 00 64 BE 00 00 00 01 1B 7F 7F 00 00 24 "
        "7F 7F 7F 7F 00 00 00 00 63 E9")


@pytest.mark.parametrize("status, secs, crc", [(0x21, 42, "26 EA"), (0x91, 7, "5D 14"),
                                               (0x99, 15, "ED 16"), (0xA1, 42, "04 30")])
def test_competition_byte_and_match_clock(status, secs, crc):
    r = linked()
    r.data_frame(0.0, RadioSample())
    body = (f"85 00 00 00 05 00 {status:02X} {secs:02X} 00 00 64 BE 00 00 00 01 "
            f"7F 7F 7F 7F 00 00 7F 7F 7F 7F 00 00 00 00 {crc}")
    assert r.data_frame(0.02, RadioSample(status=status, match_secs=secs)) == hexframe(body)


def test_partner_controller():
    r = linked()
    r.data_frame(0.0, RadioSample())
    s = RadioSample(partner=ControllerState({"Axis4": 50}))
    assert r.data_frame(0.02, s) == hexframe(
        "85 00 04 00 05 00 00 00 00 00 64 BE 5A B4 00 01 7F 7F 7F 7F 00 00 "
        "7F 4D 7F 7F 00 00 00 00 11 7F")


def test_peer_info_frame():
    assert linked().peer_info_frame() == hexframe(
        "86 00 00 00 00 00 03 07 00 00 00 00 00 40 4B" + " 00" * 15 + " AC F2")


def test_unlinked_radio_searches_and_sends_no_inputs():
    r = linked()
    frames = [r.data_frame(0.02 * i, RadioSample(linked=False)) for i in range(6)]
    assert all(f[0] == 0x80 for f in frames)
    assert frames[0] == hexframe("80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 40 01 30 "
                                 "00 00 00 00 02 00 00 00 00 00 00 00 00 53")


def test_text_arrives_in_55_frames_and_is_acked():
    r = linked()
    text = hexframe("55 0F 00 00 AA 5A 56 0B 33 01 00 05 48 65 6C 6C 6F E3 DC"
                    + " 00" * 11 + " F4 73")
    assert r.on_brain_frame(text, 0.0) == [hexframe("02 55" + " 00" * 28 + " 0A B6")]
    assert r.screen_lines()[0] == "Hello"


def test_brain_05_reboots_a_duplicate_radio():
    r = linked()
    r.on_brain_frame(protocol.build(b"\x05", 32), 0.0)
    assert r.link == LinkMode.INFO


@pytest.mark.parametrize("phase, field, auton_mode, expected", [
    (Competition.NONE, False, False, 0x00),
    (Competition.DRIVER, False, False, 0x21),
    (Competition.AUTONOMOUS, False, False, 0x11),
    (Competition.DISABLED, False, False, 0x29),
    (Competition.DISABLED, True, False, 0xA9),
    (Competition.AUTONOMOUS, True, True, 0x91),
    (Competition.DISABLED, True, True, 0x99),
    (Competition.DRIVER, True, False, 0xA1)])
def test_status_byte(phase, field, auton_mode, expected):
    assert status_byte(phase, field, auton_mode) == expected


def test_05_is_acked_and_the_screen_survives_a_reboot():
    r = linked()
    text = hexframe("55 0F 00 00 AA 5A 56 0B 33 01 00 05 48 65 6C 6C 6F E3 DC"
                    + " 00" * 11 + " F4 73")
    r.on_brain_frame(text, 0.0)
    ack = r.ack_frame(0x05)                      # 32 B, before the restart
    assert r.on_brain_frame(protocol.build(b"\x05", 32), 0.0) == [ack]
    assert r.link == LinkMode.INFO and r.screen_lines()[0] == "Hello"


def test_text_retransmit_and_other_channels_are_ignored():
    r = linked()
    clear = bytes.fromhex("AA 5A 56 0B 33 01 00 05 48 65 6C 6C 6F E3 DC")
    r.on_brain_frame(protocol.build(bytes([0x55, len(clear), 0, 7]) + clear, 32), 0.0)
    wipe = bytes.fromhex("AA 5A 56 06 33 01 00 00")
    wipe += protocol.crc16_ccitt_false(wipe, init=0).to_bytes(2, "big")
    r.on_brain_frame(protocol.build(bytes([0x55, len(wipe), 0, 7]) + wipe, 32), 0.0)
    r.on_brain_frame(protocol.build(bytes([0x55, len(wipe), 1, 8]) + wipe, 32), 0.0)
    assert r.screen.lines()[0] == "Hello"
    r.on_brain_frame(protocol.build(bytes([0x55, len(wipe), 0, 8]) + wipe, 32), 0.0)
    assert r.screen.lines()[0] == ""


def robot_status(flags: int) -> bytes:
    return protocol.build(bytes([0x52, 0x00, 99, 0x00, flags]), 32)


def test_robot_status_marks_each_program_start():
    r = linked()
    assert r.program_started is None
    r.on_brain_frame(robot_status(0x01), 1.0)
    r.on_brain_frame(robot_status(0x41), 2.0)        # still running
    assert r.program_started == 1.0
    r.on_brain_frame(robot_status(0x00), 3.0)        # stopped
    assert r.program_started == 1.0
    r.on_brain_frame(robot_status(0x41), 4.0)        # run again
    assert r.program_started == 4.0
    r.on_brain_frame(protocol.build(b"\x05", 32), 5.0)
    assert r.program_started == 4.0                  # the controller keeps its clock
