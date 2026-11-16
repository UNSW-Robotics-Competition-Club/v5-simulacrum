"""SmartAiVision against notes/aivision-protocol.md section 6."""
import struct

import pytest

from v5sim import protocol
from v5sim.smart_aivision import (AI, COLOR, AiObject, AiVisionSample, InView,
                                  SmartAiVision, hue_direction)
from v5sim.smart_device import LinkMode


def frame64(payload_hex: str) -> bytes:
    return protocol.build(bytes.fromhex(payload_hex), 64)


def linked() -> SmartAiVision:
    a = SmartAiVision()
    a.on_brain_frame(protocol.build(bytes.fromhex("03 01 64 00 00 00 00 0A 78 56 34 12")), 0.0)
    a.on_brain_frame(protocol.build(bytes.fromhex("02 13 01")), 0.0)
    a.on_brain_frame(frame64("04 00 14 00"), 0.0)
    return a


def test_link_frames():
    a = SmartAiVision()
    assert a.info_frame() == protocol.build(bytes.fromhex("0A 15 00 00 1D 00 00 40 00 00 20 00"))
    assert a.on_brain_frame(protocol.build(bytes.fromhex("03 01 64 00 00 00 00 0A 78 56 34 12")),
                            0.0) == [protocol.build(bytes.fromhex("13 40"))]
    a.on_brain_frame(protocol.build(bytes.fromhex("02 13 01")), 0.0)
    assert a.frame_len == 64
    assert a.on_brain_frame(frame64("04 00 14 00"), 0.0) == [frame64("02 04 01 00 00 00 00 00 00 00 01")]
    assert a.link == LinkMode.DATA and a.interval_s == pytest.approx(0.020)
    assert a.on_brain_frame(frame64("51 00 FF 01 FF"), 0.0) == [frame64("02 51 01 00 00 00 00 00 00 00 01")]


D5 = ("8F 05 00 24 40 01 F0 00 80 02 E0 01 40 01 40 01 00 05 00 00 00 00 00 00 "
      "00 00 00 00 00 00 1E 00 00 00 09 00 02 00 00 00 00 01 00 00 00 00 "
      "42 61 6C 6C 28 42 6C 75 65 29")
D7 = ("8F 05 00 24 40 01 F0 00 80 02 E0 01 40 01 40 01 00 05 00 00 00 00 00 00 "
      "01 00 00 00 00 00 1E 00 01 00 09 01 02 00 00 00 00 01 00 00 00 00 "
      "42 61 6C 6C 28 47 72 65 65 6E 29")


def test_status_frames_match_reference_and_cycle_class_names():
    a = linked()
    assert a.status_frame(colour_count=0) == frame64(D5)
    a.on_brain_frame(frame64("51 00 01 01 01 FF 00 00 00 00 A0 41 CD CC 4C 3E"), 0.0)
    assert a.status_frame(colour_count=1) == frame64(D7)


def test_one_colour_blob_chunk_matches_reference():
    a = linked()
    obj = AiObject(COLOR, 1, 140, 105, 40, 30, 0)
    assert a.chunk_frames([obj], seq=7) == [frame64(
        "90 81 01 01 8C 90 06 28 E0 01 00 00 00 00 00 00 07" + " FF" * 45)]


def test_two_ai_objects_chunk_matches_reference():
    a = linked()
    objs = [AiObject(AI, 2, 100, 120, 64, 48, 87), AiObject(AI, 6, 200, 75, 80, 60, 62)]
    assert a.chunk_frames(objs, seq=3) == [frame64(
        "90 81 02 04 64 80 07 40 00 03 57 00 00 00 00 00 03 "
        "06 04 C8 B0 04 50 C0 03 3E 00 00 00 00 00 03" + " FF" * 30)]


def test_many_objects_split_into_first_and_last_chunks():
    a = linked()
    objs = [AiObject(COLOR, 1, 10 * i, 0, 5, 5, 0) for i in range(5)]
    frames = a.chunk_frames(objs, seq=0)
    assert [f[1] for f in frames] == [0x01, 0x80]
    assert frames[1][3] == COLOR and frames[1][18] == 0xFF       # 1 slot used


def red_block_in_view() -> AiVisionSample:
    return AiVisionSample((InView((200, 25, 25), 140, 105, 40, 30, "red"),))


def test_report_sequence_status_first_then_chunks_on_acks():
    a = linked()
    sig = frame64("51 00 01 01 01 FF 00 00 00 00 A0 41 CD CC 4C 3E")   # red, 20 deg
    a.on_brain_frame(sig, 0.0)
    first = a.due_frames(0.02, red_block_in_view())
    assert first[0][0] == 0x8F                       # first DATA frame: status
    report = a.due_frames(0.04, red_block_in_view())
    assert report[0][:2] == bytes([0x90, 0x81])      # one chunk, red sig id 1
    assert report[0][2:4] == bytes([1, COLOR])
    after = a.on_brain_frame(frame64("02 90 01 01 00 00 02 00"), 0.04)
    assert after[0][0] == 0x8F                       # then the status
    assert a.on_brain_frame(frame64("02 8F 01 01 00 00 02 00"), 0.04) == []


def test_colour_detection_needs_a_matching_signature():
    a = linked()
    a.on_brain_frame(frame64("51 00 01 01 01 00 00 FF 00 00 A0 41 CD CC 4C 3E"), 0.0)  # blue sig
    a.due_frames(0.02, red_block_in_view())
    assert a.due_frames(0.04, red_block_in_view())[0][0] == 0x8F   # nothing matched


def test_model_detection_reports_cube_classes():
    a = linked()
    a.on_brain_frame(frame64("50 02 00 01"), 0.0)    # tags off, colour on, AI on
    a.due_frames(0.02, red_block_in_view())
    chunk = a.due_frames(0.04, red_block_in_view())[0]
    assert chunk[0] == 0x90 and chunk[2:4] == bytes([8, AI])      # Cube(Red)
    assert struct.unpack_from("<H", chunk, 10)[0] > 60             # score %


def test_hue_direction_separates_red_and_blue():
    import math
    red, blue = hue_direction((255, 0, 0)), hue_direction((0, 0, 255))
    assert abs(math.degrees(red - blue)) % 360 > 90


def red_signature(a):
    a.on_brain_frame(frame64("51 00 01 01 01 FF 00 00 00 00 A0 41 CD CC 4C 3E"), 0.0)


def test_counts_match_what_is_reported():
    a = linked()
    red_signature(a)
    many = AiVisionSample(tuple(InView((200, 25, 25), i, 0, 10, 10, "red") for i in range(300)))
    a.due_frames(0.02, many)                          # status first
    a.due_frames(0.04, many)                          # 6 chunks queued + status
    frames = [a.on_brain_frame(frame64("02 90 01 01 00 00 02 00"), 0.04) for _ in range(6)]
    status = frames[-1][0]
    assert status[0] == 0x8F and status[0x18] == 24   # not 300


def test_class_names_advance_only_when_a_status_is_sent():
    a = linked()
    red_signature(a)
    seen = AiVisionSample((InView((200, 25, 25), 10, 10, 20, 20, "red"),))
    a.due_frames(0.02, seen)                          # status, name 0
    for t in (0.04, 0.06, 0.08):                      # brain never acks: no status sent
        a.due_frames(t, seen)
    status = a.on_brain_frame(frame64("02 90 01 01 00 00 02 00"), 0.08)[0]
    assert status[0x23] == 1                          # the next name, not 4


def test_grey_and_unsaturated_things_dont_match_a_red_signature():
    a = linked()
    red_signature(a)
    grey = AiVisionSample((InView((60, 60, 60), 10, 10, 20, 20, "grey"),
                           InView((140, 110, 110), 40, 10, 20, 20, "pink")))
    assert a.detect(grey) == []


def test_detect_is_pure():
    a = linked()
    red_signature(a)
    before = a.status_frame()
    a.detect(red_block_in_view())
    a._name_idx = 0
    assert a.status_frame() == before


# ---- AprilTags and colour codes (notes/vision-and-extras-protocol.md B) --

from v5sim.smart_aivision import CODE, TAG, TagInView  # noqa: E402

UPRIGHT = ((100, 120), (140, 120), (140, 80), (100, 80))         # BL BR TR TL
TILTED = ((173, 107), (207, 127), (227, 93), (193, 73))


def test_tag_chunks_match_reference():
    a = linked()
    assert a.chunk_frames([AiObject(TAG, 5, 100, 80, 40, 40, 0, UPRIGHT)], seq=7) == [frame64(
        "90 81 05 08 64 80 07 8C 80 07 8C 00 05 64 00 05 07" + " FF" * 45)]
    assert a.chunk_frames([AiObject(TAG, 7, 173, 73, 54, 54, 0, TILTED)], seq=7) == [frame64(
        "90 81 07 08 AD B0 06 CF F0 07 E3 D0 05 C1 90 04 07" + " FF" * 45)]


def test_colour_code_chunk_matches_reference():
    a = linked()
    assert a.chunk_frames([AiObject(CODE, 3, 100, 60, 80, 50, 4500)], seq=7) == [frame64(
        "90 81 03 02 64 C0 03 50 20 03 94 11 00 00 00 00 07" + " FF" * 45)]


def test_negative_tag_corners_are_signed_12_bit():
    a = linked()
    corners = ((-5, 10), (20, 10), (20, -3), (-5, -3))
    slot = a.chunk_frames([AiObject(TAG, 1, -5, -3, 25, 13, 0, corners)], seq=0)[0][2:17]
    assert slot[2:5] == bytes([0xFB, 0xAF, 0x00])                # x -5, y 10


TAG_ON = "50 02 00 04"


def test_tags_need_tag_detection_and_the_right_family():
    a = linked()
    seen = AiVisionSample(tags=(TagInView(5, UPRIGHT),))
    assert a.detect(seen) == []                                   # off by default
    a.on_brain_frame(frame64(TAG_ON), 0.0)
    (tag,) = a.detect(seen)
    assert (tag.type, tag.id, tag.x, tag.y, tag.w, tag.h) == (TAG, 5, 100, 80, 40, 40)
    assert tag.corners == UPRIGHT
    assert a.detect(AiVisionSample(tags=(TagInView(5, UPRIGHT, family=3),))) == []
    a.on_brain_frame(frame64("50 20 00 00 00 00 00 00 00 13"), 0.0)   # 36h11, decimate
    assert a.tag_family == 3
    assert len(a.detect(AiVisionSample(tags=(TagInView(5, UPRIGHT, family=3),)))) == 1


def test_tags_come_first_and_are_counted_in_the_status():
    a = linked()
    a.on_brain_frame(frame64(TAG_ON), 0.0)
    a.on_brain_frame(frame64("51 00 01 01 01 FF 00 00 00 00 A0 41 CD CC 4C 3E"), 0.0)
    red = InView((200, 25, 25), 10, 10, 30, 30, "red")
    objs = a.detect(AiVisionSample(in_view=(red,), tags=(TagInView(9, UPRIGHT),)))
    assert [o.type for o in objs] == [TAG, COLOR]
    a.data_frame(0.0, AiVisionSample())                            # first: status
    a.data_frame(0.1, AiVisionSample(in_view=(red,), tags=(TagInView(9, UPRIGHT),)))
    assert a.status_frame()[0x18:0x1B] == bytes([1, 1, 0])


def code_sensor():
    a = linked()
    a.on_brain_frame(frame64("51 00 01 01 01 FF 00 00 00 00 A0 41 CD CC 4C 3E"), 0.0)   # red
    a.on_brain_frame(frame64("51 00 02 01 02 00 00 FF 00 00 A0 41 CD CC 4C 3E"), 0.0)   # blue
    a.on_brain_frame(frame64("51 00 01 02 01 02 01 00 02 00"), 0.0)                    # B8
    return a


def test_colour_code_from_adjacent_member_colours():
    a = code_sensor()
    red = InView((200, 25, 25), 100, 60, 30, 40, "red")
    blue = InView((25, 60, 200), 135, 70, 40, 40, "blue")         # 5 px gap
    objs = a.detect(AiVisionSample(in_view=(red, blue)))
    code = objs[0]
    assert (code.type, code.id, code.x, code.y, code.w, code.h) == (CODE, 1, 100, 60, 75, 50)
    assert code.extra == 34596             # registered order red -> blue, 14.04 deg
    assert {o.type for o in objs[1:]} == {COLOR}


def test_colour_code_needs_order_and_closeness():
    a = code_sensor()
    red = InView((200, 25, 25), 100, 60, 30, 40, "red")
    far_blue = InView((25, 60, 200), 160, 60, 30, 40, "blue")      # 30 px gap
    assert CODE not in {o.type for o in a.detect(AiVisionSample(in_view=(red, far_blue)))}
    left_blue = InView((25, 60, 200), 65, 60, 30, 40, "blue")      # reversed order
    (code,) = [o for o in a.detect(AiVisionSample(in_view=(red, left_blue))) if o.type == CODE]
    assert code.id == 1


def test_code_clear_and_reset():
    a = code_sensor()
    assert a.codes == {1: (1, 2)}
    a.on_brain_frame(frame64("51 00 FF 02 FF"), 0.0)
    assert a.codes == {}


def test_vexcode_codeset_pads_members_with_zeros():
    a = linked()
    a.on_brain_frame(frame64("51 00 01 02 01 05 01 00 02 00 00 00 00 00 00 00"), 0.0)
    assert a.codes == {1: (1, 2)}


def test_re_pair_drops_queued_chunks():
    a = linked()
    a.on_brain_frame(frame64("51 00 01 01 01 FF 00 00 00 00 A0 41 CD CC 4C 3E"), 0.0)
    red = InView((200, 25, 25), 10, 10, 30, 30, "red")
    a.data_frame(0.0, AiVisionSample())
    a.data_frame(0.1, AiVisionSample(in_view=(red,)))          # chunk out, status queued
    a.on_brain_frame(frame64("03 01 64 00 00 00 00 0A"), 0.2)
    a.on_brain_frame(frame64("04 00 14 00"), 0.2)
    assert len(a.on_brain_frame(frame64("02 90 01 01"), 0.2)) == 0


def test_malformed_tag_corners_are_dropped_or_clamped():
    a = linked()
    a.on_brain_frame(frame64(TAG_ON), 0.0)
    three = TagInView(1, UPRIGHT[:3])
    huge = TagInView(2, ((-5000, 10), (5000, 10), (5000, -3), (-5000, -3)))
    objs = a.detect(AiVisionSample(tags=(three, huge)))
    assert [o.id for o in objs] == [2]
    slot = a.chunk_frames(objs, seq=0)[0][2:17]
    assert len(slot) == 15 and slot[2:5] == bytes([0x00, 0xA8, 0x00])   # x -2048, y 10
