"""SmartVision (legacy V5 Vision Sensor, type 11) against
notes/vision-and-extras-protocol.md section A.5."""
import pytest

from v5sim import protocol
from v5sim.smart_device import LinkMode
from v5sim.smart_vision import (SmartVision, VisionObject, VisionSample,
                                VisionView, uv_for_rgb)


def f64(text: str) -> bytes:
    """Note syntax: hex bytes, `[00 xN]` = N zero bytes; CRC added."""
    out = []
    for tok in text.replace("[", " ").replace("]", " ").split():
        if tok.startswith("x"):
            out += [out.pop()] * int(tok[1:])
        else:
            out.append(int(tok, 16))
    return protocol.build(bytes(out), 64)


def linked() -> SmartVision:
    v = SmartVision()
    v.on_brain_frame(protocol.build(bytes.fromhex("03 00 64 00 00 00 00 0A")), 0.0)
    v.on_brain_frame(protocol.build(bytes.fromhex("02 13 01")), 0.0)
    v.on_brain_frame(f64("04 00 64"), 0.0)
    return v


RED_SIG = ("50 55 11 00 01 00 00 00 CD CC AC 40 0D 23 00 00 87 2B 00 00 4A 27 00 00 "
           "B9 F7 FF FF E3 FB FF FF CE F9 FF FF 00 00 FF")


def test_link_frames():
    v = SmartVision()
    assert v.info_frame() == protocol.build(bytes.fromhex("0A 12 00 00 0B 00 00 40"))
    assert v.ready_frame() == protocol.build(bytes.fromhex("13 40"))
    v = linked()
    assert v.link == LinkMode.DATA and v.frame_len == 64
    assert v.interval_s == pytest.approx(0.100)


def test_default_status_matches_reference():
    assert linked().status_frame() == f64("8F 00 01 00 32 00 00 00 00 00 32 80 80 80 [00 x48]")
    assert linked().status_frame()[-2:] == bytes.fromhex("D1 91")


def test_config_commands_are_echoed_in_the_status():
    v = linked()
    for cmd in ("51 01 00 04 00 00 00 00 00 00 32 80 80 80",       # ModeSet(1)
                "51 00 02",                                        # WifiModeSet(on)
                "51 00 00 01 4B",                                  # Brightness 75
                "51 00 00 02 00 02 40 80 C8",                      # WB manual
                "51 00 00 04 00 00 00 00 00 01 46 FF"):            # LED red 70
        assert v.on_brain_frame(f64(cmd), 0.0) == [v.ack_frame(int(cmd[:2], 16))]
    s = v.status_frame()
    assert s[1:14] == bytes([0, 2, 0, 75, 1, 64, 128, 200, 1, 70, 255, 0, 0])


def test_mode_set_reaches_the_status():
    v = linked()
    v.on_brain_frame(f64("51 01 00 04 00 00 00 00 00 00 32 80 80 80"), 0.0)
    assert v.status_frame() == f64("8F 01 01 00 32 00 00 00 00 00 32 80 80 80 [00 x48]")


def test_signature_set_is_stored():
    v = linked()
    v.on_brain_frame(f64(RED_SIG), 0.0)
    sig = v.signatures[1]
    assert (sig.u_min, sig.u_max, sig.u_mean) == (8973, 11143, 10058)
    assert (sig.v_min, sig.v_max, sig.v_mean) == (-2119, -1053, -1586)
    assert sig.range == pytest.approx(5.4)


def test_object_packets_match_reference():
    v = linked()
    objs = [VisionObject(1, 100, 60, 40, 30, 0), VisionObject(0x11, 200, 100, 50, 20, 4500)]
    assert v.object_frames(objs, seq=0) == [f64(
        "92 02 00 00 01 00 32 3C 14 1E 00 00 11 00 64 64 19 14 94 11 [00 x42]")]
    assert v.object_frames([], seq=0) == [f64("92 [00 x61]")]


def test_nine_objects_split_into_first_and_last():
    v = linked()
    objs = [VisionObject(i % 7 + 1 if i < 7 else i - 6, 10 + 20 * i, 20 + 5 * i, 16, 12, 0)
            for i in range(9)]
    first, last = v.object_frames(objs, seq=0x1234)
    assert first == f64("90 07 34 12 01 00 05 14 08 0C 00 00 02 00 0F 19 08 0C 00 00 "
                        "03 00 19 1E 08 0C 00 00 04 00 23 23 08 0C 00 00 05 00 2D 28 08 0C 00 00 "
                        "06 00 37 2D 08 0C 00 00 07 00 41 32 08 0C 00 00 [00 x4]")
    assert last == f64("92 02 34 12 01 00 4B 37 08 0C 00 00 02 00 55 3C 08 0C 00 00 [00 x44]")


def test_red_block_matches_a_vision_utility_red_signature():
    v = linked()
    v.on_brain_frame(f64(RED_SIG), 0.0)
    red = VisionView(uv_for_rgb((200, 40, 15)), 100, 80, 30, 30)
    blue = VisionView(uv_for_rgb((20, 60, 200)), 200, 80, 30, 30)
    assert v.detect(VisionSample((red, blue))) == [VisionObject(1, 100, 80, 30, 30, 0)]


def test_detect_largest_first_and_capped():
    v = linked()
    v.on_brain_frame(f64(RED_SIG), 0.0)
    uv = uv_for_rgb((200, 40, 15))
    views = tuple(VisionView(uv, 2 * i, 0, 10 + i, 10 + i) for i in range(40))
    objs = v.detect(VisionSample(views))
    assert len(objs) == 32 and objs[0].w == 49


def test_report_is_status_then_objects_one_per_heartbeat():
    v = linked()
    v.on_brain_frame(f64(RED_SIG), 0.0)
    view = VisionView(uv_for_rgb((200, 40, 15)), 100, 80, 30, 30)
    (status,) = v.due_frames(0.1, VisionSample((view,)))
    assert status[0] == 0x8F
    assert v.on_brain_frame(f64("51 00 00 01 4B"), 0.1) == [v.ack_frame(0x51)]
    (objects,) = v.on_brain_frame(f64("02 51 01 0B 00 00 00 00 00 00 01"), 0.1)
    assert objects[:2] == bytes([0x92, 1])
    assert v.on_brain_frame(f64("02 92 01 0B 00 00 00 00 00 00 01"), 0.1) == []
    assert v.due_frames(0.15, VisionSample((view,))) == []


def test_reboot_forgets_signatures():
    v = linked()
    v.on_brain_frame(f64(RED_SIG), 0.0)
    v.reboot(1.0)
    assert v.signatures == {} and v.link == LinkMode.INFO


def test_re_pair_drops_a_half_sent_report():
    v = linked()
    v.on_brain_frame(f64(RED_SIG), 0.0)
    view = VisionView(uv_for_rgb((200, 40, 15)), 100, 80, 30, 30)
    v.due_frames(0.1, VisionSample((view,)))                 # status out, 92 queued
    v.on_brain_frame(f64("03 00 64 00 00 00 00 0A"), 0.2)   # brain re-pairs
    v.on_brain_frame(f64("04 00 64"), 0.2)
    assert v.on_brain_frame(f64("02 04 01 0B 00 00 00 00 00 00 01"), 0.2) == []
