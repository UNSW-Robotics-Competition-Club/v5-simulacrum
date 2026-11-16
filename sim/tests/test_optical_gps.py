"""SmartOptical / SmartGps against notes/gps-optical-protocol.md section 4."""
import math
import struct

import pytest

from v5sim import protocol
from v5sim.smart_device import LinkMode
from v5sim.smart_gps import GpsSample, SmartGps
from v5sim.smart_optical import OpticalSample, SmartOptical, raw_for_rgb


def hexframe(s: str) -> bytes:
    out = []
    for tok in s.split():
        if "*" in tok:
            b, n = tok.split("*")
            out += [int(b, 16)] * int(n)
        else:
            out.append(int(tok, 16))
    return bytes(out)


def link(dev, length):
    dev.on_brain_frame(hexframe("03 00 64 00*4 0A 00*6 69 B3"), 0.0)
    dev.on_brain_frame(hexframe("02 13 01 00*11 85 CC"), 0.0)
    rate = {32: "04 00 14 00*27 F5 6B", 64: "04 00 0A 00*59 DA E8"}[length]
    dev.on_brain_frame(hexframe(rate), 0.0)
    return dev


# ---- optical ----------------------------------------------------------

def test_optical_link_frames():
    o = SmartOptical()
    assert o.info_frame() == hexframe("0A 04 00 00 10 00 00 40 00 20 00*4 F0 A0")
    assert o.on_brain_frame(hexframe("03 00 64 00*4 0A 00*6 69 B3"), 0.0) == \
        [hexframe("13 20 00*12 CF 56")]
    o.on_brain_frame(hexframe("02 13 01 00*11 85 CC"), 0.0)
    assert o.frame_len == 32
    assert o.on_brain_frame(hexframe("50 00 41 00*4 0A 00*22 9E A7"), 0.0) == \
        [hexframe("02 50 00*28 09 AB")]


def test_optical_data_frames_match_reference():
    o = link(SmartOptical(), 32)
    o.on_brain_frame(hexframe("50 00 41 00*4 0A 00*22 9E A7"), 0.0)   # attach: LED 0, gain 10
    o.on_brain_frame(hexframe("50 00 01 00 64 00*25 2A 98"), 0.0)     # LED 100
    reddish = OpticalSample(clear=4000, red=3000, green=500, blue=400, proximity=200)
    assert o.data_frame(0, reddish) == hexframe(
        "AB 00 03 00 00 00 A0 0F B8 0B F4 01 90 01 C8 64 07 DB 0A 87 01 00*9 AA CE")
    o.on_brain_frame(hexframe("50 00 01 00*27 F8 A3"), 0.0)           # LED 0
    assert o.data_frame(0, OpticalSample()) == hexframe(
        "AB 00 01 00*13 07 DB 0A 87 01 00*9 3E E0")


def test_optical_config_updates_shadows():
    o = link(SmartOptical(), 32)
    o.on_brain_frame(hexframe("50 00 20 00 00 00 EE 00*23 A7 11"), 0.0)  # ATIME 0xEE
    o.on_brain_frame(hexframe("50 00 10 00 00 4F 00*24 1F EA"), 0.0)     # gesture mode
    o.on_brain_frame(hexframe("50 00 40 00*4 09 00*22 15 2E"), 0.0)      # gain x4
    f = o.data_frame(0, OpticalSample())
    assert (f[16], f[17], f[18], f[19]) == (0x4F, 0xEE, 0x09, 0x89)


def sdk_hue_sat_bright(f: bytes) -> tuple[float, float, float]:
    """The SDK's conversion (notes section 2.4)."""
    c, r_, g_, b_ = struct.unpack_from("<4H", f, 6)
    counts = min((256 - f[17]) * 1024 + 1, 65535)
    r = 0.7 * r_
    g = 0.1 * r_ + g_ - 0.2 * b_
    b = -0.1 * r_ - 0.2 * g_ + 1.05 * b_
    mx, mn = max(r, g, b), min(r, g, b)
    sat = 0 if mx == 0 else 1 - mn / mx
    if sat < 1e-5:
        return 0.0, 0.0, c / counts
    if r == mx:
        h = (g - b) / mx / sat
    elif g == mx:
        h = 2 + (b - r) / mx / sat
    else:
        h = 4 + (r - g) / mx / sat
    return (60 * h) % 360, sat, c / counts


@pytest.mark.parametrize("rgb,hue", [((200, 20, 20), 0.0), ((20, 40, 200), 233.3),
                                     ((30, 200, 30), 120.0)])
def test_raw_for_rgb_round_trips_through_the_sdk(rgb, hue):
    o = link(SmartOptical(), 32)
    raw = raw_for_rgb(rgb, brightness=0.3, atime=o.atime)
    h, sat, bright = sdk_hue_sat_bright(o.data_frame(0, OpticalSample(*raw, proximity=255)))
    assert abs((h - hue + 180) % 360 - 180) < 1.0          # hue is an angle
    assert bright == pytest.approx(0.3, abs=0.01)
    assert sat > 0.5


# ---- GPS --------------------------------------------------------------

def test_gps_link_frames():
    g = SmartGps()
    assert g.info_frame() == hexframe("0A 0A 00 00 14 00 00 40 00*6 D1 10")
    assert g.on_brain_frame(hexframe("03 00 64 00*4 0A 00*6 69 B3"), 0.0) == \
        [hexframe("13 40 00*12 B8 EE")]
    g.on_brain_frame(hexframe("02 13 01 00*11 85 CC"), 0.0)
    assert g.frame_len == 64
    assert g.on_brain_frame(hexframe("04 00 0A 00*59 DA E8"), 0.0) == \
        [hexframe("02 04 01 00*7 01 00*51 88 8D")]
    assert g.link == LinkMode.DATA and g.interval_s == pytest.approx(0.020)  # device floor


def test_gps_data_frame_matches_reference():
    g = link(SmartGps(), 64)
    s = GpsSample(x_m=0.5, y_m=-0.25, yaw_rad=math.pi / 2, error_m=0.01,
                  temperature_c=30)
    assert g.data_frame(0, s) == hexframe(
        "AE 02 00 1E F3 04 35 3F 00*8 F3 04 35 3F 00*5 20 00*9 3F 00 00 80 BE "
        "0A D7 23 3C DB 0F C9 3F 00*8 0A D7 23 3C 00 00 DF AD")


def test_gps_no_fix_frame():
    g = link(SmartGps(), 64)
    s = GpsSample(quality=4, temperature_c=30)
    assert g.data_frame(0, s) == hexframe("AE 02 00 1E 00 00 80 3F 00*17 20 00*34 04 00 58 95")


def test_gps_initial_position_seeds_the_pose():
    g = link(SmartGps(), 64)
    reply = g.on_brain_frame(hexframe(
        "50 00 00 40 00*34 80 3F 00 00 00 BF DB 0F C9 3F 00*14 90 C4"), 0.0)
    assert reply == [hexframe("02 50 01 00*7 01 00*51 2B 3E")]
    assert g.seed == pytest.approx((1.0, -0.5, math.pi / 2), abs=1e-6)


def test_gesture_reported_for_160_ms_in_gesture_mode():
    o = link(SmartOptical(), 32)
    o.on_brain_frame(hexframe("50 00 10 00 00 4F 00*24 1F EA"), 0.0)     # GestureEnable
    o.request_gesture("up")
    f = o.data_payload(1.0, OpticalSample())
    assert (f[1], f[21]) == (1, 12)
    assert o.data_payload(1.15, OpticalSample())[1] == 1
    assert o.data_payload(1.17, OpticalSample())[1:2] + o.data_payload(1.17, OpticalSample())[21:22] \
        == b"\x00\x00"
    o.request_gesture("left")
    assert o.data_payload(1.2, OpticalSample())[1] == 4                   # raw 4 = VEX left


def test_gesture_ignored_outside_gesture_mode():
    o = link(SmartOptical(), 32)
    o.request_gesture("right")
    assert o.data_payload(1.0, OpticalSample())[1] == 0
    o.on_brain_frame(hexframe("50 00 10 00 00 4F 00*24 1F EA"), 2.0)
    assert o.data_payload(2.1, OpticalSample())[1] == 0                   # not queued


def test_unknown_gesture_is_rejected():
    with pytest.raises(ValueError):
        SmartOptical().request_gesture("sideways")
