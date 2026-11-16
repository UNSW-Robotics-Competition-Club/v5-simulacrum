"""Electromagnet, Light Tower, CTE Pneumatics and LED against
notes/misc-devices-protocol.md sections 2.5, 3.5, 4.5 and 5.5."""
import pytest

from v5sim import protocol
from v5sim.smart_device import LinkMode
from v5sim.smart_misc import SmartLed, SmartLightTower, SmartMagnet, SmartPneumatic


def hexframe(s: str) -> bytes:
    out = []
    for tok in s.split():
        if "*" in tok:
            b, n = tok.split("*")
            out += [int(b, 16)] * int(n)
        else:
            out.append(int(tok, 16))
    return bytes(out)


LINK_03 = "03 00 64 00*4 0A 00*6 69 B3"
LINK_0213 = "02 13 01 00*11 85 CC"


def link(dev, rate: str):
    assert dev.on_brain_frame(hexframe(LINK_03), 0.0) == [dev.ready_frame()]
    dev.on_brain_frame(hexframe(LINK_0213), 0.0)
    dev.on_brain_frame(hexframe(rate), 0.0)
    assert dev.link == LinkMode.DATA
    return dev


# ---- electromagnet ----------------------------------------------------

def magnet():
    return link(SmartMagnet(), "04 00 14 00 00*10 6E 4B")


def test_magnet_link_frames():
    m = SmartMagnet()
    assert m.info_frame() == hexframe("0A 03 00 00 11 00 00 40 00 20 00*4 55 ED")
    assert m.ready_frame() == hexframe("13 10 00*12 F4 8A")
    m = magnet()
    assert m.interval_s == pytest.approx(0.020)


def test_magnet_idle_frame():
    assert magnet().data_frame(0.0, None) == \
        hexframe("AD 00 00 08 00 00 00 00 00 00 00 00 19 01 05 30")


def test_magnet_pickup_frame_and_ack():
    m = magnet()
    assert m.on_brain_frame(hexframe("51 1F F4 01 00*10 13 29"), 0.0) == \
        [hexframe("02 51 00*12 56 AF")]
    assert m.power == 0x1F
    m.temp_c = 31.0
    assert m.data_frame(0.0, None) == \
        hexframe("AD 1F 1C 07 00 00 44 04 90 01 5A 1F 1F 00 EC 02")


def test_magnet_drop_and_timeout():
    m = magnet()
    m.on_brain_frame(hexframe("51 E1 F4 01 00*10 43 B4"), 1.0)
    assert m.power == -31
    assert m.data_payload(1.4, None)[1] == 0xE1
    p = m.data_payload(1.6, None)
    assert p[1] == 0 and p[13] & 0x01            # timed out: drive off


def test_magnet_zero_duration_holds_and_mode_ignored():
    m = magnet()
    m.on_brain_frame(protocol.build(bytes([0x51, 0x3E, 0, 0])), 0.0)
    assert m.data_payload(100.0, None)[1] == 0x3E
    assert m.on_brain_frame(hexframe("50 01 00 00 00 00 00 00 01 00 00*4 44 E1"), 100.0) \
        == [m.ack_frame(0x50)]
    assert m.power == 0x3E


def test_magnet_heats_while_driving_and_cools():
    m = magnet()
    m.on_brain_frame(protocol.build(bytes([0x51, 0x3E, 0, 0])), 0.0)
    m.data_payload(0.0, None)
    hot = m.data_payload(60.0, None)[12]
    assert hot > 30
    m.on_brain_frame(protocol.build(bytes([0x51, 0, 0, 0])), 60.0)
    m.data_payload(60.0, None)
    assert m.data_payload(200.0, None)[12] < hot


def test_magnet_reboot_drops_power():
    m = magnet()
    m.on_brain_frame(protocol.build(bytes([0x51, 0x1F, 0, 0])), 0.0)
    m.reboot(5.0)
    assert m.power == 0 and m.link == LinkMode.INFO


# ---- light tower ------------------------------------------------------

def tower():
    return link(SmartLightTower(), "04 00 0A 00 00*10 49 8D")


def test_tower_link_and_attach_clear():
    t = SmartLightTower()
    assert t.info_frame() == hexframe("0A 02 00 00 1B 00 00 40 00 20 00*4 84 4F")
    t = tower()
    assert t.interval_s == pytest.approx(0.010)
    assert t.on_brain_frame(hexframe("52 00 1F 00*11 96 53"), 0.0) == \
        [hexframe("02 52 00*12 99 0A")]
    assert t.data_frame(0.0, None) == hexframe("70 00*7 F4 F4 00*4 F8 A0")


def test_tower_rgb_blink_and_xyw():
    t = tower()
    t.on_brain_frame(hexframe("52 00 1F FF 00*10 2F 58"), 0.0)
    t.on_brain_frame(hexframe("52 00 1F FF 00 00 00 00 01 32 32 00*3 E3 14"), 0.0)
    assert t.data_frame(0.0, None) == hexframe("70 00 FF 00*4 01 F4 F4 00*4 7D 33")
    assert (t.on_ms, t.off_ms) == (500, 500)
    t.on_brain_frame(protocol.build(bytes([0x52, 0, 0x1F, 0x80, 0, 0xFF, 0x34, 0x12])), 0.0)
    assert t.data_frame(0.0, None) == hexframe("70 00 80 00 FF 34 12 00 F4 F4 00*4 E5 FA")


def test_tower_color_set_masks_channels():
    t = tower()
    t.on_brain_frame(hexframe("52 00 1F FF 00*10 2F 58"), 0.0)
    t.on_brain_frame(hexframe("52 00 04 00 00 FF 00*8 33 09"), 0.0)
    assert t.values == (0xFF, 0, 0xFF, 0, 0)


def test_tower_blink_phase_and_button():
    t = tower()
    t.on_brain_frame(protocol.build(bytes([0x52, 0, 0x1F, 0xFF, 0, 0, 0, 0, 0x01, 10, 30])), 0.0)
    assert (t.on_ms, t.off_ms) == (100, 300)
    assert t.lit(0.05)[0] == 0xFF and t.lit(0.2)[0] == 0
    t.pressed = True
    assert t.data_payload(0.0, None)[1] == 1


def test_tower_ignores_other_commands():
    t = tower()
    assert t.on_brain_frame(protocol.build(bytes([0x53, 0, 0x1F, 0xFF])), 0.0) == \
        [t.ack_frame(0x53)]
    assert t.values == (0, 0, 0, 0, 0)


# ---- pneumatics -------------------------------------------------------

def pneumatic():
    p = SmartPneumatic()
    assert p.on_brain_frame(hexframe(LINK_03), 0.0) == [hexframe("13 20 00*12 CF 56")]
    p.on_brain_frame(hexframe(LINK_0213), 0.0)
    p.on_brain_frame(hexframe("04 00 0A 00 00*26 73 90"), 0.0)
    return p


def test_pneumatic_link_and_idle():
    p = SmartPneumatic()
    assert p.info_frame() == hexframe("0A 03 00 00 1E 00 00 40 00 20 00*4 03 5C")
    p = pneumatic()
    assert p.frame_len == 32
    assert p.on_brain_frame(hexframe("50 00*9 02 00*19 B7 51"), 0.0) == \
        [hexframe("02 50 00*28 09 AB")]
    assert p.data_frame(0.0, None) == hexframe("70 00 AA 02 00*24 38 18 0D E1")


def test_pneumatic_extend_with_pump():
    p = pneumatic()
    p.on_brain_frame(hexframe("50 00*9 01 00 38 31 00*16 2D 2A"), 0.0)
    p.on_brain_frame(hexframe("50 00*9 04 00 38 31 00*16 CE 55"), 0.0)
    assert p.extended == (True, False, False, False) and p.pump
    p.pressure = 7000
    assert p.data_frame(0.0, None) == hexframe("70 00 A5 02 01 00*23 58 1B 51 A4")


def test_pneumatic_same_state_no_count_and_decay():
    p = pneumatic()
    ext = protocol.build(bytes([0x50] + [0] * 9 + [0x10, 0]), 32)
    p.on_brain_frame(ext, 0.0)
    p.on_brain_frame(ext, 0.1)
    assert p.counts == (0, 1, 0, 0)
    p.on_brain_frame(protocol.build(bytes([0x50] + [0] * 9 + [0x20, 0]), 32), 0.2)
    assert p.counts == (0, 2, 0, 0) and p.extended[1] is False
    p.data_payload(0.2, None)
    assert p.data_payload(1.3, None)[4:8] == bytes([0, 1, 0, 0])
    assert p.data_payload(5.0, None)[4:8] == bytes(4)


def test_pneumatic_pressure_model():
    p = pneumatic()
    start = p.pressure
    p.on_brain_frame(protocol.build(bytes([0x50] + [0] * 9 + [0x01, 0]), 32), 0.0)
    p.data_payload(0.0, None)
    p.data_payload(30.0, None)
    full = p.pressure
    assert start < full <= 9000
    p.on_brain_frame(protocol.build(bytes([0x50] + [0] * 9 + [0x02, 0]), 32), 30.0)
    p.on_brain_frame(protocol.build(bytes([0x50] + [0] * 9 + [0x04, 0]), 32), 30.0)
    assert p.pressure < full


# ---- LED --------------------------------------------------------------

def led():
    return link(SmartLed(), "04 00 32 00 00*10 0B D5")


def test_led_frames():
    d = SmartLed()
    assert d.info_frame() == hexframe("0A 04 00 00 03 00 00 40 00 20 00*4 D5 F9")
    d = led()
    assert d.interval_s == pytest.approx(0.050)
    assert d.data_frame(0.0, None) == hexframe("70 00*13 D9 ED")


def test_led_colour_and_brightness():
    d = led()
    assert d.on_brain_frame(hexframe("52 FF 00 00 00 00 00 FF 00*6 02 6F"), 0.0) == \
        [hexframe("02 52 00*12 99 0A")]
    assert d.rgb == (255, 0, 0)
    d.on_brain_frame(hexframe("52 FF FF FF 00 00 00 FF 00*6 86 96"), 0.0)
    assert d.rgb == (255, 255, 255)
    d.on_brain_frame(protocol.build(bytes([0x51, 40, 20, 0, 0, 0, 0, 50])), 0.0)
    assert d.rgb == (20, 10, 0)


def test_magnet_drive_off_when_link_drops_to_idle():
    m = magnet()
    m.on_brain_frame(protocol.build(bytes([0x51, 0x1F, 0, 0])), 0.0)
    m.link = LinkMode.INFO                 # brain restarts the port
    m.on_brain_frame(hexframe(LINK_03), 1.0)
    assert m.power == 0


def test_tower_colour_update_keeps_blink_phase():
    t = tower()
    blink = bytes([0x52, 0, 0x1F, 0xFF, 0, 0, 0, 0, 0x01, 10, 10])
    t.on_brain_frame(protocol.build(blink), 0.0)
    t.on_brain_frame(protocol.build(bytes([0x52, 0, 0x02, 0, 0x80, 0, 0, 0, 0x01, 10, 10])), 0.15)
    assert t.lit(0.15)[0] == 0             # still in the off half of the first period


def test_pneumatic_counters_decay_each_on_their_own_clock():
    p = pneumatic()
    p.on_brain_frame(protocol.build(bytes([0x50] + [0] * 9 + [0x04, 0]), 32), 0.0)
    p.on_brain_frame(protocol.build(bytes([0x50] + [0] * 9 + [0x10, 0]), 32), 0.9)
    assert p.data_payload(1.0, None)[4:6] == bytes([0, 1])
    assert p.data_payload(1.95, None)[4:6] == bytes([0, 0])
    assert p.data_payload(0.5, None)[4:8] == bytes(4)    # time going back: no change
