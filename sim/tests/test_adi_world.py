import math

import pytest

from v5sim.adi_world import (ENCODER_TICKS_PER_REV, AdiWiring, World,
                             adi_sample)
from v5sim.smart_adi import NO_ECHO


def test_parse_wiring():
    w = AdiWiring.parse("A=bumper,C=piston,E=encoder,G=sonar")
    assert w.kinds == ("bumper", None, "piston", None, "encoder", "encoder",
                       "sonar", "sonar")
    assert w.describe_ports("piston") == ["C"]


@pytest.mark.parametrize("text", ["A", "I=bumper", "A=laser", "B=encoder",
                                  "A=bumper,A=piston", "E=encoder,F=bumper"])
def test_parse_rejects_bad_wiring(text):
    with pytest.raises(ValueError):
        AdiWiring.parse(text)


def test_empty_wiring_is_the_builtin_block():
    # No devices is valid: the built-in port-22 block is present (unwired) so
    # the on-brain 3-wire screen has a device to configure.
    assert AdiWiring.parse("").kinds == (None,) * 8
    assert AdiWiring.empty().kinds == (None,) * 8


def test_bumper_presses_at_the_wall_only():
    w = AdiWiring.parse("A=bumper,B=limit")
    free = adi_sample(w, World(front_gap_m=0.5))
    touching = adi_sample(w, World(front_gap_m=0.0))
    assert not free.digital[0] and not free.digital[1]  # open switch reads 0
    assert touching.digital[0] and touching.digital[1]  # closed reads 1


def test_encoder_counts_tracking_wheel_travel():
    w = AdiWiring.parse("C=encoder")
    circumference = math.pi * 2.75 * 0.0254
    s = adi_sample(w, World(travel_m=circumference, speed_m_s=circumference))
    assert s.encoders[1] == (ENCODER_TICKS_PER_REV, ENCODER_TICKS_PER_REV)


def test_sonar_echo_time_from_wall_gap():
    w = AdiWiring.parse("A=sonar")
    assert adi_sample(w, World(front_gap_m=0.5)).echo_us[0] == 500 * 58 // 10
    assert adi_sample(w, World(front_gap_m=4.0)).echo_us[0] == NO_ECHO   # out of range


def test_parse_analog_kinds_with_a_shaft():
    w = AdiWiring.parse("A=pot:7,B=potv2:8,C=line,D=light,E=accel,F=gyro")
    assert w.kinds == ("pot", "potv2", "line", "light", "accel", "gyro", None, None)
    assert w.shafts == (7, 8, None, None, None, None, None, None)


@pytest.mark.parametrize("text", ["A=pot", "A=pot:x", "A=line:3"])
def test_parse_rejects_bad_shafts(text):
    with pytest.raises(ValueError):
        AdiWiring.parse(text)


def test_pot_follows_its_motor_shaft_and_clamps():
    w = AdiWiring.parse("A=pot:7,B=potv2:7")
    s = adi_sample(w, World(shaft_deg={7: 125.0}))
    assert s.analog[:2] == (2048, round(125 / 330 * 4096))
    s = adi_sample(w, World(shaft_deg={7: 400.0}))
    assert s.analog[:2] == (4095, 4095)
    assert adi_sample(w, World(shaft_deg={7: -10.0})).analog[0] == 0


def test_line_light_accel_and_gyro_read_the_world():
    w = AdiWiring.parse("A=line,B=light,C=accel,D=gyro")
    s = adi_sample(w, World(accel_m_s2=9.80665, heading_deg=-12.34))
    line, light, accel = s.analog[:3]
    assert 100 - line // 30 == 30                    # grey tile, VEXcode's maths
    assert 100 - light // 9 == 50
    assert accel == 2048 + 275                       # +1 g forward
    assert s.heading_tenths == -123
