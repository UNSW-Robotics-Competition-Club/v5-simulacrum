import pytest

from v5sim.match import PAUSE_S, MatchSchedule
from v5sim.smart_controller import Competition


def test_parse_defaults_to_a_vrc_match():
    m = MatchSchedule.parse("20")
    assert (m.start_s, m.auton_s, m.driver_s) == (20.0, 15.0, 105.0)
    assert MatchSchedule.parse("5:1.5:3") == MatchSchedule(5.0, 1.5, 3.0)


@pytest.mark.parametrize("text", ["", "x", "1:2:3:4", "-1", "1:0", "1:2:-3"])
def test_parse_rejects_bad_text(text):
    with pytest.raises(ValueError):
        MatchSchedule.parse(text)


def test_phases_in_order():
    m = MatchSchedule(10.0, 15.0, 105.0)
    driver_at = 10 + 15 + PAUSE_S
    assert m.phase_at(0.0) == Competition.DISABLED       # waiting for the field
    assert m.phase_at(10.0) == Competition.AUTONOMOUS
    assert m.phase_at(24.99) == Competition.AUTONOMOUS
    assert m.phase_at(25.0) == Competition.DISABLED      # between periods
    assert m.phase_at(driver_at) == Competition.DRIVER
    assert m.phase_at(driver_at + 104.99) == Competition.DRIVER
    assert m.phase_at(driver_at + 105.0) == Competition.DISABLED
    assert m.end_s == driver_at + 105.0


def test_driver_only_skills_run_has_no_auton():
    m = MatchSchedule.parse("3:0:60")
    assert m.phase_at(3.0) == Competition.DRIVER
    assert m.phase_at(63.0) == Competition.DISABLED
