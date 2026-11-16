"""The robot battery (battery.py): sag under load, drain over a match."""
import pytest

from v5sim.battery import Battery, BatteryState, open_circuit_v


def test_open_circuit_voltage_falls_with_charge():
    full, half, low, empty = (open_circuit_v(c) for c in (1.0, 0.5, 0.1, 0.0))
    assert full > half > low > empty
    assert 13.0 < half < 13.6                     # LiFePO4 plateau, 4 cells


def test_no_load_terminal_voltage_is_the_open_circuit_voltage():
    b = Battery()
    st = b.step(BatteryState(), load_w=0.0, dt=0.001)
    assert st.terminal_v == pytest.approx(open_circuit_v(1.0) - b.idle_a * b.r_ohm, rel=1e-6)


def test_heavy_load_sags_the_voltage():
    b = Battery()
    light = b.step(BatteryState(), load_w=20.0, dt=0.001)
    heavy = b.step(BatteryState(), load_w=250.0, dt=0.001)  # eight stalled motors
    assert heavy.terminal_v < light.terminal_v - 1.5
    assert heavy.current_a == pytest.approx(250.0 / heavy.terminal_v + b.idle_a, rel=0.02)


def test_a_match_of_driving_drains_it():
    b = Battery()
    st = BatteryState()
    for _ in range(120_000):                       # 2 min at 60 W, 1 ms steps
        st = b.step(st, load_w=60.0, dt=0.001)
    used_ah = 1.1 * (1.0 - st.charge)
    assert used_ah == pytest.approx((60.0 / 13.2 + b.idle_a) * 120 / 3600, rel=0.05)
    assert st.terminal_v < b.step(BatteryState(), 60.0, 0.001).terminal_v


def test_motor_voltage_is_capped_by_the_battery():
    b = Battery()
    st = b.step(BatteryState(charge=0.05), load_w=200.0, dt=0.001)
    assert b.limit_drive(12.0, st) == pytest.approx(st.terminal_v)
    assert b.limit_drive(-12.0, st) == pytest.approx(-st.terminal_v)
    assert b.limit_drive(5.0, st) == 5.0
    assert b.limit_drive(None, st) is None


def test_percent_the_brain_would_show():
    assert BatteryState(charge=1.0).percent == 100
    assert BatteryState(charge=0.42).percent == 42
