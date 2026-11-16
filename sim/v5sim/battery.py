"""The V5 robot battery: voltage sag under load and drain over a match.

A 4-cell LiFePO4 pack (VEX: 12.8 V nominal, 1100 mAh). Its open-circuit
voltage follows the state of charge, flat through the middle and dropping
at the ends; the terminal voltage is that minus the current through the
pack's internal resistance (cells, fuse, wiring). The current is what the
motors draw (their electrical power over the terminal voltage) plus the
brain's own draw. A motor can't apply more than the terminal voltage, so a
robot pulling hard, or one near empty, gets weaker, as on a real field.
[I: the curve and resistance are typical LiFePO4 figures, not measured on a
V5 pack.]
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from .motor import Drive

CAPACITY_AH = 1.1
R_INTERNAL_OHM = 0.12
IDLE_A = 0.35                       # brain, radio, sensors
# (state of charge, open-circuit volts), 4 cells in series
OCV_CURVE = ((0.0, 11.0), (0.05, 12.2), (0.1, 12.8), (0.2, 13.1), (0.5, 13.25),
             (0.8, 13.35), (0.95, 13.6), (1.0, 13.9))


def open_circuit_v(charge: float) -> float:
    charge = max(0.0, min(1.0, charge))
    for (c0, v0), (c1, v1) in zip(OCV_CURVE, OCV_CURVE[1:]):
        if charge <= c1:
            return v0 + (v1 - v0) * (charge - c0) / (c1 - c0)
    return OCV_CURVE[-1][1]


@dataclass(frozen=True)
class BatteryState:
    charge: float = 1.0             # 0..1
    terminal_v: float = OCV_CURVE[-1][1]
    current_a: float = 0.0

    @property
    def percent(self) -> int:
        return round(self.charge * 100)


@dataclass(frozen=True)
class Battery:
    capacity_ah: float = CAPACITY_AH
    r_ohm: float = R_INTERNAL_OHM
    idle_a: float = IDLE_A

    def step(self, st: BatteryState, load_w: float, dt: float) -> BatteryState:
        """Advance dt with the motors drawing load_w watts."""
        ocv = open_circuit_v(st.charge)
        # terminal V solves V = ocv - R (P / V + idle): the larger root
        b = ocv - self.r_ohm * self.idle_a
        disc = b * b - 4 * self.r_ohm * max(load_w, 0.0)
        v = (b + disc ** 0.5) / 2 if disc > 0 else b / 2     # past the power limit
        current = max(load_w, 0.0) / v + self.idle_a
        charge = max(0.0, st.charge - current * dt / 3600 / self.capacity_ah)
        return replace(st, charge=charge, terminal_v=v, current_a=current)

    @staticmethod
    def limit_drive(drive: Drive, st: BatteryState) -> Drive:
        """A motor's applied voltage, capped by what the battery gives."""
        if drive is None:
            return None
        return max(-st.terminal_v, min(st.terminal_v, drive))
