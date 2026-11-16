"""A timed competition match driven through the tethered controller.

The brain's own field-control state machine only runs over the VEXnet radio
(controller-protocol.md 4.3), but programs see the competition state through
byte 6 of the controller's A0 frame: that is what VEXcode's `competition`
callbacks and PROS's competition template poll. So a match is a schedule of
byte-6 values: disabled until the field starts, autonomous, a short disabled
pause while the field resets, driver control, disabled again.
"""
from __future__ import annotations

from dataclasses import dataclass

from .smart_controller import Competition

AUTON_S = 15.0         # VRC match periods
DRIVER_S = 105.0
PAUSE_S = 2.0          # disabled between autonomous and driver control


@dataclass(frozen=True)
class MatchSchedule:
    start_s: float            # sim time the field starts the match
    auton_s: float = AUTON_S  # 0 for a driver-only (skills) run
    driver_s: float = DRIVER_S

    @classmethod
    def parse(cls, text: str) -> "MatchSchedule":
        """`START[:AUTON:DRIVER]` in seconds, e.g. `25` or `25:60:0`."""
        parts = [float(p) for p in text.split(":")]
        if len(parts) not in (1, 3):
            raise ValueError("expected START or START:AUTON:DRIVER")
        m = cls(*parts)
        if min(parts) < 0 or m.auton_s + m.driver_s <= 0:
            raise ValueError("times must be >= 0 with a non-empty match")
        return m

    @property
    def _driver_start_s(self) -> float:
        pause = PAUSE_S if self.auton_s and self.driver_s else 0.0
        return self.start_s + self.auton_s + pause

    @property
    def end_s(self) -> float:
        return self._driver_start_s + self.driver_s

    def phase_at(self, t: float) -> Competition:
        if self.start_s <= t < self.start_s + self.auton_s:
            return Competition.AUTONOMOUS
        if self._driver_start_s <= t < self.end_s:
            return Competition.DRIVER
        return Competition.DISABLED
