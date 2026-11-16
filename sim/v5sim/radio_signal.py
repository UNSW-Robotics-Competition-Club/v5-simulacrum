"""A VEXnet link's signal: RSSI and link quality as the V5 radio reports them.

The radio's 0x80 link-status frame carries RSSI (byte 3, signed dBm) and link
quality (bytes 8..11, u32 LE, percent: DEVFRM08 sets 100 at link-up, then
received / expected frames per window). The controller copies both into its
0xA7 status for a field (byte 1 RSSI, byte 29 "Signal"); notes/field-control-
protocol.md section 12.

The model: RSSI wanders slowly around a field-typical level with a little
fast jitter; quality stays near 100 and dips now and then, more often when
the signal is weak. Seeded, so runs repeat.
"""
from __future__ import annotations

import random
from dataclasses import dataclass

RSSI_CENTRE_DBM = -55.0          # a robot on the field, a few metres away
RSSI_SPREAD_DBM = 6.0            # slow drift around the centre
RSSI_JITTER_DBM = 1.5            # per-update noise
RSSI_MIN_DBM, RSSI_MAX_DBM = -90, -30
DRIFT_PULL = 0.05                # how fast the drift returns to the centre
UPDATE_S = 0.5                   # how often the radio refreshes the numbers
WEAK_DBM = -75.0                 # below this, frames get lost more often
QUALITY_FULL = 100

COMP_CHANNEL = 0x51              # 81: in the app's competition table 54..81
COMP_MODE = 0x03                 # shown as "/4"
LS_LINKED = 0x0005               # 80 bytes 0x16..0x17: any link + linked


@dataclass(frozen=True)
class SignalSample:
    rssi_dbm: int = int(RSSI_CENTRE_DBM)
    quality: int = QUALITY_FULL      # percent
    link_status: int = LS_LINKED
    channel: int = COMP_CHANNEL
    mode: int = COMP_MODE


class RadioSignal:
    def __init__(self, seed: int = 5, centre_dbm: float = RSSI_CENTRE_DBM) -> None:
        self._rng = random.Random(seed)
        self._centre = centre_dbm
        self._drift = 0.0
        self._next = 0.0
        self.sample = SignalSample()

    def update(self, now: float) -> SignalSample:
        """The current reading, refreshed every UPDATE_S."""
        if now < self._next:
            return self.sample
        self._next = now + UPDATE_S
        rng = self._rng
        self._drift += rng.gauss(0.0, RSSI_SPREAD_DBM * 0.3) - DRIFT_PULL * self._drift
        self._drift = max(-2 * RSSI_SPREAD_DBM, min(2 * RSSI_SPREAD_DBM, self._drift))
        rssi = self._centre + self._drift + rng.gauss(0.0, RSSI_JITTER_DBM)
        rssi = max(RSSI_MIN_DBM, min(RSSI_MAX_DBM, round(rssi)))
        loss_chance = 0.1 + (0.5 if rssi < WEAK_DBM else 0.0)
        quality = QUALITY_FULL
        if rng.random() < loss_chance:
            quality -= rng.randint(1, 8 if rssi >= WEAK_DBM else 25)
        self.sample = SignalSample(rssi_dbm=rssi, quality=quality)
        return self.sample
