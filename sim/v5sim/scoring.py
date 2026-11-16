"""Goals and a live score for the game objects (field_objects.py).

Goals come from a JSON file, not from code, because exact field geometry
differs between games and field setups:

  [{"name": "long red", "rect": [x0, y0, x1, y1],
    "block_points": 3, "control_points": 10}, ...]

rect is in metres from the field centre (x0 < x1, y0 < y1). A block whose
centre is inside a goal's rect scores block_points for its colour; the
colour with more blocks in the goal also gets control_points (a tie gets
nothing), the shape of Push Back's scoring. Robots drive over goals: they
are zones on the floor here, not obstacles.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .field_objects import COLORS, Field

BLOCK_POINTS = 3


@dataclass(frozen=True)
class Goal:
    name: str
    rect: tuple[float, float, float, float]
    block_points: int = BLOCK_POINTS
    control_points: int = 0

    def contains(self, x: float, y: float) -> bool:
        # half-open, so goals that share an edge never both own a block
        x0, y0, x1, y1 = self.rect
        return x0 <= x < x1 and y0 <= y < y1

    def overlaps(self, other: "Goal") -> bool:
        a, b = self.rect, other.rect
        return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


@dataclass(frozen=True)
class Score:
    red: int
    blue: int
    goals: dict[str, dict[str, int]]     # goal name -> colour -> blocks


def score(field: Field, goals: tuple[Goal, ...]) -> Score:
    totals = dict.fromkeys(COLORS, 0)
    per_goal = {}
    for g in goals:
        counts = dict.fromkeys(COLORS, 0)
        for b in field.blocks:
            if g.contains(b.x, b.y):
                counts[b.color] += 1
        for c in COLORS:
            totals[c] += counts[c] * g.block_points
        red, blue = counts["red"], counts["blue"]
        if red != blue:
            totals["red" if red > blue else "blue"] += g.control_points
        per_goal[g.name] = counts
    return Score(totals["red"], totals["blue"], per_goal)


def load_goals(path: str) -> tuple[Goal, ...]:
    try:
        rows = json.loads(Path(path).read_text())
        if not isinstance(rows, list):
            raise TypeError("expected a list of goals")
        goals: list[Goal] = []
        for row in rows:
            x0, y0, x1, y1 = (float(v) for v in row["rect"])
            if not all(map(math.isfinite, (x0, y0, x1, y1))) or not (x0 < x1 and y0 < y1):
                raise ValueError(f"goal {row['name']!r}: rect must be finite, x0 < x1, y0 < y1")
            g = Goal(str(row["name"]), (x0, y0, x1, y1),
                     int(row.get("block_points", BLOCK_POINTS)),
                     int(row.get("control_points", 0)))
            for other in goals:
                if other.name == g.name:
                    raise ValueError(f"two goals named {g.name!r}")
                if other.overlaps(g):
                    raise ValueError(f"goals {other.name!r} and {g.name!r} overlap")
            goals.append(g)
    except (OSError, KeyError, TypeError, ValueError, OverflowError) as e:
        raise ValueError(f"goals {path}: {e}") from e
    return tuple(goals)
