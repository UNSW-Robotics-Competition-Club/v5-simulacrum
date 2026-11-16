"""Fixed field elements and opponent robots (--obstacles, --opponent), for
the MuJoCo physics (physics_mujoco.py) and the field view.

An obstacles file is a JSON list in the sim frame (metres from the field
centre, +x where the robot faces at heading 0), like goals and blocks:

    [{"kind": "box", "x": 0.6, "y": 0.0, "sx": 0.05, "sy": 0.5, "angle": 0},
     {"kind": "cylinder", "x": -0.6, "y": 0.6, "r": 0.08}]

sx, sy are half sizes; angle turns a box, degrees anticlockwise.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .drivetrain import Pose

OPPONENT_MASS_KG = 7.0
OPPONENT_HALF_M = 9 * 0.0254


@dataclass(frozen=True)
class Obstacle:
    """A static field element: kind "box" (half extents sx, sy, turned
    angle_deg) or "cylinder" (radius r), centred at (x, y), sim frame m."""
    kind: str
    x: float
    y: float
    sx: float = 0.0
    sy: float = 0.0
    r: float = 0.0
    angle_deg: float = 0.0

    def __post_init__(self) -> None:
        if self.kind not in ("box", "cylinder"):
            raise ValueError(f"obstacle kind {self.kind!r}: box or cylinder")
        if not all(map(math.isfinite, (self.x, self.y, self.sx, self.sy, self.r,
                                       self.angle_deg))):
            raise ValueError("obstacle numbers must be finite")
        if self.kind == "box" and min(self.sx, self.sy) <= 0:
            raise ValueError("a box obstacle needs half-sizes sx, sy > 0")
        if self.kind == "cylinder" and self.r <= 0:
            raise ValueError("a cylinder obstacle needs a radius r > 0")

    def view(self) -> list:
        """For the field view: [kind, x, y, sx, sy, r, angle rad]."""
        return [self.kind, self.x, self.y, self.sx, self.sy, self.r,
                math.radians(self.angle_deg)]


@dataclass(frozen=True)
class Opponent:
    """Another robot: braked (holds until pushed past its friction) or
    coasting (rolls along its heading, grips sideways)."""
    pose: Pose
    braked: bool = False
    mass_kg: float = OPPONENT_MASS_KG
    half_m: float = OPPONENT_HALF_M


def load_obstacles(path: str) -> tuple[Obstacle, ...]:
    try:
        rows = json.loads(Path(path).read_text())
        if not isinstance(rows, list):
            raise TypeError("expected a list of obstacles")
        return tuple(Obstacle(str(r["kind"]), float(r["x"]), float(r["y"]),
                              float(r.get("sx", 0)), float(r.get("sy", 0)),
                              float(r.get("r", 0)), float(r.get("angle", 0)))
                     for r in rows)
    except (OSError, KeyError, TypeError, ValueError, AttributeError) as e:
        raise ValueError(f"obstacles {path}: {e}") from e
