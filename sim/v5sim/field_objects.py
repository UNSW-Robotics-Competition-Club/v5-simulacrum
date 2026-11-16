"""Game objects on the field: blocks the robot plows, intakes and ejects.

Blocks (Push Back's 40 g blocks, about 9 cm across) are circles on the
floor. Each field step:

  - a block inside the robot's square footprint is pushed out the nearest
    face (plowing), unless
  - the intake is running inward and the block is at the front, within the
    intake's width: it is captured, up to the robot's capacity;
  - with the intake running outward, one held block is placed just in front
    of the robot every EJECT_PERIOD_S;
  - blocks stay inside the field walls.

Blocks don't push each other and don't slow the robot: this is for
practising autonomous routes and intake logic, not for pushing matches.
State is immutable; step_field returns a new Field.
"""
from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass, replace
from pathlib import Path

BLOCK_RADIUS_M = 0.045
INTAKE_MIN_RAD_S = 2.0          # intake shaft speed that counts as running
INTAKE_HALF_WIDTH_M = 0.12      # the intake's opening at the robot's front
EJECT_PERIOD_S = 0.25
COLORS = ("red", "blue")


@dataclass(frozen=True)
class Block:
    x: float
    y: float
    color: str


@dataclass(frozen=True)
class Field:
    blocks: tuple[Block, ...] = ()
    held: tuple[str, ...] = ()       # colours in intake order; last = newest
    eject_wait_s: float = 0.0


@dataclass(frozen=True)
class RobotBody:
    x: float
    y: float
    heading_rad: float               # 0 = +x, counter-clockwise
    half_m: float                    # half the side of the square footprint
    intake_rad_s: float = 0.0        # intake shaft speed, + = inward
    capacity: int = 0


def _to_robot(r: RobotBody, x: float, y: float) -> tuple[float, float]:
    c, s = math.cos(r.heading_rad), math.sin(r.heading_rad)
    dx, dy = x - r.x, y - r.y
    return c * dx + s * dy, -s * dx + c * dy


def _to_field(r: RobotBody, lx: float, ly: float) -> tuple[float, float]:
    c, s = math.cos(r.heading_rad), math.sin(r.heading_rad)
    return r.x + c * lx - s * ly, r.y + s * lx + c * ly


def _inside_walls(b: Block, field_half_m: float) -> Block:
    lim = field_half_m - BLOCK_RADIUS_M
    x, y = max(-lim, min(lim, b.x)), max(-lim, min(lim, b.y))
    return b if (x, y) == (b.x, b.y) else replace(b, x=x, y=y)


def _plow(r: RobotBody, b: Block, lx: float, ly: float) -> Block:
    reach = r.half_m + BLOCK_RADIUS_M
    if reach - abs(lx) <= reach - abs(ly):     # nearer the front/back face
        lx = math.copysign(reach, lx)
    else:
        ly = math.copysign(reach, ly)
    x, y = _to_field(r, lx, ly)
    return replace(b, x=x, y=y)


def step_field(f: Field, r: RobotBody, dt: float, field_half_m: float) -> Field:
    reach = r.half_m + BLOCK_RADIUS_M
    intaking = r.intake_rad_s > INTAKE_MIN_RAD_S
    held = list(f.held)
    blocks = []
    for b in f.blocks:
        lx, ly = _to_robot(r, b.x, b.y)
        if abs(lx) < reach and abs(ly) < reach:
            if (intaking and lx > 0 and abs(ly) < INTAKE_HALF_WIDTH_M
                    and len(held) < r.capacity):
                held.append(b.color)
                continue
            b = _plow(r, b, lx, ly)
        blocks.append(_inside_walls(b, field_half_m))

    wait = max(f.eject_wait_s - dt, 0.0)
    if r.intake_rad_s < -INTAKE_MIN_RAD_S and held and wait == 0.0:
        x, y = _to_field(r, reach + 0.01, 0.0)
        b = _inside_walls(Block(x, y, held[-1]), field_half_m)
        lx, ly = _to_robot(r, b.x, b.y)
        if abs(lx) >= reach or abs(ly) >= reach:   # no room against a wall: keep it
            held.pop()
            blocks.append(b)
            wait = EJECT_PERIOD_S
    out = Field(tuple(blocks), tuple(held), wait)
    return f if out == f else out


def layout(spec: str, field_half_m: float) -> Field:
    """`random:N[:SEED]` (alternating colours, uniform) or a JSON file of
    [x, y, colour] in metres from the field centre."""
    if spec.startswith("random:"):
        parts = spec.split(":")[1:]
        try:
            n, seed = int(parts[0]), int(parts[1]) if len(parts) > 1 else 0
        except (IndexError, ValueError) as e:
            raise ValueError("expected random:N[:SEED]") from e
        if n < 0 or len(parts) > 2:
            raise ValueError("expected random:N[:SEED] with N >= 0")
        rng = random.Random(seed)
        lim = field_half_m - 2 * BLOCK_RADIUS_M
        return Field(tuple(Block(rng.uniform(-lim, lim), rng.uniform(-lim, lim),
                                 COLORS[i % 2]) for i in range(n)))
    path = Path(spec)
    if path.suffix != ".json":
        raise ValueError("blocks: random:N[:SEED] or a .json file")
    try:
        rows = json.loads(path.read_text())
    except OSError as e:
        raise ValueError(f"blocks: {e}") from e
    if not isinstance(rows, list):
        raise ValueError("blocks: expected a list of [x, y, colour]")
    blocks = []
    for row in rows:
        try:
            x, y, color = row
            x, y = float(x), float(y)
        except (TypeError, ValueError, OverflowError) as e:
            raise ValueError(f"blocks: expected [x, y, colour], got {row!r}") from e
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"blocks: position must be finite, got {row!r}")
        if color not in COLORS:
            raise ValueError(f"blocks: colour must be red or blue, not {color!r}")
        blocks.append(Block(x, y, color))
    return Field(tuple(blocks))
