"""AprilTags mounted on the field, as the AI Vision Sensor sees them.

A tags file is a JSON list of {"id", "x", "y", "facing"} (metres from the
field centre in the sim frame, facing = the direction the tag's face
points, degrees counter-clockwise from +x) with optional "z" (centre
height, m), "size" (black square edge, m) and "family" (circle21h7, the
sensor's default, tag16h5, tag25h9 or tag36h11). project() gives the four
image corners the sensor reports, bottom-left, bottom-right, top-right,
top-left of the upright tag (notes/vision-and-extras-protocol.md B.1).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .drivetrain import Pose

FAMILIES = ("circle21h7", "tag16h5", "tag25h9", "tag36h11")   # sensor index
TAG_Z_M = 0.20
TAG_SIZE_M = 0.1016             # 4 in
MIN_RANGE_M = 0.05

Corners = tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]]


@dataclass(frozen=True)
class Camera:
    focal_px: float
    width: int
    height: int
    height_m: float             # lens above the floor
    ahead_m: float              # lens ahead of the robot's centre


@dataclass(frozen=True)
class FieldTag:
    id: int
    x: float
    y: float
    facing_deg: float
    z: float = TAG_Z_M
    size: float = TAG_SIZE_M
    family: int = 0


def project(tag: FieldTag, pose: Pose, cam: Camera) -> Corners | None:
    """Image corners of a tag fully in view and facing the camera, else None."""
    c, s = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
    cx, cy = pose.x_m + cam.ahead_m * c, pose.y_m + cam.ahead_m * s
    nx, ny = math.cos(math.radians(tag.facing_deg)), math.sin(math.radians(tag.facing_deg))
    if (cx - tag.x) * nx + (cy - tag.y) * ny <= 0:
        return None                                   # seen from behind
    rx, ry = -ny, nx                                  # the viewer's right
    a = tag.size / 2
    corners = []
    for side, up in ((-1, -1), (1, -1), (1, 1), (-1, 1)):     # BL BR TR TL
        dx = tag.x + side * a * rx - cx
        dy = tag.y + side * a * ry - cy
        d, lateral = c * dx + s * dy, -s * dx + c * dy
        if d < MIN_RANGE_M:
            return None
        u = round(cam.width / 2 - cam.focal_px * lateral / d)
        v = round(cam.height / 2 - cam.focal_px * (tag.z + up * a - cam.height_m) / d)
        if not (0 <= u < cam.width and 0 <= v < cam.height):
            return None                               # the detector needs it all
        corners.append((u, v))
    return tuple(corners)


def load_tags(path: str) -> tuple[FieldTag, ...]:
    try:
        rows = json.loads(Path(path).read_text())
        if not isinstance(rows, list):
            raise TypeError("expected a list of tags")
        tags = []
        for row in rows:
            if not isinstance(row, dict):
                raise TypeError(f"tag {row!r}: expected an object")
            if not isinstance(row.get("id"), int) or isinstance(row["id"], bool):
                raise ValueError(f"tag {row}: id must be a whole number")
            family = row.get("family", FAMILIES[0])
            if family not in FAMILIES:
                raise ValueError(f"family must be one of {', '.join(FAMILIES)}")
            tag = FieldTag(int(row["id"]), float(row["x"]), float(row["y"]),
                           float(row["facing"]), float(row.get("z", TAG_Z_M)),
                           float(row.get("size", TAG_SIZE_M)), FAMILIES.index(family))
            if tag.id < 0 or not all(map(math.isfinite, (tag.x, tag.y, tag.facing_deg,
                                                         tag.z, tag.size))):
                raise ValueError(f"tag {row}: id must be >= 0 and values finite")
            if tag.size <= 0:
                raise ValueError(f"tag {tag.id}: size must be positive")
            tags.append(tag)
    except (OSError, KeyError, TypeError, ValueError, OverflowError) as e:
        raise ValueError(f"tags {path}: {e}") from e
    return tuple(tags)
