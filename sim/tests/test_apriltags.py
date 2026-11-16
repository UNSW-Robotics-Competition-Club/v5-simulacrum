"""Field AprilTags projected into the AI Vision image (apriltags.py)."""
import json
import math

import pytest

from v5sim.apriltags import Camera, FieldTag, load_tags, project
from v5sim.drivetrain import Pose

CAM = Camera(focal_px=212.3, width=320, height=240, height_m=0.25, ahead_m=0.0)


def test_tag_straight_ahead_is_centred_upright_bl_br_tr_tl():
    tag = FieldTag(id=5, x=1.0, y=0.0, facing_deg=180.0, z=0.25, size=0.1)
    c = project(tag, Pose(0.0, 0.0, 0.0), CAM)
    (blx, bly), (brx, bry), (trx, try_), (tlx, tly) = c
    assert blx < brx and bly > tly and abs(bly - bry) <= 1 and abs(blx - tlx) <= 1
    assert abs((blx + brx) / 2 - 160) <= 1 and abs((bly + tly) / 2 - 120) <= 1
    assert brx - blx == pytest.approx(212.3 * 0.1, abs=1.5)


def test_tag_facing_away_or_behind_or_cut_off_is_not_seen():
    away = FieldTag(id=1, x=1.0, y=0.0, facing_deg=0.0, z=0.25, size=0.1)
    assert project(away, Pose(0.0, 0.0, 0.0), CAM) is None
    behind = FieldTag(id=1, x=-1.0, y=0.0, facing_deg=0.0, z=0.25, size=0.1)
    assert project(behind, Pose(0.0, 0.0, 0.0), CAM) is None
    edge = FieldTag(id=1, x=0.5, y=0.38, facing_deg=180.0, z=0.25, size=0.1)
    assert project(edge, Pose(0.0, 0.0, 0.0), CAM) is None        # partly out of frame


def test_tag_left_of_the_robot_appears_left():
    tag = FieldTag(id=2, x=1.0, y=0.2, facing_deg=180.0, z=0.25, size=0.1)
    (blx, _), *_ = project(tag, Pose(0.0, 0.0, 0.0), CAM)
    assert blx < 160


def test_load_tags(tmp_path):
    f = tmp_path / "tags.json"
    f.write_text(json.dumps([{"id": 3, "x": 1.78, "y": 0, "facing": 180},
                             {"id": 4, "x": 0, "y": 1.78, "facing": 270, "z": 0.3,
                              "size": 0.15, "family": "tag36h11"}]))
    a, b = load_tags(str(f))
    assert (a.id, a.facing_deg, a.family) == (3, 180.0, 0)
    assert (b.z, b.size, b.family) == (0.3, 0.15, 3)


@pytest.mark.parametrize("row", [{"id": 1, "x": 0}, {"id": 1, "x": 0, "y": 0, "facing": "n"},
                                 {"id": 1, "x": math.inf, "y": 0, "facing": 0},
                                 {"id": 1, "x": 0, "y": 0, "facing": 0, "family": "qr"},
                                 {"id": -1, "x": 0, "y": 0, "facing": 0}])
def test_load_tags_rejects_bad_rows(tmp_path, row):
    f = tmp_path / "tags.json"
    f.write_text(json.dumps([row]))
    with pytest.raises(ValueError):
        load_tags(str(f))


def test_corners_stay_inside_the_image_after_rounding():
    for y in [i / 1000 for i in range(-400, 401, 3)]:
        c = project(FieldTag(id=1, x=1.0, y=y, facing_deg=180.0, z=0.25, size=0.1),
                    Pose(0.0, 0.0, 0.0), CAM)
        if c:
            assert all(0 <= u < 320 and 0 <= v < 240 for u, v in c)


@pytest.mark.parametrize("rows", [[1, 2], ["x"], [{"id": 3.7, "x": 0, "y": 0, "facing": 0}]])
def test_load_tags_rejects_non_objects_and_fractional_ids(tmp_path, rows):
    f = tmp_path / "tags.json"
    f.write_text(json.dumps(rows))
    with pytest.raises(ValueError):
        load_tags(str(f))
