"""--obstacles / --opponent: parsing, validation, and the robot's world."""
import json
import math

import pytest

from v5sim.field_elements import Obstacle, load_obstacles
from v5sim.robot import parser


def test_obstacles_file_loads(tmp_path):
    f = tmp_path / "o.json"
    f.write_text(json.dumps([{"kind": "box", "x": 0.6, "y": 0, "sx": 0.05, "sy": 0.5, "angle": 30},
                             {"kind": "cylinder", "x": -0.6, "y": 0.6, "r": 0.08}]))
    box, post = load_obstacles(str(f))
    assert box == Obstacle("box", 0.6, 0.0, 0.05, 0.5, 0.0, 30.0)
    assert post.kind == "cylinder" and post.r == 0.08
    assert box.view()[-1] == pytest.approx(math.radians(30))


@pytest.mark.parametrize("rows", [{"kind": "box"}, [{"kind": "wall", "x": 0, "y": 0}],
                                  [{"kind": "box", "x": 0, "y": 0, "sx": 0, "sy": 1}],
                                  [{"kind": "cylinder", "x": 0, "y": 0}], [{"x": 0}],
                                  [{"kind": "box", "x": "nan", "y": 0, "sx": 1, "sy": 1}]])
def test_bad_obstacles_are_refused(tmp_path, rows):
    f = tmp_path / "o.json"
    f.write_text(json.dumps(rows))
    with pytest.raises(ValueError, match="obstacles"):
        load_obstacles(str(f))


def test_opponent_option_places_it_like_start():
    args = parser().parse_args(["--motor", "1:green", "--opponent", "0:24:90:braked",
                                "--opponent", "12:0:0"])
    braked, coasting = args.opponent
    assert braked.braked and not coasting.braked
    assert braked.pose.x_m == pytest.approx(24 * 0.0254)        # RW +Y is sim +x
    assert braked.pose.heading_rad == pytest.approx(-math.pi / 2)
    assert coasting.pose.y_m == pytest.approx(-12 * 0.0254)


def test_opponent_option_rejects_junk():
    with pytest.raises(SystemExit):
        parser().parse_args(["--opponent", "1:2:braked"])


def test_field_elements_need_mujoco(capsys):
    from v5sim.robot import main
    with pytest.raises(SystemExit):
        main(["--drive", "1,2:3,4:blue", "--opponent", "0:0:0"])
    assert "--physics mujoco" in capsys.readouterr().err
