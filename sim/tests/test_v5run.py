"""scripts/v5run.py: the run-and-score harness's reporting and checks (the
runs themselves need a brain: scripts/v5run_example.json)."""
import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "v5run.py"
spec = importlib.util.spec_from_file_location("v5run", SCRIPT)
v5run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v5run)


def state(x=0.0, y=0.0, h=0.0, score=None):
    return {"t": 10.0, "x": x, "y": y, "heading": h, "v": 0.0, "trail": [[x, y]],
            "blocks": [], "held": [], "score": score, "opponents": [[1.0, 0.0, math.pi]]}


def test_reports_the_pose_in_rw_coordinates():
    r = v5run.summarise(state(x=24 * 0.0254, y=-12 * 0.0254, h=-math.pi / 2), None, 5.0, "time")
    assert r["rw"] == {"x": 12.0, "y": 24.0, "heading": 90.0}
    assert r["opponents"][0]["heading"] == 180.0
    assert r["path"] == [[12.0, 24.0]]


def test_motors_and_battery_from_the_dashboard():
    devices = {"devices": [
        {"port": 0, "kind": "battery", "volts": 12.9, "amps": 3.0, "percent": 88},
        {"port": 1, "kind": "motor", "position_deg": 10.0, "velocity_rpm": 0.0,
         "current_a": 0.1, "temperature_c": 31.0, "faults": 0}]}
    r = v5run.summarise(state(), devices, 5.0, "settled")
    assert r["motors"]["1"]["temperature_c"] == 31.0 and r["battery"]["percent"] == 88


def test_expect_passes_within_tolerance_and_wraps_heading():
    r = v5run.summarise(state(x=24 * 0.0254, h=math.radians(1)), None, 5.0, "time")
    assert v5run.check(r, {"expect": "0:23:358", "tol": "2:5"}) == []
    assert r["pose_error"]["deg"] == pytest.approx(1.0, abs=0.01)     # 359 vs 358


def test_expect_and_score_failures_are_reported():
    r = v5run.summarise(state(score={"red": 3, "blue": 6}), None, 5.0, "time")
    problems = v5run.check(r, {"expect": "10:0:0", "min_score": 10})
    assert len(problems) == 2 and "score 6 < 10" in problems[1]


def test_robot_args_resolve_files_against_the_batch(tmp_path):
    (tmp_path / "blocks.json").write_text("[]")
    (tmp_path / "bot.args").write_text("--drive 1:2:blue")
    args = v5run.robot_args({"robot": "--drive 1:2:blue --blocks blocks.json"}, tmp_path)
    assert str(tmp_path / "blocks.json") in args and "--drive 1:2:blue" in args
    assert v5run.robot_args({"preset": "bot.args"}, tmp_path) == "@" + str(tmp_path / "bot.args")
    assert v5run.robot_args({}, tmp_path) == v5run.DEFAULT_ROBOT


def test_batch_needs_distinct_names(tmp_path):
    f = tmp_path / "runs.json"
    f.write_text('[{"program": "a.bin"}, {"program": "a.bin"}]')
    with pytest.raises(SystemExit):
        v5run.main(["--batch", str(f)])
