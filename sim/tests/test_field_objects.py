import math

import pytest

from v5sim.field_objects import (BLOCK_RADIUS_M, EJECT_PERIOD_S, Block, Field,
                                 RobotBody, layout, step_field)

HALF = 0.2286            # 18 in robot
FIELD = 1.8288           # 12 ft field half side


def body(x=0.0, y=0.0, h=0.0, intake=0.0, capacity=3):
    return RobotBody(x, y, h, HALF, intake, capacity)


def test_robot_plows_a_block_out_of_its_footprint():
    f = Field((Block(HALF, 0.0, "red"),))            # block centre on the front face
    out = step_field(f, body(), 0.01, FIELD)
    b = out.blocks[0]
    assert b.x == pytest.approx(HALF + BLOCK_RADIUS_M) and b.y == pytest.approx(0)
    assert out.held == ()


def test_plowing_works_in_the_robot_frame():
    f = Field((Block(0.0, HALF, "blue"),))            # robot faces +y: block in front
    b = step_field(f, body(h=math.pi / 2), 0.01, FIELD).blocks[0]
    assert b.y == pytest.approx(HALF + BLOCK_RADIUS_M) and b.x == pytest.approx(0, abs=1e-9)


def test_side_contact_pushes_sideways():
    f = Field((Block(0.0, -HALF, "red"),))
    b = step_field(f, body(), 0.01, FIELD).blocks[0]
    assert b.y == pytest.approx(-(HALF + BLOCK_RADIUS_M)) and b.x == pytest.approx(0)


def test_far_blocks_are_untouched():
    f = Field((Block(1.0, 1.0, "red"),))
    assert step_field(f, body(), 0.01, FIELD) == f


def test_running_intake_captures_front_blocks_up_to_capacity():
    f = Field(tuple(Block(HALF, y, c) for y, c in
                    ((0.0, "red"), (0.05, "blue"), (-0.05, "red"), (0.02, "blue"))))
    out = step_field(f, body(intake=20.0, capacity=3), 0.01, FIELD)
    assert out.held == ("red", "blue", "red")
    assert len(out.blocks) == 1                       # the fourth is plowed


def test_intake_does_not_capture_from_behind():
    f = Field((Block(-HALF, 0.0, "red"),))
    out = step_field(f, body(intake=20.0), 0.01, FIELD)
    assert out.held == () and len(out.blocks) == 1


def test_reverse_intake_ejects_one_block_at_a_time_in_front():
    f = Field((), held=("red", "blue"))
    out = step_field(f, body(h=math.pi, intake=-20.0), 0.01, FIELD)
    assert out.held == ("red",)
    b = out.blocks[0]
    assert b.color == "blue" and b.x < -HALF and b.y == pytest.approx(0, abs=1e-9)
    again = step_field(out, body(h=math.pi, intake=-20.0), 0.01, FIELD)
    assert again.held == ("red",)                     # still within the period
    later = step_field(again, body(h=math.pi, intake=-20.0), EJECT_PERIOD_S, FIELD)
    assert later.held == ()


def test_blocks_stay_inside_the_walls():
    f = Field((Block(FIELD - 0.01, 0.0, "red"),))
    b = step_field(f, body(x=FIELD - HALF - 0.05), 0.01, FIELD).blocks[0]
    assert b.x <= FIELD - BLOCK_RADIUS_M + 1e-9


def test_layout_random_is_seeded_and_balanced():
    a = layout("random:10:7", FIELD)
    assert a == layout("random:10:7", FIELD) and len(a.blocks) == 10
    assert sum(b.color == "red" for b in a.blocks) == 5
    assert all(abs(b.x) < FIELD and abs(b.y) < FIELD for b in a.blocks)


def test_layout_from_json(tmp_path):
    p = tmp_path / "f.json"
    p.write_text('[[0.5, -0.5, "blue"], [1, 1, "red"]]')
    assert layout(str(p), FIELD).blocks == (Block(0.5, -0.5, "blue"), Block(1.0, 1.0, "red"))


@pytest.mark.parametrize("spec", ["random:x", "random:-1", "nofile.json", "triangle"])
def test_layout_rejects_bad_specs(spec):
    with pytest.raises(ValueError):
        layout(spec, FIELD)


def test_eject_against_a_wall_keeps_the_block():
    # front face at the wall: no room in front, so nothing comes out
    r = body(x=FIELD - HALF, intake=-20.0)
    out = step_field(Field((), held=("red",)), r, 0.01, FIELD)
    assert out.held == ("red",) and out.blocks == ()


@pytest.mark.parametrize("text", ['[[1, 2]]', '[[null, 0, "red"]]', '[5]',
                                  '[["x", 0, "red"]]', '[[NaN, 0, "red"]]'])
def test_layout_rejects_malformed_rows(tmp_path, text):
    p = tmp_path / "f.json"
    p.write_text(text)
    with pytest.raises(ValueError):
        layout(str(p), FIELD)


@pytest.mark.parametrize("text", ['5', 'null', '[[1e400, 0, "red"]]'])
def test_layout_rejects_non_lists_and_overflow(tmp_path, text):
    p = tmp_path / "f.json"
    p.write_text(text)
    with pytest.raises(ValueError):
        layout(str(p), FIELD)
