import pytest

from v5sim.field_objects import Block, Field
from v5sim.scoring import Goal, load_goals, score

LONG = Goal("long red", (-1.6, -0.7, -1.4, -0.5), block_points=3, control_points=10)
CENTER = Goal("center", (-0.2, -0.2, 0.2, 0.2), block_points=3, control_points=8)


def test_blocks_score_in_goals_and_majority_controls():
    f = Field((Block(-1.5, -0.6, "red"), Block(-1.5, -0.55, "red"),
               Block(-1.45, -0.6, "blue"), Block(0.0, 0.0, "blue"),
               Block(1.0, 1.0, "red")))                  # outside any goal
    s = score(f, (LONG, CENTER))
    assert s.goals["long red"] == {"red": 2, "blue": 1}
    assert s.goals["center"] == {"red": 0, "blue": 1}
    assert s.red == 2 * 3 + 10 and s.blue == 2 * 3 + 8


def test_tie_controls_nothing():
    f = Field((Block(0.0, 0.0, "red"), Block(0.1, 0.0, "blue")))
    s = score(f, (CENTER,))
    assert (s.red, s.blue) == (3, 3)


def test_held_blocks_dont_score():
    s = score(Field((), held=("red",)), (CENTER,))
    assert (s.red, s.blue) == (0, 0)


def test_load_goals_from_json(tmp_path):
    p = tmp_path / "g.json"
    p.write_text('[{"name": "c", "rect": [-0.2, -0.2, 0.2, 0.2], "control_points": 8}]')
    assert load_goals(str(p)) == (Goal("c", (-0.2, -0.2, 0.2, 0.2), 3, 8),)


@pytest.mark.parametrize("text", ['{}', '[{"name": "c"}]', '[{"name": "c", "rect": [1, 2, 3]}]',
                                  '[{"name": "c", "rect": [0, 0, -1, 1]}]'])
def test_load_goals_rejects_bad_files(tmp_path, text):
    p = tmp_path / "g.json"
    p.write_text(text)
    with pytest.raises(ValueError):
        load_goals(str(p))


@pytest.mark.parametrize("text", [
    '[{"name": "a", "rect": [0, 0, 1, 1]}, {"name": "a", "rect": [2, 2, 3, 3]}]',   # duplicate
    '[{"name": "a", "rect": [0, 0, 1, 1]}, {"name": "b", "rect": [0.5, 0.5, 2, 2]}]',  # overlap
    '[{"name": "a", "rect": [0, 0, 1e400, 1]}]',
    '[{"name": "a", "rect": [0, 0, 1, 1], "control_points": Infinity}]',
])
def test_load_goals_rejects_ambiguous_or_huge(tmp_path, text):
    p = tmp_path / "g.json"
    p.write_text(text)
    with pytest.raises(ValueError):
        load_goals(str(p))


def test_goals_sharing_an_edge_count_a_block_once():
    a = Goal("a", (0.0, 0.0, 1.0, 1.0))
    b = Goal("b", (1.0, 0.0, 2.0, 1.0))
    s = score(Field((Block(1.0, 0.5, "red"),)), (a, b))
    assert s.red == 3
