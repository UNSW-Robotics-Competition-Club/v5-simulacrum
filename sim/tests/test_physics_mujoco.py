"""The MuJoCo physics backend (physics_mujoco.py) against the builtin
drivetrain model, plus what only it does: block contact."""
import math
from dataclasses import replace

import pytest

pytest.importorskip("mujoco")

from v5sim import drivetrain  # noqa: E402
from v5sim.drivetrain import INCH, DrivetrainSpec, Pose  # noqa: E402
from v5sim.field_objects import BLOCK_RADIUS_M, Block, Field  # noqa: E402
from v5sim.physics_mujoco import MujocoWorld  # noqa: E402

HALF = 70.25 * INCH
ROBOT_HALF = 9 * INCH
DT = 0.001
SPEC = DrivetrainSpec()


def world(pose=Pose(), field=None, spec=SPEC):
    return MujocoWorld(spec, HALF, ROBOT_HALF, pose, field)


def run(w, left_v, right_v, seconds, spec=SPEC, **kw):
    n = spec.motors_per_side
    st = None
    for _ in range(round(seconds / DT)):
        st = w.step([left_v] * n, [right_v] * n, DT, **kw)
    return st


def _with_wheel_inertia(spec):
    """The builtin model ignores the wheels' spin inertia while they roll;
    MuJoCo spins them. Fold it into mass and yaw inertia to compare."""
    extra = 2 * spec.wheel_spin_inertia_kg_m2 / spec.wheel_radius_m ** 2
    m = spec.mass_kg + extra
    return DrivetrainSpec(
        mass_kg=m, rolling_resistance=spec.rolling_resistance * spec.mass_kg / m,
        drag_n_per_m_s=spec.drag_n_per_m_s,
        yaw_inertia_kg_m2=spec.yaw_inertia_kg_m2 + extra * (spec.track_width_m / 2) ** 2)


REF = _with_wheel_inertia(SPEC)


def builtin(left_v, right_v, seconds, pose=Pose(), state=None):
    st = state or drivetrain.initial_state(REF, pose)
    return drivetrain.simulate(REF, st, left_v, right_v, seconds, DT)


def test_straight_line_top_speed_matches_the_builtin_model():
    ref = builtin(12.0, 12.0, 1.5, Pose(-1.2, 0.0, 0.0))
    st = run(world(Pose(-1.2, 0.0, 0.0)), 12.0, 12.0, 1.5)
    assert st.v_m_s == pytest.approx(ref.v_m_s, rel=0.05)
    assert abs(st.pose.y_m) < 0.005 and abs(st.pose.heading_rad) < 0.01


def test_distance_matches_the_builtin_model_below_the_traction_limit():
    ref = builtin(6.0, 6.0, 1.5, Pose(-1.2, 0.0, 0.0))
    st = run(world(Pose(-1.2, 0.0, 0.0)), 6.0, 6.0, 1.5)
    assert st.pose.x_m == pytest.approx(ref.pose.x_m, abs=0.03)


def test_full_power_launch_is_traction_limited():
    """Six blue motors out-push the tiles: the wheels spin up at launch, so
    the robot gains no more than mu * g (the builtin model has no slip)."""
    st = run(world(Pose(-1.2, 0.0, 0.0)), 12.0, 12.0, 0.1)
    assert st.v_m_s / 0.1 <= SPEC.traction_mu * 9.81 * 1.02
    assert st.left_slip > 0


def test_turn_in_place_rate_matches_the_builtin_model():
    ref = builtin(-6.0, 6.0, 1.0)
    st = run(world(), -6.0, 6.0, 1.0)
    assert st.omega_rad_s == pytest.approx(ref.omega_rad_s, rel=0.10)
    assert math.hypot(st.pose.x_m, st.pose.y_m) < 0.01


def test_motor_states_follow_the_wheels():
    st = run(world(Pose(-1.2, 0.0, 0.0)), 12.0, 12.0, 1.0)
    wheel_motor = st.v_m_s / SPEC.wheel_radius_m * SPEC.external_ratio
    for m in st.left + st.right:
        assert m.velocity_rad_s == pytest.approx(wheel_motor, rel=0.03)
        assert m.voltage_v == 12.0 and m.current_a > 0
    assert st.left[0].position_rad > 0


def test_coasting_slows_down_like_the_builtin_model():
    w = world(Pose(-1.2, 0.0, 0.0))
    fast = run(w, 12.0, 12.0, 0.8)
    slow = run(w, None, None, 0.5)
    ref = builtin(None, None, 0.5, state=replace(fast, left_slip=0.0, right_slip=0.0))
    assert slow.v_m_s < fast.v_m_s
    assert fast.v_m_s - slow.v_m_s == pytest.approx(fast.v_m_s - ref.v_m_s, rel=0.05)


def test_wall_stops_the_robot_and_hard_pushing_spins_the_wheels():
    w = world(Pose(1.2, 0.0, 0.0))
    st = run(w, 12.0, 12.0, 2.0)
    assert st.pose.x_m == pytest.approx(HALF - ROBOT_HALF, abs=0.01)
    assert abs(st.v_m_s) < 0.02
    free = SPEC.cartridge.free_rpm * 2 * math.pi / 60
    assert st.left[0].velocity_rad_s > 0.15 * free            # slipping
    gentle = run(world(Pose(1.2, 0.0, 0.0)), 3.0, 3.0, 2.0)
    assert abs(gentle.left[0].velocity_rad_s) < 0.05 * free   # stalled, gripping


def test_robot_pushes_a_block_ahead():
    field = Field((Block(0.0, 0.0, "red"),))
    w = world(Pose(-0.6, 0.0, 0.0), field)
    run(w, 6.0, 6.0, 1.2)
    (b,) = w.field().blocks
    assert b.x > 0.1
    assert abs(b.x - w.pose().x_m) >= ROBOT_HALF + BLOCK_RADIUS_M - 0.01


def test_blocks_push_each_other_and_never_overlap():
    field = Field((Block(0.0, 0.0, "red"), Block(0.095, 0.0, "blue")))
    w = world(Pose(-0.6, 0.0, 0.0), field)
    run(w, 6.0, 6.0, 1.5)
    a, b = w.field().blocks
    assert b.x > 0.2                                          # the far one moved too
    assert math.hypot(a.x - b.x, a.y - b.y) >= 2 * BLOCK_RADIUS_M - 0.005


def test_blocks_stay_inside_the_walls():
    field = Field((Block(1.5, 0.0, "red"),))
    w = world(Pose(0.8, 0.0, 0.0), field)
    run(w, 12.0, 12.0, 2.0)
    (b,) = w.field().blocks
    assert b.x <= HALF - BLOCK_RADIUS_M + 0.005


def test_intake_captures_a_block_at_the_front_and_ejects_it():
    field = Field((Block(0.0, 0.0, "blue"),))
    w = world(Pose(-0.5, 0.0, 0.0), field)
    run(w, 4.0, 4.0, 1.0, intake_rad_s=20.0, capacity=2)
    f = w.field()
    assert f.held == ("blue",) and f.blocks == ()
    run(w, 0.0, 0.0, 0.5, intake_rad_s=-20.0, capacity=2)
    f = w.field()
    assert f.held == () and len(f.blocks) == 1
    (b,) = f.blocks
    lx = (b.x - w.pose().x_m) * math.cos(w.pose().heading_rad)
    assert lx > ROBOT_HALF


def test_a_standing_robot_ejects_everything_it_holds():
    """Each block is thrown clear, so the next one has room."""
    field = Field(tuple(Block(x, 0.0, c) for x, c in ((0.0, "red"), (0.2, "blue"), (0.4, "red"))))
    w = world(Pose(-0.5, 0.0, 0.0), field)
    run(w, 4.0, 4.0, 2.0, intake_rad_s=20.0, capacity=3)
    assert len(w.field().held) == 3
    run(w, 0.0, 0.0, 1.5, intake_rad_s=-20.0, capacity=3)
    f = w.field()
    assert f.held == () and len(f.blocks) == 3


def test_full_intake_plows_instead():
    field = Field((Block(0.0, 0.0, "red"),), held=("blue",))
    w = world(Pose(-0.5, 0.0, 0.0), field)
    run(w, 4.0, 4.0, 1.0, intake_rad_s=20.0, capacity=1)
    f = w.field()
    assert f.held == ("blue",) and len(f.blocks) == 1


def test_same_inputs_same_result():
    a = run(world(Pose(-1.0, 0.2, 0.3)), 9.0, 5.0, 1.0)
    b = run(world(Pose(-1.0, 0.2, 0.3)), 9.0, 5.0, 1.0)
    assert a == b


def test_start_pose_and_heading_are_kept():
    st = run(world(Pose(0.3, -0.4, math.pi / 2)), None, None, 0.1)
    assert (st.pose.x_m, st.pose.y_m) == pytest.approx((0.3, -0.4), abs=1e-4)
    assert st.pose.heading_rad == pytest.approx(math.pi / 2, abs=1e-4)


def test_encoders_track_true_distance_below_the_traction_limit():
    """Odometry only drifts when the wheels really slip: an 8 V start asks
    ~23 N a side (traction 25.5 N), then a coast; encoder distance matches
    the ground's to 0.5 %. (Reversing at speed would skid: ~46 N.)"""
    w = world(Pose(-1.2, 0.0, 0.0))
    for volts, secs in ((8.0, 0.8), (None, 0.8)):
        st = run(w, volts, volts, secs)
    turns = st.left[0].position_rad / SPEC.external_ratio
    encoder_m = turns * SPEC.wheel_radius_m
    assert encoder_m == pytest.approx(st.pose.x_m + 1.2, rel=0.005)


def test_turning_on_the_spot_does_not_wander():
    w = world()
    for volts, secs in ((-8.0, 0.6), (8.0, 0.3), (0.0, 0.3)):
        st = run(w, volts, -volts, secs)
    assert math.hypot(st.pose.x_m, st.pose.y_m) < 0.005


# ---- holonomic drives -----------------------------------------------------

from v5sim import holonomic  # noqa: E402
from v5sim.motor import Cartridge  # noqa: E402

XSPEC = DrivetrainSpec(motors_per_side=2, cartridge=Cartridge.GREEN_18, holonomic="x")
MSPEC = DrivetrainSpec(motors_per_side=2, cartridge=Cartridge.GREEN_18, holonomic="mecanum")


def holo_builtin(spec, lf, lb, rf, rb, seconds):
    st = drivetrain.initial_state(spec, Pose())
    for _ in range(round(seconds / DT)):
        st = holonomic.step(spec, st, [lf, lb], [rf, rb], DT)
    return st


def holo_mujoco(spec, lf, lb, rf, rb, seconds, pose=Pose()):
    w = world(pose, spec=spec)
    st = None
    for _ in range(round(seconds / DT)):
        st = w.step([lf, lb], [rf, rb], DT)
    return st


@pytest.mark.parametrize("spec", [XSPEC, MSPEC], ids=["x", "mecanum"])
def test_holonomic_forward_matches_the_builtin_model(spec):
    ref = holo_builtin(spec, 6, 6, 6, 6, 1.5)
    st = holo_mujoco(spec, 6, 6, 6, 6, 1.5)
    assert st.v_m_s == pytest.approx(ref.v_m_s, rel=0.05)
    assert abs(st.strafe_m_s) < 0.01 and abs(st.omega_rad_s) < 0.02


@pytest.mark.parametrize("spec", [XSPEC, MSPEC], ids=["x", "mecanum"])
def test_holonomic_strafe_right_matches_the_builtin_model(spec):
    ref = holo_builtin(spec, 6, -6, -6, 6, 1.5)
    st = holo_mujoco(spec, 6, -6, -6, 6, 1.5)
    assert ref.strafe_m_s > 0.25                                 # right, per holonomic.py
    assert st.strafe_m_s == pytest.approx(ref.strafe_m_s, rel=0.05)
    assert abs(st.v_m_s) < 0.01 and abs(st.omega_rad_s) < 0.02
    assert st.pose.y_m < -0.2                                    # sim frame: right = -y


@pytest.mark.parametrize("spec", [XSPEC, MSPEC], ids=["x", "mecanum"])
def test_holonomic_turn_matches_the_builtin_model(spec):
    ref = holo_builtin(spec, 6, 6, -6, -6, 1.0)
    st = holo_mujoco(spec, 6, 6, -6, -6, 1.0)
    assert ref.omega_rad_s < 0                                   # clockwise
    assert st.omega_rad_s == pytest.approx(ref.omega_rad_s, rel=0.10)


def test_holonomic_slides_along_a_wall():
    st = holo_mujoco(XSPEC, 8, 8, 8, 8, 2.0, pose=Pose(1.2, 0.0, math.radians(30)))
    assert st.pose.x_m == pytest.approx(HALF - ROBOT_HALF, abs=0.02)
    assert st.pose.y_m > 0.15                                    # pushed along, not stuck


# ---- field elements and an opponent ------------------------------------

from v5sim.physics_mujoco import Obstacle, Opponent  # noqa: E402


def test_robot_stops_at_a_field_element():
    wall = Obstacle("box", x=0.6, y=0.0, sx=0.05, sy=0.5)
    w = MujocoWorld(SPEC, HALF, ROBOT_HALF, Pose(-0.4, 0.0, 0.0), obstacles=(wall,))
    st = run(w, 8.0, 8.0, 2.0)
    assert st.pose.x_m == pytest.approx(0.6 - 0.05 - ROBOT_HALF, abs=0.01)


def test_round_element_deflects_the_robot():
    post = Obstacle("cylinder", x=0.5, y=0.12, r=0.08)
    w = MujocoWorld(SPEC, HALF, ROBOT_HALF, Pose(-0.4, 0.0, 0.0), obstacles=(post,))
    st = run(w, 6.0, 6.0, 1.5)
    assert st.pose.y_m < -0.01 or abs(st.pose.heading_rad) > 0.05   # glanced off


def test_blocks_stop_at_field_elements_too():
    wall = Obstacle("box", x=0.4, y=0.0, sx=0.05, sy=0.5)
    w = MujocoWorld(SPEC, HALF, ROBOT_HALF, Pose(-0.5, 0.0, 0.0), Field((Block(0.0, 0.0, "red"),)),
                    obstacles=(wall,))
    run(w, 6.0, 6.0, 2.0)
    (b,) = w.field().blocks
    assert b.x <= 0.4 - 0.05 - BLOCK_RADIUS_M + 0.005


def test_pushing_a_coasting_opponent_moves_it_but_slower_than_driving_free():
    free = run(world(Pose(-0.8, 0.0, 0.0)), 12.0, 12.0, 1.2)
    w = MujocoWorld(SPEC, HALF, ROBOT_HALF, Pose(-0.8, 0.0, 0.0),
                    opponents=(Opponent(Pose(-0.8 + 2 * ROBOT_HALF + 0.02, 0.0, 0.0)),))
    st = run(w, 12.0, 12.0, 1.2)
    (opp,) = w.opponents()
    assert opp.x_m > -0.8 + 2 * ROBOT_HALF + 0.05                 # shoved
    assert st.pose.x_m < free.pose.x_m - 0.3                     # but it pushed back
    assert opp.x_m - st.pose.x_m == pytest.approx(2 * ROBOT_HALF, abs=0.02)


def test_a_braked_opponent_of_the_same_weight_holds_its_ground():
    """Traction caps our push (mu m g, 51 N) below its skid friction (55 N)."""
    touching = -0.8 + 2 * ROBOT_HALF
    w = MujocoWorld(SPEC, HALF, ROBOT_HALF, Pose(-0.8, 0.0, 0.0),
                    opponents=(Opponent(Pose(touching, 0.0, 0.0), braked=True),))
    run(w, 12.0, 12.0, 1.0)
    (opp,) = w.opponents()
    assert opp.x_m == pytest.approx(touching, abs=0.005)


def test_ramming_a_braked_opponent_shoves_it_then_it_holds():
    """A run-up's momentum beats its friction for a moment, then it sticks."""
    start = -0.8 + 2 * ROBOT_HALF + 0.05
    w = MujocoWorld(SPEC, HALF, ROBOT_HALF, Pose(-0.8, 0.0, 0.0),
                    opponents=(Opponent(Pose(start, 0.0, 0.0), braked=True),))
    run(w, 12.0, 12.0, 2.5)
    (shoved,) = w.opponents()
    run(w, 12.0, 12.0, 1.0)
    (later,) = w.opponents()
    assert shoved.x_m > start + 0.02
    assert later.x_m == pytest.approx(shoved.x_m, abs=0.002)


def test_a_coasting_opponent_resists_sideways():
    """Pushed side-on its traction wheels grip: it hardly moves."""
    w = MujocoWorld(SPEC, HALF, ROBOT_HALF, Pose(-0.8, 0.0, 0.0),
                    opponents=(Opponent(Pose(-0.8 + 2 * ROBOT_HALF, 0.0, math.pi / 2)),))
    run(w, 12.0, 12.0, 1.0)
    (opp,) = w.opponents()
    assert opp.x_m - (-0.8 + 2 * ROBOT_HALF) < 0.005
