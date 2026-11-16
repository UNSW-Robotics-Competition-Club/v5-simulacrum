"""MuJoCo physics for the robot and its field (`--physics mujoco`).

The same robot as drivetrain.py, in a MuJoCo world with real contact:

  - the chassis is a planar body (x, y, yaw) with the spec's mass and yaw
    inertia and a square box collider; the field walls are static boxes
  - wheels are joints driven by their motors through the external ratio
    (motor.py's model): a tank drive has one per side, with the side's spin
    inertia; an X-drive or mecanum drive four on the corners pushing at 45
    degrees, as holonomic.py lays them out, rolling freely across their
    direction (omni rollers) with mecanum's sqrt(2) rim factor
  - the ground force at each wheel is a tire model: force along the wheel
    from its slip and, for a tank's traction wheels, against sideways
    sliding, capped at traction (mu x the wheel's share of the weight).
    Pushing a wall hard enough therefore spins the wheels, and the encoders
    count it.
  - blocks are bodies sliding on the tiles with Coulomb friction; the robot
    pushes them and they push each other. The intake takes a block that
    reaches the front of the robot and ejects one there.
  - field elements (Obstacle: boxes, cylinders) are static colliders for the
    robot and the blocks; opponents are other robots: a braked one skids
    only past mu x its weight (true static friction), a coasting one rolls
    along its heading and grips sideways like a tank on traction wheels.

Rolling resistance acts at the wheels, drag on the chassis, turn scrub as
yaw damping: the builtin model's terms, so the two agree on open ground.
step() returns the drivetrain.DrivetrainState the rest of the sim uses.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import mujoco

from . import holonomic, motor
from .drivetrain import G, DrivetrainSpec, DrivetrainState, Pose
from .field_elements import Obstacle, Opponent
from .field_objects import (BLOCK_RADIUS_M, EJECT_PERIOD_S, INTAKE_HALF_WIDTH_M,
                            INTAKE_MIN_RAD_S, Block, Field)
from .motor import Drive, MotorState

# Tire forces are explicit, so their stiffness is bounded by the step:
# k * dt * r^2 / I_wheel (and k_lat * dt / m) must stay well under 1.
# At 0.25 ms a tire needs ~8 mm/s of slip for full traction, so encoder
# odometry matches the ground until the wheels really slip.
TIMESTEP_S = 0.00025
SLIP_STIFFNESS_N_PER_M_S = 3000.0    # longitudinal force per m/s of wheel slip
LATERAL_STIFFNESS_N_PER_M_S = 6000.0
BLOCK_MASS_KG = 0.06
BLOCK_HALF_HEIGHT_M = 0.04
BLOCK_FLOOR_MU = 0.5
GEOM_Z_M = 0.05                       # every collider's centre height
WALL_THICKNESS_M = 0.05
ROBOT_WALL_MU = 0.3
PARK_M = 50.0                         # where held blocks wait, off the field
CAPTURE_DEPTH_M = 2 * BLOCK_RADIUS_M + 0.01
EJECT_SPEED_M_S = 0.7                 # the rollers' push: a block slides ~5 cm
OBSTACLE_MU = 0.3
OPPONENT_MU = 0.8
OPPONENT_ROLLING = 0.02          # fraction of its weight, coasting forwards
OPPONENT_YAW_ARM_M = 0.12        # wheel friction's lever about its centre
STICTION_N_PER_M = 2.0e5         # a gripping wheel's spring to where it stuck
STICTION_DAMP_N_PER_M_S = 1500.0


@dataclass(frozen=True)
class Wheel:
    name: str
    fwd: float                  # contact point in the body frame, m
    left: float
    ux: float                   # unit push direction in the body frame (fwd, left)
    uy: float
    rim_scale: float            # rim speed per unit of chassis speed along u
    grips_sideways: bool        # traction wheel (tank) vs free rollers (omni)
    motors: tuple[tuple[int, int], ...]   # (side 0/1, index) of its motors


def wheels_for(spec: DrivetrainSpec) -> list[Wheel]:
    half = spec.track_width_m / 2
    n = spec.motors_per_side
    if not spec.holonomic:
        return [Wheel("wheel_left", 0.0, half, 1.0, 0.0, 1.0, True,
                      tuple((0, i) for i in range(n))),
                Wheel("wheel_right", 0.0, -half, 1.0, 0.0, 1.0, True,
                      tuple((1, i) for i in range(n)))]
    c = holonomic.RIM_SCALE[spec.holonomic]
    sides = ((0, 0), (0, 1), (1, 0), (1, 1))              # LF, LB, RF, RB
    names = ("wheel_lf", "wheel_lb", "wheel_rf", "wheel_rb")
    # holonomic.py: x right, y forward; here x forward, y left
    return [Wheel(name, y, -x, uy, -ux, c, False, (side,))
            for name, side, (x, y, ux, uy) in zip(names, sides, holonomic._wheels(spec))]


def _obstacle_xml(o: Obstacle) -> str:
    if o.kind == "box":
        shape = f'type="box" size="{o.sx} {o.sy} 0.15" euler="0 0 {o.angle_deg}"'
    else:
        shape = f'type="cylinder" size="{o.r} 0.15"'
    return (f'<geom {shape} pos="{o.x} {o.y} {GEOM_Z_M}" '
            f'friction="{OBSTACLE_MU} 0.005 0.0001"/>')


def _opponent_xml(i: int, o: Opponent) -> str:
    iz = o.mass_kg * (2 * o.half_m) ** 2 / 6
    return (f'<body name="opp{i}" pos="0 0 {GEOM_Z_M}">'
            f'<joint name="opp{i}_x" type="slide" axis="1 0 0"/>'
            f'<joint name="opp{i}_y" type="slide" axis="0 1 0"/>'
            f'<joint name="opp{i}_yaw" type="hinge" axis="0 0 1"/>'
            f'<inertial pos="0 0 0" mass="{o.mass_kg}" diaginertia="{iz} {iz} {iz}"/>'
            f'<geom type="box" size="{o.half_m} {o.half_m} 0.05" mass="0" '
            f'friction="{ROBOT_WALL_MU} 0.005 0.0001"/></body>')


def _model_xml(spec: DrivetrainSpec, half_m: float, robot_half_m: float,
               n_blocks: int, obstacles: Sequence[Obstacle] = (),
               opponents: Sequence[Opponent] = ()) -> str:
    t = WALL_THICKNESS_M
    walls = "".join(
        f'<geom type="box" pos="{x} {y} {GEOM_Z_M}" size="{sx} {sy} 0.15" '
        f'friction="{ROBOT_WALL_MU} 0.005 0.0001"/>'
        for x, y, sx, sy in ((half_m + t, 0, t, half_m + 2 * t),
                             (-half_m - t, 0, t, half_m + 2 * t),
                             (0, half_m + t, half_m + 2 * t, t),
                             (0, -half_m - t, half_m + 2 * t, t)))
    wl = wheels_for(spec)
    # drivetrain.py's rolling resistance and each side's spin inertia, shared
    # between the wheels
    share = len(wl) / 2
    # each motor's gearbox friction, at the wheel (motor.py)
    gear_nm = motor.gear_friction_nm(spec.cartridge)[0] * spec.external_ratio
    wheels = "".join(
        f'<body name="{w.name}" pos="{w.fwd} {w.left} 0">'
        f'<joint name="{w.name}" type="hinge" axis="0 1 0" '
        f'armature="{spec.wheel_spin_inertia_kg_m2 / share}" '
        f'frictionloss="{spec.rolling_resistance * spec.mass_kg * G / len(wl) * spec.wheel_radius_m / w.rim_scale + gear_nm * len(w.motors)}"/>'
        f'<inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/></body>'
        for w in wl)
    slide_loss = BLOCK_FLOOR_MU * BLOCK_MASS_KG * G
    blocks = "".join(
        f'<body name="block{i}" pos="0 0 {GEOM_Z_M}">'
        f'<joint type="slide" axis="1 0 0" frictionloss="{slide_loss}" damping="0.01"/>'
        f'<joint type="slide" axis="0 1 0" frictionloss="{slide_loss}" damping="0.01"/>'
        f'<joint type="hinge" axis="0 0 1" frictionloss="0.0005" damping="1e-5"/>'
        f'<geom name="block{i}" type="cylinder" size="{BLOCK_RADIUS_M} {BLOCK_HALF_HEIGHT_M}" '
        f'mass="{BLOCK_MASS_KG}" friction="0.4 0.005 0.0001"/></body>'
        for i in range(n_blocks))
    m, iz = spec.mass_kg, spec.yaw_inertia_kg_m2
    return f"""<mujoco model="v5robot">
  <option timestep="{TIMESTEP_S}" gravity="0 0 0" integrator="implicitfast"/>
  <default><geom solref="0.004 1" solimp="0.95 0.99 0.001" condim="3"/></default>
  <worldbody>
    {walls}
    {"".join(_obstacle_xml(o) for o in obstacles)}
    {"".join(_opponent_xml(i, o) for i, o in enumerate(opponents))}
    <body name="robot" pos="0 0 {GEOM_Z_M}">
      <joint name="x" type="slide" axis="1 0 0"/>
      <joint name="y" type="slide" axis="0 1 0"/>
      <joint name="yaw" type="hinge" axis="0 0 1" damping="{spec.turn_scrub_nm_per_rad_s}"/>
      <inertial pos="0 0 0" mass="{m}" diaginertia="{iz} {iz} {iz}"/>
      <geom name="chassis" type="box" size="{robot_half_m} {robot_half_m} 0.05" mass="0"
            friction="{ROBOT_WALL_MU} 0.005 0.0001"/>
      {wheels}
    </body>
    {blocks}
  </worldbody>
</mujoco>"""


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


class MujocoWorld:
    def __init__(self, spec: DrivetrainSpec, field_half_m: float, robot_half_m: float,
                 pose: Pose = Pose(), field: Field | None = None,
                 obstacles: Sequence[Obstacle] = (), opponents: Sequence[Opponent] = ()):
        self.spec = spec
        self.robot_half_m = robot_half_m
        self.field_half_m = field_half_m
        field = field or Field()
        self._colors = [b.color for b in field.blocks] + list(field.held)
        self.obstacles = tuple(obstacles)
        self._opponents = tuple(opponents)
        self.model = mujoco.MjModel.from_xml_string(
            _model_xml(spec, field_half_m, robot_half_m, len(self._colors),
                       self.obstacles, self._opponents))
        self.data = mujoco.MjData(self.model)
        self.wheels = wheels_for(spec)
        j = self.model.joint
        self._q = {name: (j(name).qposadr[0], j(name).dofadr[0])
                   for name in ("x", "y", "yaw", *(w.name for w in self.wheels))}
        self._block_dofs = [self.model.body(f"block{i}").dofadr[0]
                            for i in range(len(self._colors))]
        self._block_qpos = [self.model.jnt_qposadr[self.model.body(f"block{i}").jntadr[0]]
                            for i in range(len(self._colors))]
        self._block_geoms = [self.model.geom(f"block{i}").id for i in range(len(self._colors))]
        self._active = [True] * len(field.blocks) + [False] * len(field.held)
        self._held = list(range(len(field.blocks), len(self._colors)))   # body ids
        self._eject_wait = field.eject_wait_s
        n = spec.motors_per_side
        self._motors = (tuple(MotorState() for _ in range(n)),
                        tuple(MotorState() for _ in range(n)))
        qp = self.data.qpos
        qp[self._q["x"][0]], qp[self._q["y"][0]] = pose.x_m, pose.y_m
        qp[self._q["yaw"][0]] = pose.heading_rad
        self._opp_q = []
        self._opp_stuck = [(o.pose.x_m, o.pose.y_m, o.pose.heading_rad)
                           for o in self._opponents]
        for i, o in enumerate(self._opponents):
            q = tuple((j(f"opp{i}_{a}").qposadr[0], j(f"opp{i}_{a}").dofadr[0])
                      for a in ("x", "y", "yaw"))
            self._opp_q.append(q)
            for (adr, _), v in zip(q, (o.pose.x_m, o.pose.y_m, o.pose.heading_rad)):
                qp[adr] = v
        for i, b in enumerate(field.blocks):
            self._place(i, b.x, b.y)
        for i in self._held:
            self._park(i)
        mujoco.mj_forward(self.model, self.data)

    # ---- blocks ---------------------------------------------------------

    def _place(self, i: int, x: float, y: float, vx: float = 0.0, vy: float = 0.0) -> None:
        a = self._block_qpos[i]
        self.data.qpos[a:a + 3] = (x, y, 0.0)
        d = self._block_dofs[i]
        self.data.qvel[d:d + 3] = (vx, vy, 0.0)
        self.model.geom_contype[self._block_geoms[i]] = 1
        self.model.geom_conaffinity[self._block_geoms[i]] = 1

    def _park(self, i: int) -> None:
        a = self._block_qpos[i]
        self.data.qpos[a:a + 3] = (PARK_M + i, PARK_M, 0.0)
        d = self._block_dofs[i]
        self.data.qvel[d:d + 3] = 0.0
        self.model.geom_contype[self._block_geoms[i]] = 0
        self.model.geom_conaffinity[self._block_geoms[i]] = 0

    def _block_xy(self, i: int) -> tuple[float, float]:
        a = self._block_qpos[i]
        return float(self.data.qpos[a]), float(self.data.qpos[a + 1])

    def _local(self, x: float, y: float) -> tuple[float, float]:
        p = self.pose()
        c, s = math.cos(p.heading_rad), math.sin(p.heading_rad)
        dx, dy = x - p.x_m, y - p.y_m
        return c * dx + s * dy, -s * dx + c * dy

    def _intake(self, rad_s: float, capacity: int, dt: float) -> None:
        self._eject_wait = max(self._eject_wait - dt, 0.0)
        if rad_s > INTAKE_MIN_RAD_S:
            for i, on in enumerate(self._active):
                if not on or len(self._held) >= capacity:
                    continue
                lx, ly = self._local(*self._block_xy(i))
                if (self.robot_half_m <= lx <= self.robot_half_m + CAPTURE_DEPTH_M
                        and abs(ly) < INTAKE_HALF_WIDTH_M):
                    self._active[i] = False
                    self._held.append(i)
                    self._park(i)
        elif rad_s < -INTAKE_MIN_RAD_S:
            self._eject(capacity)

    def _eject(self, capacity: int) -> None:
        """Reversed rollers drive whatever is in the intake's mouth out, and
        once the mouth is clear put the next held block there, moving out."""
        p = self.pose()
        c, s = math.cos(p.heading_rad), math.sin(p.heading_rad)
        qv = self.data.qvel
        v = EJECT_SPEED_M_S + max(0.0, c * qv[self._q["x"][1]] + s * qv[self._q["y"][1]])
        in_mouth = []
        for k, on in enumerate(self._active):
            if on:
                lx, ly = self._local(*self._block_xy(k))
                if (self.robot_half_m - 0.01 <= lx < self.robot_half_m + 2 * BLOCK_RADIUS_M + 0.01
                        and abs(ly) < INTAKE_HALF_WIDTH_M):
                    in_mouth.append(k)
        for k in in_mouth:
            d = self._block_dofs[k]
            self.data.qvel[d:d + 2] = (v * c, v * s)
        if not self._held or self._eject_wait > 0.0 or in_mouth:
            return
        reach = self.robot_half_m + BLOCK_RADIUS_M + 0.01
        x, y = p.x_m + reach * c, p.y_m + reach * s
        lim = self.field_half_m - BLOCK_RADIUS_M
        if abs(x) <= lim and abs(y) <= lim:   # else keep it
            i = self._held.pop()
            self._active[i] = True
            self._place(i, x, y, v * c, v * s)
            self._eject_wait = EJECT_PERIOD_S

    def field(self) -> Field:
        blocks = tuple(Block(*self._block_xy(i), self._colors[i])
                       for i, on in enumerate(self._active) if on)
        return Field(blocks, tuple(self._colors[i] for i in self._held), self._eject_wait)

    # ---- robot ----------------------------------------------------------

    def pose(self) -> Pose:
        qp = self.data.qpos
        return Pose(float(qp[self._q["x"][0]]), float(qp[self._q["y"][0]]),
                    float(qp[self._q["yaw"][0]]))

    def _wheel_forces(self, drives: tuple[Sequence[Drive], Sequence[Drive]],
                      limits: tuple[Sequence[float], Sequence[float]]) -> None:
        spec, d = self.spec, self.data
        qv, f = d.qvel, d.qfrc_applied
        f[:] = 0.0
        yaw = float(d.qpos[self._q["yaw"][0]])
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = float(qv[self._q["x"][1]]), float(qv[self._q["y"][1]])
        wz = float(qv[self._q["yaw"][1]])
        r, ratio = spec.wheel_radius_m, spec.external_ratio
        traction = spec.traction_mu * spec.mass_kg * G / len(self.wheels)
        fx = fy = tz = 0.0
        for w in self.wheels:
            dof = self._q[w.name][1]
            w_wheel = float(qv[dof])
            w_motor = w_wheel * ratio
            torque = sum(motor.output_torque(
                motor.motor_current(drives[k][i], w_motor, spec.cartridge, limits[k][i]),
                spec.cartridge) for k, i in w.motors) * ratio
            # contact point and push direction in the world
            px, py = c * w.fwd - s * w.left, s * w.fwd + c * w.left
            ux, uy = c * w.ux - s * w.uy, s * w.ux + c * w.uy
            cvx, cvy = vx - wz * py, vy + wz * px
            v_u = ux * cvx + uy * cvy                 # along the wheel
            v_n = -uy * cvx + ux * cvy                # across it
            f_u = _clamp(SLIP_STIFFNESS_N_PER_M_S * (w_wheel * r / w.rim_scale - v_u),
                         traction)
            f_n = (_clamp(-LATERAL_STIFFNESS_N_PER_M_S * v_n, traction)
                   if w.grips_sideways else 0.0)
            f[dof] = torque - f_u * r / w.rim_scale
            gx, gy = ux * f_u - uy * f_n, uy * f_u + ux * f_n
            fx, fy = fx + gx, fy + gy
            tz += px * gy - py * gx
        f[self._q["x"][1]] = fx - spec.drag_n_per_m_s * vx
        f[self._q["y"][1]] = fy - spec.drag_n_per_m_s * vy
        f[self._q["yaw"][1]] = tz
        self._opponent_forces()

    def _opponent_forces(self) -> None:
        """Opponents grip by stiction: a spring to the point where a wheel
        stuck, which slips (the point follows) past mu x its weight, so a
        push below that holds for good. A coasting one rolls along its
        heading (rolling resistance) and grips sideways; a braked one grips
        every way, turning too."""
        qp, qv, f = self.data.qpos, self.data.qvel, self.data.qfrc_applied
        for i, (o, ((ax, dx), (ay, dy), (ayaw, dyaw))) in enumerate(
                zip(self._opponents, self._opp_q)):
            h = float(qp[ayaw])
            c, s = math.cos(h), math.sin(h)
            x, y = float(qp[ax]), float(qp[ay])
            vx, vy = float(qv[dx]), float(qv[dy])
            sx, sy, sh = self._opp_stuck[i]
            grip = OPPONENT_MU * o.mass_kg * G

            def stick(d: float, v: float, limit: float, k: float = STICTION_N_PER_M,
                      kd: float = STICTION_DAMP_N_PER_M_S) -> tuple[float, float]:
                """(force, how far the stuck point moves to slip)."""
                want = -k * d - kd * v
                if abs(want) <= limit:
                    return want, 0.0
                force = math.copysign(limit, want)
                return force, (d + force / k) if abs(k * d) > limit else 0.0

            ex, ey = x - sx, y - sy
            d_fwd, d_lat = c * ex + s * ey, -s * ex + c * ey
            v_fwd, v_lat = c * vx + s * vy, -s * vx + c * vy
            f_lat, slip_lat = stick(d_lat, v_lat, grip)
            if o.braked:
                f_fwd, slip_fwd = stick(d_fwd, v_fwd, grip)
                arm = OPPONENT_YAW_ARM_M
                tz, slip_h = stick(h - sh, float(qv[dyaw]), grip * arm,
                                   STICTION_N_PER_M * arm * arm, STICTION_DAMP_N_PER_M_S * arm * arm)
                f[dyaw] = tz
            else:
                roll = OPPONENT_ROLLING * o.mass_kg * G
                f_fwd, slip_fwd = _clamp(-roll * v_fwd / 0.01, roll), d_fwd
                slip_h = h - sh
            f[dx] = c * f_fwd - s * f_lat
            f[dy] = s * f_fwd + c * f_lat
            self._opp_stuck[i] = (sx + c * slip_fwd - s * slip_lat,
                                  sy + s * slip_fwd + c * slip_lat, sh + slip_h)

    def opponents(self) -> list[Pose]:
        qp = self.data.qpos
        return [Pose(float(qp[x]), float(qp[y]), float(qp[yaw]))
                for (x, _), (y, _), (yaw, _) in self._opp_q]

    def step(self, left: Sequence[Drive], right: Sequence[Drive], dt: float,
             limits_a: Sequence[float] | None = None, intake_rad_s: float = 0.0,
             capacity: int = 0) -> DrivetrainState:
        n = self.spec.motors_per_side
        if len(left) != n or len(right) != n:
            raise ValueError(f"need {n} drives per side")
        lim = list(limits_a) if limits_a else [motor.CURRENT_LIMIT_A] * (2 * n)
        lims = (lim[:n], lim[n:])
        for _ in range(max(1, round(dt / TIMESTEP_S))):
            self._wheel_forces((left, right), lims)
            mujoco.mj_step(self.model, self.data)
        self._intake(intake_rad_s, capacity, dt)
        return self._state((left, right), lims, dt)

    def _state(self, drives, lims, dt: float) -> DrivetrainState:
        spec, qv = self.spec, self.data.qvel
        pose = self.pose()
        c, s = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
        vx, vy = float(qv[self._q["x"][1]]), float(qv[self._q["y"][1]])
        v, strafe = c * vx + s * vy, s * vx - c * vy           # forward, right
        omega = float(qv[self._q["yaw"][1]])
        speed = {}                                              # (side, i) -> motor rad/s
        for w in self.wheels:
            for key in w.motors:
                speed[key] = float(qv[self._q[w.name][1]]) * spec.external_ratio
        sides = [tuple(motor.advance(m, drives[k][i], speed[(k, i)], dt, spec.cartridge,
                                     lims[k][i])
                       for i, m in enumerate(self._motors[k])) for k in (0, 1)]
        self._motors = (sides[0], sides[1])
        slips = [0.0, 0.0]
        if not spec.holonomic:                  # tank: spin beyond rolling, per side
            half = spec.track_width_m / 2
            for k, sign in ((0, -1), (1, 1)):
                ground = (v + sign * omega * half) / spec.wheel_radius_m * spec.external_ratio
                slips[k] = speed[(k, 0)] - ground
        return DrivetrainState(pose=pose, v_m_s=v, omega_rad_s=omega, left=sides[0],
                               right=sides[1], left_slip=slips[0], right_slip=slips[1],
                               strafe_m_s=strafe)
