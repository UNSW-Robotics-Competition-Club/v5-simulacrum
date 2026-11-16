"""Run simulated V5 devices against an emulated brain's smart ports.

  python3 -m v5sim.robot --drive 1,2,3:4,5,6:blue --imu 10 --view 8765

--motor P[r]:CART     a motor on port P driving a light free shaft (r: it
                      turns the other way, e.g. an intake reversed in code)
--drive L:R:CART      a drivetrain; L and R are comma-separated ports.
                      Right-side motors are mounted mirrored, as on a real
                      robot, so programs must reverse them to drive forward.
                      A port written "4r" is mounted the other way round
                      from the rest of its side (the program reverses it):
                      e.g. 1r,2r,3:4r,5r,6 for RW-Template's defaults
--motor-fw P|all      the motor on port P (repeatable, or every motor) runs
                      VEX's own motor firmware (DEVFRM02 / 05) in its own
                      QEMU (machine vex-motor): the real PID, current limit
                      and faults drive the physics (firmware_motor.py)
--holonomic x|mecanum the --drive is a four-motor X-drive or mecanum drive
                      (holonomic.py); list each side front then back, e.g.
                      --drive LF,LB:RF,RB:green --holonomic x
--imu P               an Inertial Sensor on port P, mounted on the drivetrain
                      (flat, heading clockwise-positive like VEX)
--rotation P[r]:SRC   a Rotation Sensor on port P; SRC "track[:D[:X:Y]]" =
                      a tracking wheel under the drivetrain rolling forward,
                      "strafe[:D[:X:Y]]" = one rolling sideways (positive
                      to the right); D its diameter (2.75 in), X, Y where it
                      touches the floor in inches from the turning centre
                      (right, forward; 0:0). "motorN" = the shaft of the
                      motor on port N. r: reads the other way round
--distance P[:FACE[:X:Y]]
                      a Distance Sensor measuring to the field walls. FACE:
                      front (default), right, back, left or degrees clockwise
                      from the front; X, Y: its offset from the robot's
                      centre in inches (right, forward; default on that face)
--adi P:A=KIND,...    3-wire ports A..H; P 22 = the brain's built-in ports,
                      other P = a 3-wire expander. KIND: bumper|limit (front,
                      pressed at the wall), encoder|sonar (A/C/E/G, uses the
                      next port too; tracking wheel / front echo), piston
                      (digital out, shown in the status line), gyro (robot
                      yaw), pot:N / potv2:N (on motor N's shaft), line,
                      light, accel (forward); an addressable LED strip on a
                      digital-out port shows in the status line. Port 22 is
                      present by default (unwired) so the on-brain 3-wire
                      screen works; pass --adi 22:... to wire it
--no-builtin-adi      leave port 22 alone: for a second plant that only plugs
                      devices into free ports of a running brain (its own
                      plant already holds 22), e.g. a field-control
                      brain's tethered controller
--field-controller P[:TEAM]
                      a robot's V5 Controller smart-cabled to port P of a
                      brain running the VEX field-control app (ports 1-5
                      red1, 6-10 blue1, 11-15 blue2, 16-20 red2): reports
                      its robot linked over VEXnet with team TEAM (default
                      SIM<P>). Repeatable, one per station
--optical P           an Optical Sensor at the intake, facing forward: sees
                      the newest held block (proximity 255), a block at the
                      front, or the floor
--aivision P          an AI Vision Sensor at the robot's front: sees --blocks
                      ahead (320x240, 74 deg FOV); reports colour objects for
                      matching colour signatures, and Cube(Red/Blue) model
                      objects when model detection is on
--tags FILE.json      AprilTags on the field for --aivision (apriltags.py):
                      [{"id", "x", "y", "facing"}, ...] in m / degrees CCW
                      from +x, optional "z", "size", "family"
--vision P            a (legacy) Vision Sensor at the robot's front: sees
                      --blocks ahead (316x212, 60 deg FOV) and reports those
                      matching its colour signatures
--gps P               a GPS Sensor: the robot's pose from the field strip
                      (x, y in m from the centre; heading 0 = +y, clockwise)
--start X:Y:HEADING   start pose in RW-Template field coordinates: inches from
                      the centre, heading clockwise from +Y (the status line
                      shows the pose that way too, as rw=(x, y, h))
--blocks SPEC         game objects (Push Back blocks): random:N[:SEED] or a
                      JSON file of [x, y, "red"|"blue"] (m from the centre).
                      The robot plows them; see --intake
--goals FILE.json     scoring zones for the blocks (scoring.py): a block
                      inside a goal's rect scores for its colour, the colour
                      with more blocks there gets the goal's control points
--arm P:RATIO:LEN:KG[:START:LO:HI]
                      the --motor on port P drives an arm: RATIO motor turns
                      per arm turn, LEN inches to a KG load, gravity and hard
                      stops at LO / HI degrees above level (default -90, 90),
                      starting at START (default LO); e.g. 8:5:12:1.5:-30:-30:90
--intake P[:CAP]      the --motor on port P is an intake at the robot's
                      front: spinning forward captures blocks (up to CAP,
                      default 6), reverse ejects them
--magnet P            a V5 Electromagnet: power, timeout, current, temperature
--tower P             a Light Tower (signaltower): its five lights and blink
                      state in the status line; button never pressed
--pneumatic P         a CTE pneumatics controller: pump, four cylinders,
                      actuation counters and a pressure model
--led P               a smart-port LED: its colour in the status line
--controller P        a tethered V5 Controller on port P; drive it from the
                      field view page or POST JSON to /input (needs --view)
--radio P[:partner]   a VEXnet robot radio on port P linked to a wireless
                      controller (driven like --controller); "partner" adds
                      a second controller, set through /input's "partner".
                      With --match the radio reports field control and the
                      seconds counter the brain's header timer shows.
                      It replaces a --controller, and the last --radio
                      replaces earlier ones (e.g. a preset's radio)
--match START[:AUTON:DRIVER]
                      run a timed match through the controller's competition
                      byte: disabled until START s, AUTON s autonomous
                      (default 15), a 2 s pause, DRIVER s driver control
                      (default 105), then disabled. AUTON 0 = skills run
--competition field|switch
                      plug into competition control, disabled until the
                      field view's buttons pick autonomous or driver: a
                      field controller (the radio reports field control)
                      or a competition switch. For testing pre-auton UI
                      such as an autonomous selector. With --view it opens
                      the field-control page (/field) in the browser
                      (V5SIM_NO_BROWSER=1 skips that)
--sock PATTERN        chardev socket per port (default /tmp/v5port{port}.sock)
--trace P[,P...]      print each brain frame on these ports (02 heartbeats
                      are counted, not printed)
--view PORT           serve a live field view at http://localhost:PORT/

--battery PCT         start the robot battery at PCT % (default 100): its
                      voltage sags under load and as it drains, and no motor
                      gets more than it gives; the brain shows it too
--no-battery          an ideal supply (every motor always gets its volts)
--physics builtin|mujoco
                      the robot and field physics: builtin (default, pure
                      Python) or MuJoCo (needs the mujoco package, e.g. the
                      ~/venvs/sim venv): real contact, so blocks push each
                      other and the robot, and wheel traction limits launches
--obstacles FILE.json field elements the robot and blocks hit (MuJoCo): a
                      list of {"kind": "box", "x", "y", "sx", "sy", "angle"}
                      or {"kind": "cylinder", "x", "y", "r"}, sim frame m
--opponent X:Y:HEADING[:braked]
                      another robot (18 in, 7 kg) to push or be blocked by
                      (MuJoCo), placed like --start; braked holds until
                      pushed past its friction, else it rolls along its
                      heading but grips sideways. Repeatable

@FILE                 read more arguments from FILE (several per line, #
                      comments), e.g. @robots/competition.args

Physics runs at 1 kHz against the wall clock; QEMU runs close to real time.
"""
from __future__ import annotations

import argparse
import math
import random
import os
import select
import shutil
import subprocess
import socket
import sys
import threading
import time
import traceback
from dataclasses import dataclass, replace
from typing import Any, Callable

from . import brainctl, drivetrain, holonomic, motor, protocol
from .adi_world import AdiWiring, World, adi_sample
from .battery import Battery, BatteryState
from .field_elements import Obstacle, Opponent, load_obstacles
from .apriltags import Camera, FieldTag, load_tags, project
from .drivetrain import INCH, DrivetrainSpec
from .field_objects import BLOCK_RADIUS_M, Field, RobotBody, layout, step_field
from .fieldview import FieldView
from .match import PAUSE_S, MatchSchedule
from .mechanism import ArmLoad, arm_angle_deg, step_arm
from .physics import PhysicsWorld
from .scoring import Goal, load_goals, score
from .smart_adi import SmartAdi
from .smart_device import LinkMode
from .smart_aivision import AiVisionSample, InView, SmartAiVision, TagInView
from .motor import Cartridge, Drive, MotorState
from .smart_controller import (AXIS_BYTES, BUTTONS, Competition,
                               ControllerState, SmartController)
from .smart_gps import GpsSample, SmartGps
from .smart_imu import ImuSample, SmartImu
from .smart_optical import GESTURES, OpticalSample, SmartOptical, raw_for_rgb
from .smart_radio import RadioSample, SmartRadio, status_byte
from .smart_vision import SmartVision, VisionSample, VisionView, uv_for_rgb
from .smart_misc import (TOWER_CHANNELS, SmartLed, SmartLightTower,
                         SmartMagnet, SmartPneumatic)
from .firmware_motor import FirmwareMotor
from .smart_motor import COUNTS_PER_SHAFT_REV, SmartMotor
from .smart_sensors import (DistanceSample, RotationSample, SmartDistance,
                            SmartRotation, ray_to_square_wall)

DT = 0.001
CARTRIDGES = {"red": Cartridge.RED_36, "green": Cartridge.GREEN_18,
              "blue": Cartridge.BLUE_6,
              "5.5w": Cartridge.MOTOR_5_5W}          # the 5.5 W motor (200 rpm)
CONNECT_TIMEOUT_S = 30
RECONNECT_S = 0.5                # retry a closed port this often
SEND_TIMEOUT_S = 0.010           # QEMU not reading a port: drop, don't stall
STALL_HOLDOFF_S = 0.100          # then drop that port's frames this long
ALL_DOWN_EXIT_S = 30.0           # give up once every port is gone this long
TRACKING_WHEEL_DIAMETER_M = 2.75 * INCH
# Inside of the 12 ft perimeter, as measured for SRA's V5RC auton template
# Runs the field-control page (/field) starts: (autonomous s, driver s).
FIELD_RUNS = {"match": (15.0, 105.0), "skills_driver": (0.0, 60.0),
              "skills_auton": (60.0, 0.0)}
FIELD_COMMANDS = (*FIELD_RUNS, "stop")

# (field_half_size = 70.25 in), so wall readings match the course maths.
FIELD_HALF_SIDE_M = 70.25 * INCH
ROBOT_HALF_M = 9 * INCH                        # 18 in robot
FACES = {"front": 0.0, "right": 90.0, "back": 180.0, "left": 270.0}
# Distance Sensor jitter: real readings never repeat exactly (VEX quotes
# +-15 mm under 200 mm, +-5 % beyond); filters like RW-Template's MCL skip
# identical readings, so a noiseless sensor starves them.
DISTANCE_NOISE_MIN_MM = 2.0
DISTANCE_NOISE_FRACTION = 0.005
DISTANCE_MIN_MM = 20                           # closest the sensor reads
# What the optical sensor at the intake sees (SDK rgb, brightness 0..1)
OPTICAL_COLORS = {"red": ((200, 25, 25), 0.30), "blue": ((25, 60, 200), 0.30)}
OPTICAL_FLOOR = ((60, 60, 60), 0.03)
OPTICAL_RANGE_M = 0.10                         # proximity rises inside this
GPS_ERROR_M = 0.01
# AI Vision camera at the robot's front centre, looking straight ahead
AIV_HEIGHT_M = 0.25
AIV_FOCAL_PX = 160 / math.tan(math.radians(74 / 2))   # 74 deg horizontal FOV
AIV_W, AIV_H = 320, 240
# Legacy Vision Sensor (Pixy2 optics) at the same spot; block colours as its
# chroma sees them [I: a red that fits a Vision Utility red signature]
VIS_FOCAL_PX = 158 / math.tan(math.radians(60 / 2))   # 60 deg horizontal FOV
VIS_W, VIS_H = 316, 212
VISION_RGB = {"red": (200, 40, 15), "blue": (20, 60, 200)}
VIEW_PERIOD_S = 0.05
FIELD_STEP_S = 0.010                             # game objects update rate


def mirrored(state: MotorState) -> MotorState:
    return replace(state, position_rad=-state.position_rad,
                   velocity_rad_s=-state.velocity_rad_s,
                   current_a=-state.current_a, voltage_v=-state.voltage_v)


@dataclass
class Port:
    number: int
    sock: socket.socket
    reader: protocol.FrameReader
    firmware: Any                 # SmartMotor, SmartImu, SmartRotation, ...
    sign: int = 1                 # -1: motor mounted mirrored to the plant
    trace: bool = False
    heartbeats: int = 0
    path: str = ""                # chardev socket, for reconnecting
    stalled_until: float = -1.0   # sim time: frames dropped until then
    pending: bytes = b""          # rest of a frame a stalled send cut short
    down_since: float | None = None   # sim time the brain went away
    next_retry: float = 0.0

    def view(self, plant_state: MotorState) -> MotorState:
        return plant_state if self.sign > 0 else mirrored(plant_state)

    def to_plant(self, drive: Drive) -> Drive:
        return None if drive is None else drive * self.sign


@dataclass(frozen=True)
class TrackerMount:
    """An unpowered tracking wheel on a Rotation Sensor: it rolls forward
    (or sideways, positive to the right), touching the floor at (x right,
    y forward) inches from the turning centre."""
    sideways: bool = False
    diameter_in: float = TRACKING_WHEEL_DIAMETER_M / INCH
    x_in: float = 0.0
    y_in: float = 0.0
    flipped: bool = False

    def speed_m_s(self, v_m_s: float, omega_rad_s: float,
                  strafe_m_s: float = 0.0) -> float:
        """Floor speed under the wheel along its rolling direction, for a
        chassis moving forward at v (right at strafe) and yawing
        counter-clockwise at omega."""
        if self.sideways:
            return strafe_m_s - omega_rad_s * self.y_in * INCH
        return v_m_s + omega_rad_s * self.x_in * INCH


@dataclass(frozen=True)
class DistanceMount:
    """Where a Distance Sensor sits, in the convention of SRA's auton
    template: face clockwise from the robot's front (0 front, 90 right,
    180 back, 270 left), offset (x right, y forward) in inches."""
    port: int
    face_deg: float = 0.0
    x_in: float = 0.0
    y_in: float = ROBOT_HALF_M / INCH


@dataclass
class Sensor:
    port: Port
    sample: Callable[[], Any]     # what the device senses right now


def connect(path: str) -> socket.socket:
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while True:
        try:
            s = socket.socket(socket.AF_UNIX)
            s.connect(path)
            s.settimeout(SEND_TIMEOUT_S)
            return s
        except (FileNotFoundError, ConnectionRefusedError):
            s.close()
            if time.monotonic() > deadline:
                raise
            time.sleep(0.2)


def parse_controller_input(msg: dict, current: ControllerState) -> ControllerState:
    """Validate a /input message (see fieldview.py) into controller state."""
    raw_axes = msg.get("axes") or {}
    raw_buttons = msg.get("buttons") or []
    if not isinstance(raw_axes, dict) or not isinstance(raw_buttons, list):
        raise ValueError("axes must be an object, buttons a list")
    try:
        axes = {name: float(v) for name, v in raw_axes.items()}
    except OverflowError as e:
        raise ValueError(str(e)) from e
    if not all(math.isfinite(v) for v in axes.values()):
        raise ValueError("axis values must be finite")
    buttons = frozenset(str(b) for b in raw_buttons)
    if not set(axes) <= set(AXIS_BYTES) or not buttons <= set(BUTTONS):
        raise ValueError("unknown axis or button")
    competition = (Competition[msg["competition"]] if "competition" in msg
                   else current.competition)
    return current.with_input({k: round(v) for k, v in axes.items()},
                              buttons, competition)


class Robot:
    def __init__(self, free_ports: list[Port],
                 drive: tuple[DrivetrainSpec, list[Port], list[Port]] | None):
        self.free_ports = free_ports
        self.free_states = [MotorState() for _ in free_ports]
        self.arms: dict[int, ArmLoad] = {}      # motor port -> its arm (--arm)
        self.battery: Battery | None = None     # None = an ideal supply
        self.battery_state = BatteryState()
        self._battery_sent = -1.0
        self.drive = drive
        self.dt_state = drivetrain.initial_state(drive[0]) if drive else None
        self.sensors: list[Sensor] = []
        self.controller = ControllerState()
        self._controller_lock = threading.Lock()
        self._forward_accel = 0.0
        self.partner: ControllerState | None = None
        self.tower_pressed = False
        self.qmp_path = ""                # the brain's QMP socket (--qmp)
        self._now = 0.0
        self._phase = (Competition.NONE, 0.0)        # competition, since
        self._last_active = Competition.NONE         # last auton/driver phase
        self.travel_m = 0.0
        self._trackers: list[tuple[TrackerMount, float]] = []   # (mount, travel m)
        self.match: MatchSchedule | None = None
        self.field_control = False        # radio reports a field, not a switch
        self.match_kind = ""              # the field page's run: match, skills_*
        self.field: Field | None = None
        self.intake: tuple[int, int] | None = None   # (motor port, capacity)
        self.goals: tuple[Goal, ...] = ()
        self.obstacles: tuple[Obstacle, ...] = ()
        self.opponents: tuple[Opponent, ...] = ()
        self.tags: tuple[FieldTag, ...] = ()
        self.sensor_noise = True
        self._noise = random.Random(0x5EED)       # reproducible runs
        self._field_dt = 0.0
        self.world: PhysicsWorld | None = None    # None = builtin physics

    @property
    def ports(self) -> list[Port]:
        ports = list(self.free_ports)
        if self.drive:
            ports += self.drive[1] + self.drive[2]
        return ports + [s.port for s in self.sensors]

    def port_states(self) -> list[tuple[Port, MotorState]]:
        pairs = list(zip(self.free_ports, self.free_states))
        if self.drive:
            pairs += list(zip(self.drive[1], self.dt_state.left))
            pairs += list(zip(self.drive[2], self.dt_state.right))
        return [(p, p.view(s)) for p, s in pairs]

    def motor_state(self, number: int) -> MotorState:
        for p, s in self.port_states():
            if p.number == number:
                return s
        raise SystemExit(f"no motor on port {number}")

    # ---- what each sensor senses --------------------------------------

    def imu_sample(self) -> ImuSample:
        """A flat IMU on the chassis: VEX heading and gyro z are clockwise-
        positive; the plant's heading is counter-clockwise."""
        if not self.dt_state:
            return ImuSample()
        st = self.dt_state
        # Accelerometer in the sensor's frame, x forward, z up (flat reads
        # +1 g on z, imu-protocol.md 2.2), so y points left: forward speed
        # change on x, the turn's centripetal acceleration v * omega on y.
        lateral = st.v_m_s * st.omega_rad_s
        return ImuSample(heading_deg=-math.degrees(st.pose.heading_rad),
                         gyro_dps=(0.0, 0.0, -math.degrees(st.omega_rad_s)),
                         accel_g=(self._forward_accel / drivetrain.G,
                                  lateral / drivetrain.G, 1.0))

    def tracker_sample(self, mount: TrackerMount) -> Callable[[], RotationSample]:
        """Attach a tracking wheel; the sampler reads its shaft angle."""
        i = len(self._trackers)
        self._trackers.append((mount, 0.0))
        sign = -1 if mount.flipped else 1

        def sample() -> RotationSample:
            turns = self._trackers[i][1] / (math.pi * mount.diameter_in * INCH)
            return RotationSample(shaft_deg=sign * turns * 360)
        return sample

    def _advance_trackers(self, dt: float) -> None:
        st = self.dt_state
        self._trackers = [(m, d + m.speed_m_s(st.v_m_s, st.omega_rad_s, st.strafe_m_s) * dt)
                          for m, d in self._trackers]

    def motor_shaft_sample(self, number: int) -> Callable[[], RotationSample]:
        return lambda: RotationSample(
            shaft_deg=self.motor_state(number).position_deg)

    def optical_sample(self) -> OpticalSample:
        """An optical sensor at the intake, facing forward: it sees the
        newest held block, else a block right at the front, else floor."""
        color, prox = None, 0
        if self.field is not None and self.field.held:
            color, prox = self.field.held[-1], 255
        elif self.field is not None and self.dt_state:
            pose = self.dt_state.pose
            c, s = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
            best = OPTICAL_RANGE_M
            for b in self.field.blocks:
                dx, dy = b.x - pose.x_m, b.y - pose.y_m
                gap = c * dx + s * dy - ROBOT_HALF_M - BLOCK_RADIUS_M
                if 0 <= gap < best and abs(-s * dx + c * dy) < 2 * BLOCK_RADIUS_M:
                    best, color = gap, b.color
            if color:
                prox = round(255 * (1 - best / OPTICAL_RANGE_M))
        rgb, bright = OPTICAL_COLORS[color] if color else OPTICAL_FLOOR
        fw = self._firmware(SmartOptical)       # counts scale with its ATIME
        raw = raw_for_rgb(rgb, bright, fw.atime if fw else SmartOptical().atime)
        return OpticalSample(*raw, proximity=prox)

    def aivision_sample(self) -> AiVisionSample:
        """Blocks on the floor ahead and field AprilTags, projected into
        the 320x240 image."""
        if not self.dt_state:
            return AiVisionSample()
        pose = self.dt_state.pose
        cam = Camera(AIV_FOCAL_PX, AIV_W, AIV_H, AIV_HEIGHT_M, ROBOT_HALF_M)
        tags = tuple(TagInView(t.id, corners, t.family) for t in self.tags
                     if (corners := project(t, pose, cam)) is not None)
        seen = tuple(InView(OPTICAL_COLORS.get(color, OPTICAL_FLOOR)[0], *box, color)
                     for color, box in self._blocks_in_image(AIV_W, AIV_H, AIV_FOCAL_PX))
        return AiVisionSample(seen, tags)

    def vision_sample(self) -> VisionSample:
        """Blocks ahead in the legacy Vision Sensor's 316x212 image."""
        return VisionSample(tuple(
            VisionView(uv_for_rgb(VISION_RGB.get(color, (60, 60, 60))), *box)
            for color, box in self._blocks_in_image(VIS_W, VIS_H, VIS_FOCAL_PX)))

    def _blocks_in_image(self, width: int, height: int, focal: float
                         ) -> list[tuple[str, tuple[int, int, int, int]]]:
        """Blocks on the floor ahead of a camera at the robot's front:
        (colour, (x, y, w, h)) boxes clipped to the image."""
        if self.field is None or not self.dt_state:
            return []
        pose = self.dt_state.pose
        c, s = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
        seen = []
        for b in self.field.blocks:
            dx, dy = b.x - pose.x_m, b.y - pose.y_m
            d = c * dx + s * dy - ROBOT_HALF_M          # ahead of the camera
            lateral = -s * dx + c * dy                   # + = left
            if d < 0.05:
                continue
            size = focal * 2 * BLOCK_RADIUS_M / d
            cx = width / 2 - focal * lateral / d
            cy = height / 2 + focal * (AIV_HEIGHT_M - BLOCK_RADIUS_M) / d
            x0, y0 = cx - size / 2, cy - size / 2
            if x0 + size < 0 or x0 > width or y0 > height:
                continue
            x1, y1 = min(x0 + size, width), min(y0 + size, height)
            x0, y0 = max(x0, 0), max(y0, 0)
            if round(x1 - x0) < 1 or round(y1 - y0) < 1:
                continue                                  # a sliver at the edge
            seen.append((b.color, (round(x0), round(y0), round(x1 - x0),
                                   round(y1 - y0))))
        return seen

    def gps_sample(self) -> GpsSample:
        """The camera's pose solve: GPS heading is 0 facing +y, clockwise
        (like the IMU) [I: convention not confirmed on hardware]."""
        if not self.dt_state:
            return GpsSample(quality=0xFF)
        st = self.dt_state
        heading = math.radians((90 - math.degrees(st.pose.heading_rad)) % 360)
        return GpsSample(x_m=st.pose.x_m, y_m=st.pose.y_m, yaw_rad=heading,
                         error_m=GPS_ERROR_M,
                         gyro_z_dps=-math.degrees(st.omega_rad_s))

    def distance_sample(self) -> DistanceSample:
        """A Distance Sensor on the front face, facing forward."""
        return self.distance_sample_for(DistanceMount(0))

    def distance_sample_for(self, m: DistanceMount) -> DistanceSample:
        """The beam from the sensor's mount to the field wall it hits."""
        if not self.dt_state:
            return DistanceSample()
        pose = self.dt_state.pose
        h = pose.heading_rad                       # counter-clockwise from +x
        right, fwd = (math.sin(h), -math.cos(h)), (math.cos(h), math.sin(h))
        lim = FIELD_HALF_SIDE_M - 0.001
        sx = max(-lim, min(lim, pose.x_m + (m.x_in * right[0] + m.y_in * fwd[0]) * INCH))
        sy = max(-lim, min(lim, pose.y_m + (m.x_in * right[1] + m.y_in * fwd[1]) * INCH))
        mm = ray_to_square_wall(sx, sy, h - math.radians(m.face_deg), FIELD_HALF_SIDE_M) * 1000
        if self.sensor_noise and math.isfinite(mm):
            mm += self._noise.gauss(0.0, max(DISTANCE_NOISE_MIN_MM,
                                             DISTANCE_NOISE_FRACTION * mm))
        return DistanceSample(distance_mm=max(mm, DISTANCE_MIN_MM))

    def adi_world(self) -> World:
        shafts = {p.number: s.position_deg for p, s in self.port_states()}
        if not self.dt_state:
            return World(shaft_deg=shafts)
        st = self.dt_state
        gap = ray_to_square_wall(st.pose.x_m, st.pose.y_m, st.pose.heading_rad,
                                 FIELD_HALF_SIDE_M) - ROBOT_HALF_M
        return World(front_gap_m=max(gap, 0.0), travel_m=self.travel_m,
                     speed_m_s=st.v_m_s, accel_m_s2=self._forward_accel,
                     heading_deg=-math.degrees(st.pose.heading_rad),
                     shaft_deg=shafts)

    def _firmware(self, cls: type) -> Any:
        for s in self.sensors:
            if isinstance(s.port.firmware, cls):
                return s.port.firmware
        return None

    def controller_firmware(self) -> SmartController | SmartRadio | None:
        return self._firmware(SmartController) or self._firmware(SmartRadio)

    def radio_sample(self) -> RadioSample:
        """The wireless controller(s) behind the radio; a --match runs as
        field control, with byte 7 counting the seconds of each phase. On a
        competition switch the controller's clock also restarts with each
        program run, so VEXos's banner timer does too."""
        c = self.controller_sample()
        phase, since = self._phase
        radio = self._firmware(SmartRadio)
        if not self.match and radio and radio.program_started is not None:
            since = max(since, radio.program_started)
        auton_mode = (phase == Competition.AUTONOMOUS
                      or (phase == Competition.DISABLED
                          and self._last_active == Competition.AUTONOMOUS))
        secs = int(self._now - since) if phase != Competition.NONE else 0
        with self._controller_lock:
            partner = self.partner
        return RadioSample(c, partner, status_byte(phase, bool(self.match) or self.field_control, auton_mode),
                           secs)

    def _track_phase(self, now: float) -> None:
        phase = self.controller_sample().competition
        if phase != self._phase[0]:
            if self._phase[0] in (Competition.AUTONOMOUS, Competition.DRIVER):
                self._last_active = self._phase[0]
            self._phase = (phase, now)

    def controller_sample(self) -> ControllerState:
        with self._controller_lock:
            return self.controller

    def set_input(self, msg: dict) -> None:
        """A /input message: controller sticks and buttons, plus
        "gesture" (optical sensors), "tower" (Light Tower button) and
        "partner" (the radio's second controller). The whole message is
        checked before any of it applies."""
        press = msg.get("brain_button")
        if press is not None:
            if (isinstance(press, bool) or not isinstance(press, (int, float))
                    or not 0 < press <= brainctl.MAX_PRESS_S):
                raise ValueError(f"brain_button is a press length, 0..{brainctl.MAX_PRESS_S:g} s")
            if not self.qmp_path:
                raise ValueError("no brain to press: the sim needs --qmp")
        gesture = msg.get("gesture")
        if gesture is not None and gesture not in GESTURES:
            raise ValueError(f"gesture must be one of {', '.join(GESTURES)}")
        tower = msg.get("tower")
        if "tower" in msg and not isinstance(tower, bool):
            raise ValueError("tower must be true or false")
        command = msg.get("field")
        if command is not None and command not in FIELD_COMMANDS:
            raise ValueError(f"field must be one of {', '.join(FIELD_COMMANDS)}")
        sub = msg.get("partner")
        if "partner" in msg and not (isinstance(sub, dict)
                                     and set(sub) <= {"axes", "buttons"}):
            raise ValueError("partner must be an object of axes and buttons")
        if command is not None:
            self.field_command(command)
        with self._controller_lock:
            if sub is not None and self.partner is None:
                raise ValueError("no partner controller (--radio P:partner)")
            partner = (parse_controller_input(sub, self.partner) if sub is not None
                       else self.partner)
            controller = self.controller
            if {"axes", "buttons", "competition"} & set(msg):
                if self.match:        # the schedule owns the competition state
                    msg = {k: v for k, v in msg.items() if k != "competition"}
                controller = parse_controller_input(msg, self.controller)
            self.partner, self.controller = partner, controller
            if tower is not None:
                self.tower_pressed = tower
        if gesture is not None:
            for fw in self._all_firmware(SmartOptical):
                fw.request_gesture(gesture)
        if press is not None:
            brainctl.press_async(self.qmp_path, float(press))

    def tower_sample(self) -> bool:
        """The Light Tower button, held from the field view (kept here so a
        device reboot doesn't release it)."""
        with self._controller_lock:
            return self.tower_pressed

    def _all_firmware(self, cls: type) -> list[Any]:
        return [s.port.firmware for s in self.sensors
                if isinstance(s.port.firmware, cls)]

    def start_competition(self, field: bool) -> None:
        """Plug into competition control, disabled until the field view's
        buttons (or /input) pick autonomous or driver. field=True reports a
        field controller over the radio, otherwise a competition switch."""
        self.field_control = field
        with self._controller_lock:
            self.controller = replace(self.controller, competition=Competition.DISABLED)

    def field_command(self, command: str) -> None:
        """The field-control page: start a match or skills run now (the
        schedule then owns the competition state), or stop it, leaving
        the robot disabled."""
        if command not in FIELD_COMMANDS:
            raise ValueError(f"field must be one of {', '.join(FIELD_COMMANDS)}")
        self.field_control = True
        if command == "stop":
            self.match, self.match_kind = None, ""
            with self._controller_lock:
                self.controller = replace(self.controller, competition=Competition.DISABLED)
            return
        auton, driver = FIELD_RUNS[command]
        self.match, self.match_kind = MatchSchedule(self._now, auton, driver), command

    def field_snapshot(self, now: float) -> dict:
        """What the field-control page shows: the competition state, the
        running schedule and the robot's radio link."""
        m = self.match
        match = None if m is None else {
            "start": m.start_s, "auton": m.auton_s, "driver": m.driver_s,
            "pause": PAUSE_S if m.auton_s and m.driver_s else 0.0, "end": m.end_s}
        radio = self._firmware(SmartRadio)
        link = None
        if radio is not None:
            status = radio.robot_status
            link = {"linked": radio.link == LinkMode.DATA,
                    "running": radio.program_running,
                    "battery": status[2] if status else None}
        return {"t": now, "competition": self.controller_sample().competition.name,
                "field_control": self.field_control or bool(m),
                "kind": self.match_kind if m else "", "match": match, "radio": link}

    def set_controller_input(self, msg: dict) -> None:
        if self.match:            # the schedule owns the competition state
            msg = {k: v for k, v in msg.items() if k != "competition"}
        with self._controller_lock:
            self.controller = parse_controller_input(msg, self.controller)

    def _follow_match(self, now: float) -> None:
        phase = self.match.phase_at(now)
        with self._controller_lock:
            if phase == self.controller.competition:
                return
            self.controller = replace(self.controller, competition=phase)
        print(f"{now:7.2f}s match: {phase.name}", flush=True)

    # ---- simulation ---------------------------------------------------

    def step(self, now: float) -> None:
        self._now = now
        if self.match:
            self._follow_match(now)
        self._track_phase(now)
        self._step_battery()
        self.free_states = [self._step_free(p, s, now)
                            for p, s in zip(self.free_ports, self.free_states)]
        if self.drive:
            spec, left, right = self.drive
            st = self.dt_state

            def drives(ports, states):
                return [self._supply(p.to_plant(p.firmware.drive(p.view(s), now, DT)))
                        for p, s in zip(ports, states)]

            limits = [p.firmware.effective_current_limit_a for p in left + right]
            dl, dr = drives(left, st.left), drives(right, st.right)
            if self.world is not None:
                self.dt_state = self.world.step(dl, dr, DT, limits, *self._intake_drive())
            elif spec.holonomic:
                self.dt_state = holonomic.constrain_to_square(
                    spec, holonomic.step(spec, st, dl, dr, DT, limits),
                    FIELD_HALF_SIDE_M, ROBOT_HALF_M)
            else:
                self.dt_state = drivetrain.constrain_to_square(
                    spec, drivetrain.step(spec, st, dl, dr, DT, limits),
                    FIELD_HALF_SIDE_M, ROBOT_HALF_M, (dl, dr), DT, limits)
            self._forward_accel = (self.dt_state.v_m_s - st.v_m_s) / DT
            self.travel_m += self.dt_state.v_m_s * DT
            self._advance_trackers(DT)
        if self.field is not None and self.dt_state:
            self._field_dt += DT
            if self._field_dt >= FIELD_STEP_S - 1e-9:
                self._step_field(self._field_dt)
                self._field_dt = 0.0
        for s in self.sensors:
            if isinstance(s.port.firmware, SmartRotation):
                s.port.firmware.sample(now, s.sample())

    def use_mujoco(self) -> None:
        """Hand the drivetrain and the blocks to a MuJoCo world, starting
        from the current pose and field."""
        from .physics_mujoco import MujocoWorld
        self.world = MujocoWorld(self.drive[0], FIELD_HALF_SIDE_M, ROBOT_HALF_M,
                                 self.dt_state.pose, self.field,
                                 self.obstacles, self.opponents)

    def opponent_poses(self) -> list[drivetrain.Pose]:
        """Where the opponents are now (MuJoCo moves them)."""
        if self.world is not None and hasattr(self.world, "opponents"):
            return self.world.opponents()
        return [o.pose for o in self.opponents]

    def enable_battery(self, charge: float = 1.0) -> None:
        self.battery = Battery()
        self.battery_state = self.battery.step(BatteryState(charge=charge), 0.0, 0.0)

    def _supply(self, drive: Drive) -> Drive:
        """A motor's voltage, capped by what the battery gives."""
        return drive if self.battery is None else Battery.limit_drive(drive, self.battery_state)

    def _step_battery(self) -> None:
        if self.battery is None:
            return
        for p in self.ports:          # emulated boards measure the battery
            if isinstance(p.firmware, FirmwareMotor):
                p.firmware.battery_v = self.battery_state.terminal_v
        load_w = sum(max(0.0, s.voltage_v * s.current_a) for _, s in self.port_states())
        self.battery_state = self.battery.step(self.battery_state, load_w, DT)

    def _step_free(self, p: Port, s: MotorState, now: float) -> MotorState:
        """A motor off the drivetrain: a light shaft, or an arm (--arm)."""
        drive = self._supply(p.to_plant(p.firmware.drive(p.view(s), now, DT)))
        limit = p.firmware.effective_current_limit_a
        arm = self.arms.get(p.number)
        if arm is not None:
            return step_arm(s, drive, DT, p.firmware.cartridge, arm, limit)
        return motor.step_shaft(s, drive, DT, p.firmware.cartridge, limit_a=limit)

    def _intake_drive(self) -> tuple[float, int]:
        """(intake shaft rad/s, + = inward whatever the mount; capacity)."""
        port, capacity = self.intake or (0, 0)
        for p, s in zip(self.free_ports, self.free_states):
            if p.number == port:
                return s.velocity_rad_s, capacity     # plant side
        return 0.0, capacity

    def _step_field(self, dt: float) -> None:
        if self.world is not None:
            self.field = self.world.field()       # MuJoCo moves the blocks
            return
        pose = self.dt_state.pose
        speed, capacity = self._intake_drive()
        body = RobotBody(pose.x_m, pose.y_m, pose.heading_rad, ROBOT_HALF_M,
                         speed, capacity)
        self.field = step_field(self.field, body, dt, FIELD_HALF_SIDE_M)

    def _disconnect(self, port: Port, now: float) -> None:
        print(f"{now:7.2f}s p{port.number}: brain closed the port, retrying",
              file=sys.stderr, flush=True)
        port.sock.close()
        port.sock, port.down_since, port.next_retry = None, now, now + RECONNECT_S
        port.reader = protocol.FrameReader()
        port.pending = b""
        port.firmware.reboot(now)

    def _reconnect(self, now: float) -> None:
        for port in self.ports:
            if port.sock or not port.path or now < port.next_retry:
                continue
            port.next_retry = now + RECONNECT_S
            s = socket.socket(socket.AF_UNIX)
            try:
                s.connect(port.path)
            except OSError:
                s.close()
                continue
            s.settimeout(SEND_TIMEOUT_S)
            port.sock, port.down_since = s, None
            print(f"{now:7.2f}s p{port.number}: reconnected", file=sys.stderr,
                  flush=True)

    def all_down_for(self, now: float) -> float:
        """How long every port has been disconnected (0 if any is up)."""
        downs = [p.down_since for p in self.ports]
        if not downs or any(d is None for d in downs):
            return 0.0
        return now - max(downs)

    def _send(self, port: Port, frame: bytes, now: float) -> None:
        """Send a frame; when the brain side isn't reading, drop whole new
        frames for a while (like a busy wire) instead of freezing every port
        and the physics, but always finish a frame that was started."""
        if not port.sock or now < port.stalled_until:
            return                    # holding off: nothing may block here
        started = port.pending        # a frame already partly on the wire
        data = started + frame
        if not data:
            return
        try:
            sent = port.sock.send(data)
        except TimeoutError:
            sent = 0
        except OSError:
            self._disconnect(port, now)
            return
        if sent < len(started):
            port.pending = started[sent:]       # new frame never started: dropped
        elif sent == len(started):
            port.pending = b""                  # likewise
        else:
            port.pending = data[sent:]          # new frame partly sent: finish later
        if sent < len(data):
            port.stalled_until = now + STALL_HOLDOFF_S
            print(f"{now:7.2f}s p{port.number}: brain not reading, dropping frames "
                  f"for {STALL_HOLDOFF_S * 1000:.0f} ms", file=sys.stderr, flush=True)

    def service_io(self, now: float, timeout: float) -> None:
        self._reconnect(now)
        by_sock = {p.sock: p for p in self.ports if p.sock}
        if not by_sock:
            time.sleep(max(timeout, 0))
            return
        readable, _, _ = select.select(list(by_sock), [], [], max(timeout, 0))
        for s in readable:
            port = by_sock[s]
            try:
                data = s.recv(4096)
            except OSError:
                data = b""
            if not data:
                self._disconnect(port, now)
                continue
            port.reader.push(data)
            while True:
                # Devices change their frame length mid-stream (02 13).
                port.reader.length = getattr(port.firmware, "frame_len",
                                             protocol.FRAME_LEN)
                frame = port.reader.next_frame()
                if frame is None:
                    break
                if port.trace:
                    self._trace(port, frame, now)
                try:
                    replies = port.firmware.on_brain_frame(frame, now)
                except Exception:        # a model bug: drop the frame, keep going
                    print(f"p{port.number}: error handling {frame.hex(' ')}",
                          file=sys.stderr)
                    traceback.print_exc()
                    continue
                for reply in replies:
                    self._send(port, reply, now)
        for port, state in self.port_states():
            if port.sock:
                for frame in port.firmware.due_frames(now, state):
                    self._send(port, frame, now)
        for sensor in self.sensors:
            if sensor.port.sock:
                for frame in sensor.port.firmware.due_frames(now, sensor.sample()):
                    self._send(sensor.port, frame, now)

    @staticmethod
    def _trace(port: Port, frame: bytes, now: float) -> None:
        if frame[0] == 0x02:
            port.heartbeats += 1
            return
        print(f"{now:8.3f}s p{port.number} < {frame.hex(' ')} "
              f"(+{port.heartbeats} hb)", flush=True)
        port.heartbeats = 0

    def report_battery(self, now: float) -> None:
        """Once a second, tell the brain's NXP model the battery's state."""
        if self.battery is None or not self.qmp_path or now - self._battery_sent < 1.0:
            return
        self._battery_sent = now
        brainctl.set_battery_async(self.qmp_path, self.battery_state.terminal_v,
                                   self.battery_state.current_a,
                                   self.battery_state.percent)

    def _plant_state(self, number: int) -> MotorState:
        for p, s in zip(self.free_ports, self.free_states):
            if p.number == number:
                return s
        raise KeyError(number)

    def device_snapshot(self, now: float) -> list[dict]:
        """Every device as the dashboard shows it: motors with their
        measured state and command (as the brain sees them), sensors with
        their reading."""
        roles = {p.number: "motor" for p in self.free_ports}
        if self.intake:
            roles[self.intake[0]] = "intake"
        if self.drive:
            roles.update({p.number: "drive left" for p in self.drive[1]})
            roles.update({p.number: "drive right" for p in self.drive[2]})
        out = []
        for p, st in self.port_states():
            fw = p.firmware
            c = fw.command
            counts_per_deg = getattr(fw, "brain_counts_per_rev",
                                     COUNTS_PER_SHAFT_REV * fw.cartridge.ratio) / 360
            out.append({
                "port": p.number, "kind": "motor", "role": roles.get(p.number, "motor"),
                "link": fw.link.name, "cartridge": fw.cartridge.name.lower(),
                "position_deg": round(st.position_deg, 2),
                "velocity_rpm": round(st.velocity_rpm, 2),
                "voltage_v": round(st.voltage_v, 3), "current_a": round(st.current_a, 3),
                "power_w": round(st.power_w, 2), "temperature_c": round(st.temperature_c, 2),
                "limited": st.current_limited, "faults": fw.faults(st),
                "mode": c.mode.name.lower(),
                **({"arm_deg": round(arm_angle_deg(self.arms[p.number],
                                                   self._plant_state(p.number)), 2)}
                   if p.number in self.arms else {}),
                "brake": c.brake.name.lower(),
                "target_deg": round(c.target / counts_per_deg, 2),
                "target_rpm": round(c.velocity / counts_per_deg / 6, 2),
                "percent": c.percent})
        for sensor in self.sensors:
            fw = sensor.port.firmware
            item = {"port": sensor.port.number,
                    "kind": type(fw).__name__.removeprefix("Smart").lower(),
                    "link": fw.link.name, "text": self.sensor_detail(sensor, now).strip()}
            if isinstance(fw, SmartImu):
                item["heading_deg"] = round(self.imu_sample().heading_deg % 360, 2)
            elif isinstance(fw, SmartRotation):
                item["position_deg"] = round(fw.position_cdeg / 100, 2)
            elif isinstance(fw, SmartDistance):
                mm = sensor.sample().distance_mm
                item["distance_mm"] = None if mm is None else round(mm, 1)
            out.append(item)
        if self.battery is not None:
            b = self.battery_state
            out.append({"port": 0, "kind": "battery", "link": "DATA",
                        "volts": round(b.terminal_v, 2), "amps": round(b.current_a, 2),
                        "percent": b.percent,
                        "text": f"{b.terminal_v:5.2f} V {b.current_a:5.1f} A {b.percent}%"})
        return sorted(out, key=lambda d: d["port"])

    def sensor_detail(self, s: Sensor, now: float) -> str:
        """What a non-motor device reads now, for the status line and the
        dashboard."""
        fw = s.port.firmware
        if isinstance(fw, SmartImu):
            detail = f"{self.imu_sample().heading_deg % 360:5.1f}°"
        elif isinstance(fw, SmartRotation):
            detail = f"{fw.position_cdeg / 100:8.1f}°"
        elif isinstance(fw, SmartDistance):
            mm = s.sample().distance_mm
            detail = f"{mm:5.0f}mm" if mm is not None else "  ---"
        elif isinstance(fw, SmartOptical):
            o = s.sample()
            detail = f"prox {o.proximity:3d} rgbc {o.red}/{o.green}/{o.blue}/{o.clear}"
        elif isinstance(fw, SmartGps):
            g = s.sample()
            detail = f"({g.x_m:+.2f}, {g.y_m:+.2f}) {math.degrees(g.yaw_rad):5.1f}°"
        elif isinstance(fw, SmartAiVision):
            view = s.sample()
            detail = (f"sees {len(view.in_view)} reports "
                      f"{len(fw.detect(view))} cfg {fw.cfg_byte:02x} "
                      f"sigs {sorted(fw.signatures)}")
        elif isinstance(fw, SmartVision):
            detail = (f"sees {len(s.sample().in_view)} reports "
                      f"{len(fw.detect(s.sample()))} sigs {sorted(fw.signatures)}")
        elif isinstance(fw, SmartRadio):
            r = s.sample()
            detail = (f"{r.controller.competition.name} status {r.status:02x} "
                      f"t={r.match_secs}s {dict(r.controller.axes)} "
                      f"{sorted(r.controller.buttons)}")
            if r.partner:
                detail += f" partner {dict(r.partner.axes)} {sorted(r.partner.buttons)}"
            if any(fw.screen_lines()) or fw.rumble:
                detail += f" screen {'|'.join(fw.screen_lines())!r}"
        elif isinstance(fw, SmartAdi):
            detail = _describe_adi(fw)
        elif isinstance(fw, (SmartMagnet, SmartLightTower, SmartPneumatic,
                             SmartLed)):
            detail = _describe_misc(fw, now)
        else:
            c = self.controller_sample()
            detail = f"{c.competition.name} {dict(c.axes)} {sorted(c.buttons)}"
            if fw.field_team is not None:
                detail = f"field team {fw.field_team} state {fw.field_state:02x}"
            if any(fw.screen_lines()) or fw.rumble:
                detail += f" screen {'|'.join(fw.screen_lines())!r}"
                detail += f" rumble {fw.rumble!r}" if fw.rumble else ""
        return detail

    def describe(self, now: float = 0.0) -> str:
        parts = [f"p{p.number}[{p.firmware.link.name}] "
                 f"{s.velocity_rpm:6.1f}rpm {s.voltage_v:5.2f}V"
                 for p, s in self.port_states()]
        for s in self.sensors:
            fw, p = s.port.firmware, s.port
            parts.append(f"{type(fw).__name__[5:].lower()} p{p.number}"
                         f"[{fw.link.name}] {self.sensor_detail(s, now)}")
        if self.field is not None:
            parts.append(f"blocks {len(self.field.blocks)} held "
                         f"[{' '.join(self.field.held)}]")
            if self.goals:
                s = score(self.field, self.goals)
                parts.append(f"score red {s.red} blue {s.blue}")
        if self.dt_state:
            pose = self.dt_state.pose
            rx, ry, rh = rw_pose(pose)
            parts.insert(0, f"pose x={pose.x_m:+.3f} y={pose.y_m:+.3f} "
                            f"h={math.degrees(pose.heading_rad):+.1f}° "
                            f"rw=({rx:+.1f}, {ry:+.1f}, {rh:.0f})")
        return " | ".join(parts)


def _describe_adi(fw: SmartAdi) -> str:
    outs = []
    for p in range(len(fw.config)):
        level, pwm = fw.digital_out(p), fw.pwm(p)
        if level is not None:
            outs.append(f"{chr(65 + p)}={int(level)}")
        elif pwm is not None:
            outs.append(f"{chr(65 + p)}={pwm:+d}")
    for p in range(len(fw.config)):
        lit = [c for c in fw.led_strip(p) if any(c)]
        if lit:
            outs.append(f"{chr(65 + p)} leds {len(lit)} lit, first "
                        "#{:02x}{:02x}{:02x}".format(*lit[0]))
    return "cfg " + fw.config.hex() + (" out " + " ".join(outs) if outs else "")


def _describe_misc(fw: Any, now: float) -> str:
    if isinstance(fw, SmartMagnet):
        return f"power {fw.power:+d} {fw.current_ma}mA {fw.temp_c:.1f}C"
    if isinstance(fw, SmartLightTower):
        lights = " ".join(f"{name}={v}" + ("*" if fw.blink_mask >> i & 1 else "")
                          for i, (name, v) in enumerate(zip(TOWER_CHANNELS, fw.values)))
        return f"{lights} lit {'/'.join(map(str, fw.lit(now)))}"
    if isinstance(fw, SmartPneumatic):
        cyl = "".join("E" if e else "r" for e in fw.extended)
        return (f"pump {'on' if fw.pump else 'off'} cyl {cyl} "
                f"count {'/'.join(map(str, fw.counts))} pressure {fw.pressure}")
    return "rgb #{:02x}{:02x}{:02x}".format(*fw.rgb)


def run(robot: Robot, seconds: float | None, status_every: float,
        view: FieldView | None = None) -> None:
    t0 = time.monotonic()
    sim_t = 0.0
    next_status = status_every
    next_view = 0.0
    while seconds is None or sim_t < seconds:
        now = time.monotonic() - t0
        while sim_t < now:
            robot.step(sim_t)
            sim_t += DT
        robot.service_io(sim_t, sim_t + DT - (time.monotonic() - t0))
        if robot.all_down_for(sim_t) > ALL_DOWN_EXIT_S:
            raise ConnectionError("the brain is gone (every port closed)")
        robot.report_battery(sim_t)
        if status_every and sim_t >= next_status:
            print(f"{sim_t:7.2f}s {robot.describe(sim_t)}", flush=True)
            next_status += status_every
        if view and sim_t >= next_view:
            pose = robot.dt_state.pose if robot.dt_state else drivetrain.Pose()
            speed = robot.dt_state.v_m_s if robot.dt_state else 0.0
            f = robot.field
            ctrl = robot.controller_firmware()
            points = score(f, robot.goals) if f and robot.goals else None
            view.publish(sim_t, pose.x_m, pose.y_m, pose.heading_rad, speed,
                         blocks=[(b.x, b.y, b.color) for b in f.blocks] if f else (),
                         held=f.held if f else (),
                         screen=ctrl.screen_lines() if ctrl else (),
                         goals=[(g.name, g.rect) for g in robot.goals],
                         score={"red": points.red, "blue": points.blue} if points else None,
                         obstacles=[o.view() for o in robot.obstacles],
                         opponents=[(o.x_m, o.y_m, o.heading_rad)
                                    for o in robot.opponent_poses()])
            view.publish_devices(sim_t, robot.device_snapshot(sim_t))
            view.publish_field(sim_t, robot.field_snapshot(sim_t))
            next_view = sim_t + VIEW_PERIOD_S


def open_in_browser(url: str) -> None:
    """Open url in the Windows browser (WSL) or the desktop's; quietly
    skipped where neither exists or V5SIM_NO_BROWSER=1 (scripted runs)."""
    if os.environ.get("V5SIM_NO_BROWSER") == "1":
        return
    for opener in ("explorer.exe", "xdg-open"):
        if shutil.which(opener):
            try:
                subprocess.Popen([opener, url], stdin=subprocess.DEVNULL,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                 start_new_session=True)
            except OSError as e:
                print(f"couldn't open {url}: {e}", file=sys.stderr)
            return


def _adi_arg(text: str) -> tuple[int, AdiWiring]:
    n, _, wiring = text.partition(":")
    try:
        return int(n), AdiWiring.parse(wiring)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


def _blocks_arg(text: str) -> Field:
    try:
        return layout(text, FIELD_HALF_SIDE_M)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


def _start_arg(text: str) -> drivetrain.Pose:
    """X:Y:HEADING in RW-Template's field coordinates: inches from the field
    centre, heading clockwise from +Y. The sim's +x is RW's +Y (the robot
    faces it at heading 0) and its +y is RW's -X."""
    try:
        x, y, h = (float(v) for v in text.split(":"))
    except ValueError as e:
        raise argparse.ArgumentTypeError("expected X:Y:HEADING (in, in, deg)") from e
    if not all(map(math.isfinite, (x, y, h))):
        raise argparse.ArgumentTypeError("start pose must be finite")
    return drivetrain.Pose(x_m=y * INCH, y_m=-x * INCH, heading_rad=-math.radians(h))


def rw_pose(p: drivetrain.Pose) -> tuple[float, float, float]:
    """A sim pose in RW-Template field coordinates (in, in, deg clockwise)."""
    return -p.y_m / INCH, p.x_m / INCH, -math.degrees(p.heading_rad) % 360


def _distance_arg(text: str) -> DistanceMount:
    """P[:FACE[:X:Y]]: FACE front|right|back|left or degrees clockwise;
    X, Y the offset in inches (right, forward); default: on that face."""
    parts = text.split(":")
    try:
        port = int(parts[0])
        face = parts[1] if len(parts) > 1 else "front"
        deg = FACES[face] if face in FACES else float(face) % 360
        if not math.isfinite(deg):
            raise ValueError("face must be finite")
        if len(parts) == 4:
            x, y = float(parts[2]), float(parts[3])
        elif len(parts) <= 2:
            half = ROBOT_HALF_M / INCH
            x = round(half * math.sin(math.radians(deg)), 6)
            y = round(half * math.cos(math.radians(deg)), 6)
        else:
            raise ValueError("give both X and Y")
    except (ValueError, IndexError) as e:
        raise argparse.ArgumentTypeError(
            "expected P[:FACE[:X:Y]], FACE front|right|back|left|degrees") from e
    return DistanceMount(port, deg, x + 0.0, y + 0.0)


def _tags_arg(text: str) -> tuple[FieldTag, ...]:
    try:
        return load_tags(text)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


def _goals_arg(text: str) -> tuple[Goal, ...]:
    try:
        return load_goals(text)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


def _obstacles_arg(text: str) -> tuple[Obstacle, ...]:
    try:
        return load_obstacles(text)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


def _opponent_arg(text: str) -> Opponent:
    """X:Y:HEADING[:braked], placed like --start."""
    pose, _, braked = text.rpartition(":") if text.endswith(":braked") else (text, "", "")
    return Opponent(_start_arg(pose), braked=bool(braked))


def _arm_arg(text: str) -> tuple[int, ArmLoad]:
    parts = text.split(":")
    try:
        if len(parts) not in (4, 7):
            raise ValueError("expected P:RATIO:LEN:KG[:START:LO:HI]")
        port = int(parts[0])
        ratio, length_in, mass = (float(v) for v in parts[1:4])
        if len(parts) == 7:
            start, lo, hi = (float(v) for v in parts[4:7])
        else:
            start, lo, hi = -90.0, -90.0, 90.0
        return port, ArmLoad(ratio, length_in * INCH, mass, start, lo, hi)
    except ValueError as e:
        raise argparse.ArgumentTypeError(f"--arm {text}: {e}") from e


def _intake_arg(text: str) -> tuple[int, int]:
    port, _, cap = text.partition(":")
    try:
        p, c = int(port), int(cap or 6)
    except ValueError as e:
        raise argparse.ArgumentTypeError("expected P[:CAPACITY]") from e
    if c < 1:
        raise argparse.ArgumentTypeError("the intake's capacity must be at least 1")
    return p, c


def _mounted_port(token: str) -> tuple[int, bool]:
    """"7" or "7r": a motor port, r = mounted the other way round (the
    program reverses it in code)."""
    flipped = token.endswith("r")
    digits = token[:-1] if flipped else token
    if not digits.isdigit():
        raise ValueError(f"bad motor port {token!r}")
    return int(digits), flipped


def _motor_arg(text: str) -> tuple[int, Cartridge, bool]:
    port, _, cart = text.partition(":")
    try:
        n, flipped = _mounted_port(port)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e
    if cart not in CARTRIDGES:
        raise argparse.ArgumentTypeError(
            f"expected P[r]:CART with CART one of {', '.join(CARTRIDGES)}")
    return n, CARTRIDGES[cart], flipped


def _drive_arg(text: str) -> tuple[list[tuple[int, bool]], list[tuple[int, bool]], Cartridge]:
    parts = text.split(":")
    try:
        left, right, cart = parts
        lp = [_mounted_port(t) for t in left.split(",") if t]
        rp = [_mounted_port(t) for t in right.split(",") if t]
    except ValueError as e:
        raise argparse.ArgumentTypeError("expected L[r],..:R[r],..:CART") from e
    if not lp or len(lp) != len(rp):
        raise argparse.ArgumentTypeError("--drive needs the same number of motors per side")
    if cart not in CARTRIDGES:
        raise argparse.ArgumentTypeError(f"CART must be one of {', '.join(CARTRIDGES)}")
    return lp, rp, CARTRIDGES[cart]


def _rotation_arg(text: str) -> tuple[int, TrackerMount | int]:
    """P[r]:track[:D[:X:Y]], P[r]:strafe[:D[:X:Y]] or P:motorN."""
    parts = text.split(":")
    usage = "expected P[r]:track|strafe[:D[:X:Y]] or P:motorN"
    try:
        n, flipped = _mounted_port(parts[0])
        kind, rest = parts[1], parts[2:]
        if kind.startswith("motor") and not rest and not flipped:
            return n, int(kind.removeprefix("motor"))
        if kind not in ("track", "strafe") or len(rest) not in (0, 1, 3):
            raise ValueError(usage)
        nums = [float(x) for x in rest]
        if not all(math.isfinite(x) for x in nums) or (nums and nums[0] <= 0):
            raise ValueError("the wheel diameter must be positive")
        d = nums[0] if nums else TRACKING_WHEEL_DIAMETER_M / INCH
        x, y = nums[1:] if len(nums) == 3 else (0.0, 0.0)
    except (ValueError, IndexError) as e:
        raise argparse.ArgumentTypeError(f"{usage} ({e})") from e
    return n, TrackerMount(kind == "strafe", d, x + 0.0, y + 0.0, flipped)


def _port(text: str) -> int:
    """A smart port number, 1..22 (22 = the brain's 3-wire ports)."""
    if not text.isdigit() or not 1 <= int(text) <= 22:
        raise argparse.ArgumentTypeError(f"bad port {text!r}: use 1..22")
    return int(text)


def _field_controller_arg(text: str) -> tuple[int, str]:
    """P[:TEAM] for --field-controller; TEAM is up to 10 ASCII characters."""
    p, _, team = text.partition(":")
    n = _port(p)
    team = team or f"SIM{n}"
    if len(team) > 10 or not team.isascii():
        raise argparse.ArgumentTypeError(f"bad team {team!r}: up to 10 ASCII characters")
    return n, team


def _ports(text: str) -> list[int]:
    return [int(p) for p in text.split(",") if p]


def drive_spec(args: argparse.Namespace) -> DrivetrainSpec:
    lp, _, cart = args.drive
    geometry = dict(motors_per_side=len(lp), cartridge=cart,
                    wheel_diameter_m=args.wheel_diameter * INCH,
                    track_width_m=args.track * INCH, holonomic=args.holonomic,
                    traction_mu=args.traction)
    if args.wheel_rpm:                    # free motor output rpm -> wheel rpm
        geometry["external_ratio"] = motor.INTERNAL_FREE_RPM / cart.ratio / args.wheel_rpm
    return DrivetrainSpec(**geometry)


def build(args: argparse.Namespace) -> Robot:
    traced = set(_ports(args.trace or ""))

    used: dict[int, str] = {}

    def port(n: int, firmware: Any, sign: int = 1) -> Port:
        name = type(firmware).__name__.removeprefix("Smart").lower()
        if n in used:
            hint = ""
            replaced = getattr(args, "replaced_controller", [])
            if name == "radio" and replaced:
                hint = f" (the preset's controller was on {', '.join(replaced)}: use that port)"
            raise SystemExit(f"smart port {n} has two devices, {used[n]} and {name}{hint}")
        used[n] = name
        path = args.sock.format(port=n)
        return Port(n, connect(path), protocol.FrameReader(), firmware, sign,
                    trace=n in traced, path=path)

    def motor_port(n: int, cart: Cartridge, sign: int = 1) -> Port:
        if "all" in args.motor_fw or str(n) in args.motor_fw:
            return port(n, FirmwareMotor(cart, name=f"p{n}"), sign)
        return port(n, SmartMotor(cart, serial=0x5EED0000 + n), sign)

    def sign(base: int, flipped: bool) -> int:
        return -base if flipped else base

    free = [motor_port(n, cart, sign(1, f)) for n, cart, f in args.motor]
    drive = None
    if args.drive:
        lp, rp, c = args.drive
        spec = drive_spec(args)
        drive = (spec, [motor_port(n, c, sign(1, f)) for n, f in lp],
                 [motor_port(n, c, sign(-1, f)) for n, f in rp])
    robot = Robot(free, drive)

    for n in (int(i) for i in args.imu):
        robot.sensors.append(Sensor(port(n, SmartImu(serial=0x1A0D0000 + n)),
                                    robot.imu_sample))
    for n, src in args.rotation:
        sample = (robot.tracker_sample(src) if isinstance(src, TrackerMount)
                  else robot.motor_shaft_sample(src))
        robot.sensors.append(Sensor(port(int(n), SmartRotation()), sample))
    for n in (int(i) for i in args.optical):
        robot.sensors.append(Sensor(port(n, SmartOptical()), robot.optical_sample))
    for n in (int(i) for i in args.aivision):
        robot.sensors.append(Sensor(port(n, SmartAiVision()), robot.aivision_sample))
    for n in (int(i) for i in args.vision):
        robot.sensors.append(Sensor(port(n, SmartVision()), robot.vision_sample))
    for n in (int(i) for i in args.gps):
        robot.sensors.append(Sensor(port(n, SmartGps()), robot.gps_sample))
    for m in args.distance:
        robot.sensors.append(Sensor(port(m.port, SmartDistance()),
                                    lambda m=m: robot.distance_sample_for(m)))
    # The brain's built-in 3-wire ports (A..H) are an ADI device on port 22;
    # keep it present by default so the on-brain 3-wire screen has a device,
    # unless the user wired port 22 themselves.
    # --no-builtin-adi skips it: for a second plant that only plugs extra
    # devices into free ports of a brain whose own plant already holds 22.
    if args.no_builtin_adi or any(n == 22 for n, _ in args.adi):
        adi = args.adi
    else:
        adi = [(22, AdiWiring.empty())] + args.adi
    for n, wiring in adi:
        robot.sensors.append(Sensor(port(n, SmartAdi()),
                                    lambda w=wiring: adi_sample(w, robot.adi_world())))
    for kind, cls in (("magnet", SmartMagnet), ("tower", SmartLightTower),
                      ("pneumatic", SmartPneumatic), ("led", SmartLed)):
        sample = robot.tower_sample if kind == "tower" else (lambda: None)
        for n in getattr(args, kind):
            robot.sensors.append(Sensor(port(n, cls()), sample))
    for spec in args.radio:
        n, _, extra = spec.partition(":")        # validated in main()
        if extra:
            robot.partner = ControllerState()
        robot.sensors.append(Sensor(port(int(n), SmartRadio()), robot.radio_sample))
    for n in (int(i) for i in args.controller):
        robot.sensors.append(Sensor(port(n, SmartController()),
                                    robot.controller_sample))
    for n, team in args.field_controller:
        fw = SmartController(serial=0x12345678 + n, field_team=team)
        robot.sensors.append(Sensor(port(n, fw), robot.controller_sample))
    return robot


class _Parser(argparse.ArgumentParser):
    """@FILE reads more arguments from FILE: several per line, # comments."""

    def convert_arg_line_to_args(self, arg_line: str) -> list[str]:
        return arg_line.lstrip("\ufeff").split("#", 1)[0].split()


def parser() -> argparse.ArgumentParser:
    ap = _Parser(description=__doc__, fromfile_prefix_chars="@",
                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motor", action="append", default=[], type=_motor_arg,
                    metavar="P:CART")
    ap.add_argument("--drive", type=_drive_arg, metavar="L:R:CART")
    ap.add_argument("--motor-fw", action="append", default=[], metavar="P|all",
                    help="the motor on port P (or every motor) runs VEX's own "
                    "motor firmware in QEMU (machine vex-motor)")
    ap.add_argument("--imu", action="append", default=[], metavar="P")
    ap.add_argument("--rotation", action="append", default=[], type=_rotation_arg,
                    metavar="P[r]:SRC")
    ap.add_argument("--distance", action="append", default=[], type=_distance_arg,
                    metavar="P[:FACE[:X:Y]]")
    ap.add_argument("--optical", action="append", default=[], metavar="P")
    ap.add_argument("--gps", action="append", default=[], metavar="P")
    ap.add_argument("--aivision", action="append", default=[], metavar="P")
    ap.add_argument("--vision", action="append", default=[], type=_port, metavar="P")
    for kind in ("magnet", "tower", "pneumatic", "led"):
        ap.add_argument(f"--{kind}", action="append", default=[], type=_port,
                        metavar="P")
    ap.add_argument("--controller", action="append", default=[], metavar="P")
    ap.add_argument("--radio", action="append", default=[], metavar="P[:partner]")
    ap.add_argument("--adi", action="append", default=[], type=_adi_arg,
                    metavar="P:A=KIND,...")
    ap.add_argument("--no-builtin-adi", action="store_true")
    ap.add_argument("--field-controller", action="append", default=[],
                    type=_field_controller_arg, metavar="P[:TEAM]")
    ap.add_argument("--sock", default="/tmp/v5port{port}.sock")
    ap.add_argument("--trace", metavar="P[,P...]")
    ap.add_argument("--view", type=int, metavar="PORT")
    ap.add_argument("--qmp", default="", metavar="SOCK",
                    help="the brain's QMP socket: lets the field view press its button")
    ap.add_argument("--blocks", type=_blocks_arg, metavar="random:N[:SEED]|FILE.json")
    ap.add_argument("--intake", type=_intake_arg, metavar="P[:CAPACITY]")
    ap.add_argument("--arm", action="append", default=[], type=_arm_arg,
                    metavar="P:RATIO:LEN:KG[:START:LO:HI]")
    ap.add_argument("--goals", type=_goals_arg, metavar="FILE.json")
    ap.add_argument("--tags", type=_tags_arg, metavar="FILE.json")
    ap.add_argument("--match", type=MatchSchedule.parse,
                    metavar="START[:AUTON:DRIVER]")
    ap.add_argument("--competition", choices=("field", "switch"))
    ap.add_argument("--wheel-diameter", type=float, default=3.25, metavar="IN",
                    help="drive wheel diameter (effective), inches (3.25)")
    ap.add_argument("--holonomic", choices=("x", "mecanum"),
                    help="the --drive is a four-motor holonomic drive")
    ap.add_argument("--wheel-rpm", type=float, metavar="RPM",
                    help="drive wheel free speed: sets the gearing from the "
                         "cartridge (default 3/4 of it, e.g. blue 600 -> 450)")
    ap.add_argument("--track", type=float, default=12.0, metavar="IN",
                    help="distance between the left and right wheels, inches (12)")
    ap.add_argument("--traction", type=float, default=DrivetrainSpec.traction_mu,
                    metavar="MU", help="wheel-tile friction (0.8): where wheels "
                    "slip at walls, and with --physics mujoco at launch and braking")
    ap.add_argument("--start", type=_start_arg, metavar="X:Y:HEADING",
                    help="start pose in RW-Template field coordinates (in, deg)")
    ap.add_argument("--battery", type=float, default=100.0, metavar="PCT",
                    help="robot battery charge at the start, percent (100)")
    ap.add_argument("--no-battery", dest="battery", action="store_const", const=None,
                    help="an ideal supply instead of the battery")
    ap.add_argument("--physics", choices=("builtin", "mujoco"), default="builtin",
                    help="robot and field physics (builtin)")
    ap.add_argument("--obstacles", type=_obstacles_arg, metavar="FILE.json",
                    help="field elements to collide with (--physics mujoco)")
    ap.add_argument("--opponent", action="append", default=[], type=_opponent_arg,
                    metavar="X:Y:HEADING[:braked]",
                    help="another robot on the field (--physics mujoco)")
    ap.add_argument("--no-noise", action="store_true",
                    help="exact Distance Sensor readings (no jitter)")
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--status", type=float, default=1.0,
                    help="print state every N seconds (0 = off)")
    return ap


def main(argv: list[str] | None = None) -> None:
    ap = parser()
    args = ap.parse_args(argv)
    if not any((args.motor, args.drive, args.imu, args.rotation, args.distance,
                args.controller, args.adi, args.optical, args.gps,
                args.aivision, args.vision, args.magnet, args.tower, args.pneumatic,
                args.led, args.radio, args.field_controller)):
        ap.error("give at least one device")
    if args.radio and args.controller:
        # a preset's tethered controller: the radio takes over the link
        print(f"--radio replaces --controller {', '.join(args.controller)}",
              file=sys.stderr)
        args.replaced_controller, args.controller = args.controller, []
    if (args.controller or args.radio) and not (args.view or args.match):
        ap.error("--controller / --radio take input through --view's HTTP server")
    if args.match and not (args.controller or args.radio):
        ap.error("--match reports the competition state through --controller or --radio")
    if args.competition and not (args.controller or args.radio):
        ap.error("--competition reports the competition state through --controller or --radio")
    if args.competition and args.match:
        ap.error("--match already runs competition control: use one or the other")
    if len(args.radio) > 1:
        # a preset's radio: the last --radio (the user's) takes over the link
        print(f"--radio {args.radio[-1]} replaces --radio {', '.join(args.radio[:-1])}",
              file=sys.stderr)
        replaced = [r.partition(":")[0] for r in args.radio[:-1]]
        args.replaced_controller = getattr(args, "replaced_controller", []) + replaced
        args.radio = args.radio[-1:]
    if any(not r.partition(":")[0].isdigit() or r.partition(":")[2] not in ("", "partner")
           for r in args.radio):
        ap.error("--radio takes P or P:partner")
    if (args.blocks is not None or args.intake) and not args.drive:
        ap.error("--blocks / --intake need --drive")
    if not (0.5 <= args.wheel_diameter <= 10 and 4 <= args.track <= 30):
        ap.error("--wheel-diameter must be 0.5..10 in and --track 4..30 in")
    if not 0.1 <= args.traction <= 2.0:
        ap.error("--traction must be 0.1..2.0")
    if args.holonomic and not (args.drive and len(args.drive[0]) == len(args.drive[1]) == 2):
        ap.error("--holonomic needs a --drive of two motors a side (front, back)")
    if args.wheel_rpm is not None and not (args.drive and 10 <= args.wheel_rpm <= 2000):
        ap.error("--wheel-rpm needs --drive and 10..2000 rpm")
    if args.physics == "mujoco":
        if not args.drive:
            ap.error("--physics mujoco simulates a --drive robot")
        try:
            import mujoco  # noqa: F401
        except ImportError:
            ap.error("--physics mujoco needs the mujoco package: run the sim "
                     "with ~/venvs/sim/bin/python (pip install mujoco)")
    if (args.obstacles or args.opponent) and args.physics != "mujoco":
        ap.error("--obstacles / --opponent collide in --physics mujoco")
    if args.start and not args.drive:
        ap.error("--start places the --drive robot")
    if args.tags and not (args.aivision and args.drive):
        ap.error("--tags are seen by --aivision on a --drive robot")
    if args.goals and args.blocks is None:
        ap.error("--goals scores --blocks: give both")
    if args.intake and args.intake[0] not in [n for n, _, _ in args.motor]:
        ap.error("--intake P must be one of the --motor ports")
    for n, _ in args.arm:
        if n not in [m for m, _, _ in args.motor]:
            ap.error(f"--arm {n}: put a --motor on port {n} first")
    robot = build(args)              # connects to every port: validate first
    robot.match = args.match
    if args.competition:
        robot.start_competition(field=args.competition == "field")
    if args.blocks is not None or args.intake:
        robot.field = args.blocks or Field()
    robot.intake = args.intake
    robot.arms = dict(args.arm)
    if args.battery is not None:
        if not 0 <= args.battery <= 100:
            ap.error("--battery is a percentage, 0..100")
        robot.enable_battery(args.battery / 100)
    robot.goals = args.goals or ()
    robot.obstacles = args.obstacles or ()
    robot.opponents = tuple(args.opponent)
    robot.tags = args.tags or ()
    robot.sensor_noise = not args.no_noise
    if args.start:
        robot.dt_state = drivetrain.initial_state(robot.drive[0], args.start)
    if args.physics == "mujoco":
        robot.use_mujoco()
    robot.qmp_path = args.qmp
    takes_input = True                # dashboard: brain button; plus any controller
    view = (FieldView(args.view, robot.set_input if takes_input else None)
            if args.view else None)
    if view:
        print(f"field view: http://localhost:{args.view}/", flush=True)
        if args.competition:
            url = f"http://localhost:{args.view}/field"
            print(f"field control: {url}", flush=True)
            open_in_browser(url)
    try:
        run(robot, args.seconds, args.status, view)
    except (KeyboardInterrupt, ConnectionError) as e:
        print(f"stopped: {e or 'interrupted'}", file=sys.stderr)
    finally:
        if view:
            view.close()
        for p in robot.ports:
            if p.sock:                # a port can be down (reconnecting)
                p.sock.close()
            if isinstance(p.firmware, FirmwareMotor):
                p.firmware.close()


if __name__ == "__main__":
    main()
