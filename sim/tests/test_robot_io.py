import socket

import pytest

from v5sim import protocol
from v5sim.robot import Port, Robot, Sensor
from v5sim.smart_motor import SmartMotor


class Exploding(SmartMotor):
    def on_brain_frame(self, frame, now):
        raise IndexError("model bug")


def test_a_device_model_bug_does_not_stop_the_sim(capsys):
    brain, dev = socket.socketpair()
    with brain, dev:
        robot = Robot([Port(1, dev, protocol.FrameReader(), Exploding())], None)
        brain.sendall(protocol.build(b"\x03"))
        robot.service_io(0.0, 0.5)               # must not raise
        assert "p1" in capsys.readouterr().err


def test_match_schedule_drives_the_controller_competition_byte(capsys):
    from v5sim.match import MatchSchedule
    from v5sim.smart_controller import Competition
    robot = Robot([], None)
    robot.match = MatchSchedule(1.0, 2.0, 3.0)
    robot.step(0.5)
    assert robot.controller_sample().competition == Competition.DISABLED
    robot.step(1.0)
    assert robot.controller_sample().competition == Competition.AUTONOMOUS
    assert "AUTONOMOUS" in capsys.readouterr().out   # phase changes are logged
    robot.set_controller_input({"axes": {"Axis3": 50}})
    robot.step(5.0)
    s = robot.controller_sample()
    assert s.competition == Competition.DRIVER and s.axes == {"Axis3": 50}


def test_field_objects_step_with_the_robot():
    from v5sim import drivetrain
    from v5sim.field_objects import Block, Field
    robot = Robot([], None)
    robot.dt_state = drivetrain.initial_state(drivetrain.DrivetrainSpec())
    robot.field = Field((Block(0.2, 0.0, "red"),))  # inside the front face
    for i in range(20):                              # 20 ms
        robot.step(i * 0.001)
    b = robot.field.blocks[0]
    assert b.x > 0.27 and "blocks 1" in robot.describe()


def test_field_view_cannot_override_a_running_match():
    from v5sim.match import MatchSchedule
    from v5sim.smart_controller import Competition
    robot = Robot([], None)
    robot.match = MatchSchedule(0.0, 5.0, 5.0)
    robot.step(1.0)
    robot.set_controller_input({"axes": {"Axis1": 9}, "competition": "NONE"})
    s = robot.controller_sample()
    assert s.competition == Competition.AUTONOMOUS and s.axes == {"Axis1": 9}


def _robot_with_field(blocks=(), held=()):
    from v5sim import drivetrain
    from v5sim.field_objects import Field
    robot = Robot([], None)
    robot.dt_state = drivetrain.initial_state(drivetrain.DrivetrainSpec())
    robot.field = Field(tuple(blocks), tuple(held))
    return robot


def test_optical_sees_held_then_front_block_then_floor():
    from v5sim.field_objects import Block
    from v5sim.smart_optical import raw_for_rgb
    held = _robot_with_field(held=("red", "blue")).optical_sample()
    assert held.proximity == 255 and held.blue > held.red           # newest = blue
    front = _robot_with_field(blocks=[Block(0.2286 + 0.045 + 0.02, 0.0, "red")]).optical_sample()
    assert 0 < front.proximity < 255 and front.red > front.blue
    floor = _robot_with_field().optical_sample()
    assert floor.proximity == 0 and floor.clear < held.clear


def test_gps_reports_pose_clockwise_from_plus_y():
    import math
    from v5sim import drivetrain
    robot = _robot_with_field()
    robot.dt_state = drivetrain.initial_state(
        drivetrain.DrivetrainSpec(), drivetrain.Pose(0.5, -0.25, 0.0))   # facing +x
    s = robot.gps_sample()
    assert (s.x_m, s.y_m) == (0.5, -0.25) and s.quality == 0
    assert math.degrees(s.yaw_rad) == pytest.approx(90.0)


def test_optical_counts_follow_the_sensors_integration_time():
    from v5sim.smart_optical import SmartOptical, full_scale
    robot = _robot_with_field(held=("red",))
    fw = SmartOptical()
    fw.atime = 0xEE                                   # integrationTime(50 ms)
    robot.sensors.append(Sensor(Port(8, None, None, fw), robot.optical_sample))
    s = robot.optical_sample()
    assert s.clear / full_scale(0xEE) == pytest.approx(0.30, abs=0.01)


def test_a_closed_port_reboots_its_device_and_reconnects(tmp_path):
    from v5sim.smart_device import LinkMode
    from v5sim.smart_sensors import SmartRotation
    path = str(tmp_path / "p1.sock")
    server = socket.socket(socket.AF_UNIX)
    server.bind(path)
    server.listen(2)
    client = socket.socket(socket.AF_UNIX)
    client.connect(path)
    brain, _ = server.accept()
    fw = SmartRotation()
    fw.link = LinkMode.DATA
    robot = Robot([], None)
    port = Port(1, client, protocol.FrameReader(), fw, path=path)
    robot.sensors.append(Sensor(port, lambda: None))
    brain.close()                                    # QEMU went away
    robot.service_io(1.0, 0.2)
    assert port.sock is None and fw.link == LinkMode.INFO
    robot.service_io(1.1, 0.0)
    assert port.sock is None                         # retries every 0.5 s
    robot.service_io(1.6, 0.0)                       # listener still there:
    assert port.sock is not None                     # back on the next retry
    assert robot.all_down_for(1.6) == 0.0
    brain2, _ = server.accept()
    brain2.close()
    server.close()


@pytest.mark.parametrize("argv", [["--motor", "abc:red"], ["--motor", "7:pink"],
                                  ["--drive", "1,2:3:blue"],
                                  ["--drive", "1:2:blue", "--motor", "7:green", "--intake", "7:0"]])
def test_bad_device_specs_are_usage_errors_before_connecting(argv, capsys):
    from v5sim.robot import main
    with pytest.raises(SystemExit) as e:
        main(argv + ["--sock", "/nonexistent/{port}.sock"])
    assert e.value.code == 2 and "error" in capsys.readouterr().err


def test_aivision_sees_blocks_ahead_left_of_centre_when_left():
    from v5sim.field_objects import Block
    robot = _robot_with_field(blocks=[Block(1.0, 0.2, "red"), Block(-1.0, 0.0, "blue")])
    seen = robot.aivision_sample().in_view
    assert len(seen) == 1 and seen[0].color == "red"            # the one behind: no
    v = seen[0]
    assert v.x + v.w / 2 < 160                                   # +y is left: image left
    assert 0 < v.w < 80 and v.y + v.h > 120                      # below the horizon


def test_aivision_never_reports_zero_size_boxes():
    from v5sim.field_objects import Block
    robot = _robot_with_field(blocks=[Block(0.2286 + 0.29, 0.0, "red")])  # at the image's bottom edge
    assert all(v.w >= 1 and v.h >= 1 for v in robot.aivision_sample().in_view)


def test_a_stalled_port_drops_frames_instead_of_freezing_the_sim():
    import time
    from v5sim.robot import SEND_TIMEOUT_S
    from v5sim.smart_sensors import SmartRotation
    brain, dev = socket.socketpair()
    with brain, dev:
        dev.settimeout(SEND_TIMEOUT_S)
        robot = Robot([], None)
        port = Port(1, dev, protocol.FrameReader(), SmartRotation())
        robot.sensors.append(Sensor(port, lambda: None))
        t0 = time.monotonic()
        for _ in range(2000):                   # far more than the socket buffers
            robot._send(port, bytes(64), 0.0)
        assert time.monotonic() - t0 < 2.0
        assert port.sock is dev                 # still connected: frames dropped


def test_robot_presets_parse(tmp_path):
    import pathlib
    from v5sim.robot import parser
    presets = sorted(pathlib.Path(__file__).parent.parent.joinpath("robots").glob("*.args"))
    assert presets, "ship at least one preset"
    for p in presets:
        args = parser().parse_args([f"@{p}"])
        assert args.drive, p                         # every preset has a drivetrain
    extra = tmp_path / "x.args"
    extra.write_text("# a comment\n--imu 10   --gps 9  # trailing\n\n")
    args = parser().parse_args([f"@{extra}"])
    assert args.imu == ["10"] and args.gps == ["9"]


@pytest.mark.parametrize("spec,expected", [
    ("13", (13, 0.0, 0.0, 9.0)),                      # front face, default
    ("13:right", (13, 90.0, 9.0, 0.0)),
    ("13:back:0:-6", (13, 180.0, 0.0, -6.0)),         # the lesson's back sensor
    ("13:270:-7.5:1", (13, 270.0, -7.5, 1.0)),
])
def test_distance_mount_specs(spec, expected):
    from v5sim.robot import _distance_arg
    m = _distance_arg(spec)
    assert (m.port, m.face_deg, m.x_in, m.y_in) == expected


def test_distance_sensors_read_the_wall_their_beam_hits():
    import math
    from v5sim import drivetrain
    from v5sim.robot import FIELD_HALF_SIDE_M, INCH, _distance_arg
    robot = _robot_with_field()
    robot.dt_state = drivetrain.initial_state(
        drivetrain.DrivetrainSpec(), drivetrain.Pose(0.3, -0.2, math.radians(90)))  # facing +y
    robot.sensor_noise = False                       # exact geometry
    right = robot.distance_sample_for(_distance_arg("13:right"))   # beam to +x
    back = robot.distance_sample_for(_distance_arg("13:back:0:-6"))
    assert right.distance_mm == pytest.approx((FIELD_HALF_SIDE_M - 0.3 - 9 * INCH) * 1000, abs=1)
    assert back.distance_mm == pytest.approx((FIELD_HALF_SIDE_M - 0.2 - 6 * INCH) * 1000, abs=1)


def test_field_matches_the_course_template():
    from v5sim.robot import FIELD_HALF_SIDE_M, INCH
    assert FIELD_HALF_SIDE_M == pytest.approx(70.25 * INCH)


def test_motor_mount_flips_in_drive_and_motor_specs():
    from v5sim.motor import Cartridge
    from v5sim.robot import _drive_arg, _motor_arg
    left, right, cart = _drive_arg("1r,2r,3:4r,5r,6:blue")
    assert left == [(1, True), (2, True), (3, False)]
    assert right == [(4, True), (5, True), (6, False)] and cart == Cartridge.BLUE_6
    assert _motor_arg("18r:green") == (18, Cartridge.GREEN_18, True)
    assert _motor_arg("7:green") == (7, Cartridge.GREEN_18, False)


def test_a_reversed_intake_still_takes_blocks_in():
    from v5sim import drivetrain
    from v5sim.field_objects import Block, Field
    from v5sim.motor import MotorState
    from v5sim.smart_motor import SmartMotor
    intake = Port(18, None, None, SmartMotor(), sign=-1)       # "18r"
    robot = Robot([intake], None)
    robot.free_states = [MotorState(velocity_rad_s=20.0)]      # plant: inward
    robot.dt_state = drivetrain.initial_state(drivetrain.DrivetrainSpec())
    robot.field = Field((Block(0.25, 0.0, "red"),))
    robot.intake = (18, 3)
    robot._step_field(0.01)
    assert robot.field.held == ("red",)


def test_a_partial_send_finishes_before_the_next_frame():
    from v5sim.robot import STALL_HOLDOFF_S
    from v5sim.smart_sensors import SmartRotation
    brain, dev = socket.socketpair()
    with brain, dev:
        dev.settimeout(0.01)
        robot = Robot([], None)
        port = Port(1, dev, protocol.FrameReader(), SmartRotation())
        frames = [bytes([i % 250 + 1]) * 64 for i in range(8000)]   # 512 KB
        t = 0.0
        for f in frames:                        # overfill: some stall midway
            robot._send(port, f, t)
            t += 0.001
        got = b""
        brain.setblocking(False)
        for _ in range(400):                    # drain while the sim retries
            try:
                got += brain.recv(1 << 16)
            except BlockingIOError:
                pass
            t += STALL_HOLDOFF_S
            robot._send(port, b"", t)
        assert len(got) % 64 == 0               # never a torn frame
        assert all(got[i] == got[i + 63] for i in range(0, len(got), 64))


def test_distance_face_must_be_finite():
    from v5sim.robot import _distance_arg
    for bad in ("13:nan", "13:inf"):
        with pytest.raises(Exception):
            _distance_arg(bad)


def test_distance_readings_jitter_like_a_real_sensor():
    import statistics
    from v5sim.robot import FIELD_HALF_SIDE_M, _distance_arg
    robot = _robot_with_field()
    m = _distance_arg("13:front:0:0")
    readings = [robot.distance_sample_for(m).distance_mm for _ in range(200)]
    true_mm = FIELD_HALF_SIDE_M * 1000
    assert len(set(readings)) > 50                       # never a frozen value
    assert statistics.mean(readings) == pytest.approx(true_mm, abs=3)
    assert 2 <= statistics.pstdev(readings) <= 20
    robot.sensor_noise = False
    assert robot.distance_sample_for(m).distance_mm == pytest.approx(true_mm, abs=0.5)


def test_start_pose_uses_rw_template_field_coordinates():
    import math
    from v5sim.robot import INCH, _start_arg, rw_pose
    pose = _start_arg("-36:-60:90")              # RW: 36 left, 60 back, facing +X
    assert pose.x_m == pytest.approx(-60 * INCH)  # sim +x = RW +Y
    assert pose.y_m == pytest.approx(36 * INCH)   # sim +y = RW -X
    assert math.degrees(pose.heading_rad) == pytest.approx(-90)
    x, y, h = rw_pose(pose)
    assert (x, y, h) == pytest.approx((-36, -60, 90))


def test_drivetrain_geometry_options():
    from v5sim.robot import INCH, parser
    args = parser().parse_args(["--drive", "1:2:blue", "--wheel-diameter", "3.17",
                                "--track", "12.3", "--start=-36:-60:0"])
    assert args.wheel_diameter == pytest.approx(3.17) and args.track == pytest.approx(12.3)
    assert args.start.x_m == pytest.approx(-60 * INCH)


def test_misc_devices_show_in_the_status_line():
    from v5sim.robot import parser
    from v5sim.smart_misc import SmartLed, SmartLightTower, SmartMagnet, SmartPneumatic
    args = parser().parse_args(["--magnet", "3", "--tower", "4", "--pneumatic", "5",
                                "--led", "6"])
    assert (args.magnet, args.tower, args.pneumatic, args.led) == ([3], [4], [5], [6])
    robot = Robot([], None)
    tower, led = SmartLightTower(), SmartLed()
    tower.values, led.rgb = (255, 0, 0, 0, 7), (255, 0, 16)
    for n, fw in enumerate((SmartMagnet(), tower, SmartPneumatic(), led), 3):
        robot.sensors.append(Sensor(Port(n, None, None, fw), lambda: None))
    line = robot.describe(0.0)
    assert "magnet p3[INFO] power +0 0mA 25.0C" in line
    assert "red=255" in line and "yellow=7" in line
    assert "pump off cyl rrrr count 0/0/0/0 pressure 6200" in line
    assert "rgb #ff0010" in line


def test_field_view_input_drives_gestures_and_the_tower_button():
    from v5sim.smart_misc import SmartLightTower
    from v5sim.smart_optical import SmartOptical
    robot = Robot([], None)
    optical, tower = SmartOptical(), SmartLightTower()
    optical.mode = 0x4F
    for n, fw in ((8, optical), (4, tower)):
        robot.sensors.append(Sensor(Port(n, None, None, fw), lambda: None))
    before = robot.controller_sample()
    robot.set_input({"gesture": "down", "tower": True})
    from v5sim.smart_optical import OpticalSample
    assert optical.data_payload(0.0, OpticalSample())[1] == 2
    assert tower.data_payload(0.0, robot.tower_sample())[1] == 1
    assert robot.controller_sample() == before        # sticks untouched
    with pytest.raises(ValueError):
        robot.set_input({"tower": "yes"})


def test_aivision_sees_field_tags_ahead():
    from v5sim.apriltags import FieldTag
    robot = _robot_with_field()
    robot.field = None
    robot.tags = (FieldTag(id=11, x=1.2, y=0.0, facing_deg=180.0),
                  FieldTag(id=12, x=-1.2, y=0.0, facing_deg=0.0))     # behind
    (tag,) = robot.aivision_sample().tags
    assert tag.id == 11 and abs((tag.corners[0][0] + tag.corners[1][0]) / 2 - 160) <= 1


def test_legacy_vision_sees_blocks_in_its_own_image():
    from v5sim.field_objects import Block
    from v5sim.smart_vision import uv_for_rgb
    robot = _robot_with_field([Block(0.8, 0.0, "red")])
    (view,) = robot.vision_sample().in_view
    assert abs(view.x + view.w / 2 - 158) <= 1 and view.uv == uv_for_rgb((200, 40, 15))
    (ai,) = robot.aivision_sample().in_view
    assert abs(ai.x + ai.w / 2 - 160) <= 1 and ai.w != view.w      # different optics


def test_radio_reports_a_field_match_and_its_clock():
    from v5sim.match import MatchSchedule
    from v5sim.smart_radio import SmartRadio
    robot = Robot([], None)
    robot.sensors.append(Sensor(Port(1, None, None, SmartRadio()), robot.radio_sample))
    robot.match = MatchSchedule(1.0, 2.0, 3.0)
    seen = []
    for i in range(0, 900, 5):
        robot.step(i / 100)
        r = robot.radio_sample()
        seen.append((r.status, r.match_secs))
    statuses = [st for st, _ in seen]
    assert statuses[0] == 0xA9                          # pre-match: disabled
    order = [st for k, st in enumerate(statuses) if k == 0 or statuses[k - 1] != st]
    assert order == [0xA9, 0x91, 0x99, 0xA1, 0xA9]
    assert max(sec for st, sec in seen if st == 0x91) == 1   # 2 s auton, count-up
    assert "status a9" in robot.describe(9.0)


def test_switch_competition_clock_restarts_with_the_program():
    from v5sim.smart_radio import SmartRadio
    robot = Robot([], None)
    radio = SmartRadio()
    robot.sensors.append(Sensor(Port(1, None, None, radio), robot.radio_sample))
    robot.set_input({"competition": "DRIVER"})
    robot.step(10.0)
    robot.step(30.0)
    assert robot.radio_sample().match_secs == 20     # since driver was switched on
    radio.on_brain_frame(protocol.build(bytes([0x52, 0, 99, 0, 0x41]), 32), 31.0)
    robot.step(35.0)
    assert robot.radio_sample().match_secs == 4      # since the program started


def test_a_program_restart_leaves_the_field_match_clock_alone():
    from v5sim.match import MatchSchedule
    from v5sim.smart_radio import SmartRadio
    robot = Robot([], None)
    radio = SmartRadio()
    robot.sensors.append(Sensor(Port(1, None, None, radio), robot.radio_sample))
    robot.match = MatchSchedule(1.0, 20.0, 3.0)
    for i in range(0, 1200, 5):
        robot.step(i / 100)
    radio.on_brain_frame(protocol.build(bytes([0x52, 0, 99, 0, 0x81]), 32), 11.0)
    robot.step(12.0)
    assert robot.radio_sample().match_secs == 11     # auton began at 1 s


def test_radio_partner_input():
    from v5sim.smart_controller import ControllerState
    from v5sim.smart_radio import SmartRadio
    robot = Robot([], None)
    robot.partner = ControllerState()
    robot.sensors.append(Sensor(Port(1, None, None, SmartRadio()), robot.radio_sample))
    robot.set_input({"partner": {"axes": {"Axis3": 60}, "buttons": ["A"]}})
    r = robot.radio_sample()
    assert r.partner.axes == {"Axis3": 60} and r.partner.buttons == {"A"}
    assert r.controller.axes == {}
    alone = Robot([], None)
    with pytest.raises(ValueError):
        alone.set_input({"partner": {"axes": {}}})


def test_a_bad_input_message_changes_nothing():
    from v5sim.smart_misc import SmartLightTower
    from v5sim.smart_optical import OpticalSample, SmartOptical
    robot = Robot([], None)
    optical = SmartOptical()
    optical.mode = 0x4F
    robot.sensors.append(Sensor(Port(8, None, None, optical), lambda: OpticalSample()))
    robot.sensors.append(Sensor(Port(4, None, None, SmartLightTower()), lambda: None))
    for bad in ({"gesture": "up", "tower": "x"}, {"gesture": "up", "partner": 5},
                {"gesture": "up", "competition": "NOPE"}, {"gesture": "sideways"}):
        with pytest.raises((ValueError, KeyError)):
            robot.set_input(bad)
        assert optical.data_payload(0.0, OpticalSample())[1] == 0


def test_tower_button_stays_held_across_a_device_reboot():
    from v5sim.smart_misc import SmartLightTower
    robot = Robot([], None)
    tower = SmartLightTower()
    robot.sensors.append(Sensor(Port(4, None, None, tower), robot.tower_sample))
    robot.set_input({"tower": True})
    tower.reboot(1.0)
    assert tower.data_payload(1.0, robot.tower_sample())[1] == 1


@pytest.mark.parametrize("argv", [["--tower", "x"], ["--magnet", "0"], ["--led", "23"],
                                  ["--vision", "abc"], ["--pneumatic", "-1"]])
def test_misc_device_ports_are_validated(argv, capsys):
    from v5sim.robot import parser
    with pytest.raises(SystemExit):
        parser().parse_args(argv)


@pytest.mark.parametrize("spec,expected", [
    ("8:track", (8, False, 2.75, 0.0, 0.0, False)),           # the old default
    ("8:track:2:-2.803:0", (8, False, 2.0, -2.803, 0.0, False)),
    ("7r:strafe:2:0:-2.276", (7, True, 2.0, 0.0, -2.276, True)),
    ("7:strafe", (7, True, 2.75, 0.0, 0.0, False)),
])
def test_tracking_wheel_specs(spec, expected):
    from v5sim.robot import _rotation_arg
    n, m = _rotation_arg(spec)
    assert (n, m.sideways, m.diameter_in, m.x_in, m.y_in, m.flipped) == expected


@pytest.mark.parametrize("spec", ["8:track:2:1", "8:track:0", "x:track", "8:wheel",
                                  "8:motorx"])
def test_bad_tracking_wheel_specs(spec):
    import argparse
    from v5sim.robot import _rotation_arg
    with pytest.raises(argparse.ArgumentTypeError):
        _rotation_arg(spec)


def _spin_and_drive(robot, v_m_s, omega_rad_s, seconds):
    from dataclasses import replace
    from v5sim import drivetrain
    from v5sim.robot import DT
    for _ in range(round(seconds / DT)):
        st = robot.dt_state
        robot.dt_state = replace(st, v_m_s=v_m_s, omega_rad_s=omega_rad_s)
        robot._advance_trackers(DT)
    assert drivetrain                                 # imported for the plant


def test_offset_trackers_read_the_arc_their_wheel_rolls():
    import math
    from v5sim import drivetrain
    from v5sim.robot import INCH, Robot, _rotation_arg
    robot = Robot([], None)
    robot.dt_state = drivetrain.initial_state(drivetrain.DrivetrainSpec())
    fwd = robot.tracker_sample(_rotation_arg("8:track:2:-3:0")[1])
    side = robot.tracker_sample(_rotation_arg("7:strafe:2:0:-2")[1])
    flipped = robot.tracker_sample(_rotation_arg("6r:strafe:2:0:-2")[1])
    # one full turn counter-clockwise on the spot
    _spin_and_drive(robot, 0.0, 2 * math.pi, 1.0)
    # left-of-centre wheel rolls backwards one circle of radius 3 in
    assert fwd().shaft_deg == pytest.approx(-(2 * math.pi * 3) / (math.pi * 2) * 360, rel=1e-3)
    # a wheel behind the centre rolls right while turning left
    assert side().shaft_deg == pytest.approx((2 * math.pi * 2) / (math.pi * 2) * 360, rel=1e-3)
    assert flipped().shaft_deg == pytest.approx(-side().shaft_deg)
    # then 1 m straight ahead: only the forward wheel turns
    before = side().shaft_deg
    start = fwd().shaft_deg
    _spin_and_drive(robot, 1.0, 0.0, 1.0)
    assert fwd().shaft_deg - start == pytest.approx(1.0 / (math.pi * 2 * INCH) * 360, rel=1e-3)
    assert side().shaft_deg == pytest.approx(before)


def test_wheel_rpm_sets_the_drive_gearing():
    from v5sim.robot import drive_spec, parser
    spec = drive_spec(parser().parse_args(["--drive", "1:2:green", "--wheel-rpm", "333.333333"]))
    assert spec.external_ratio == pytest.approx(200 / 333.333333)
    default = drive_spec(parser().parse_args(["--drive", "1:2:blue"]))
    assert default.external_ratio == pytest.approx(48 / 36)   # 600 -> 450 rpm


def test_radio_replaces_a_presets_tethered_controller(tmp_path, capsys, monkeypatch):
    import v5sim.robot as rb
    seen = {}
    monkeypatch.setattr(rb, "build", lambda args: seen.setdefault("args", args) and
                        (_ for _ in ()).throw(SystemExit(0)))
    with pytest.raises(SystemExit):
        rb.main(["--drive", "1:2:green", "--controller", "11", "--radio", "11",
                 "--match", "5"])
    assert seen["args"].controller == [] and seen["args"].radio == ["11"]
    assert "replaces --controller 11" in capsys.readouterr().err


def test_last_radio_replaces_a_presets_radio(capsys, monkeypatch):
    import v5sim.robot as rb
    seen = {}
    monkeypatch.setattr(rb, "build", lambda args: seen.setdefault("args", args) and
                        (_ for _ in ()).throw(SystemExit(0)))
    with pytest.raises(SystemExit):
        rb.main(["--drive", "1:2:green", "--radio", "11", "--radio", "11:partner",
                 "--match", "5"])
    assert seen["args"].radio == ["11:partner"]
    assert seen["args"].replaced_controller == ["11"]
    assert "--radio 11:partner replaces --radio 11" in capsys.readouterr().err


def test_two_devices_on_one_port_are_refused(monkeypatch):
    import v5sim.robot as rb
    from v5sim.robot import build, parser
    monkeypatch.setattr(rb, "connect", lambda path: None)
    args = parser().parse_args(["--drive", "11:12:green", "--radio", "11",
                                "--sock", "/nonexistent/{port}"])
    args.replaced_controller = ["21"]
    with pytest.raises(SystemExit, match=r"port 11 has two devices, motor and radio.*on 21"):
        build(args)


def test_imu_feels_the_turn_sideways():
    from dataclasses import replace
    from v5sim import drivetrain
    robot = Robot([], None)
    base = drivetrain.initial_state(drivetrain.DrivetrainSpec())
    robot.dt_state = replace(base, v_m_s=1.0, omega_rad_s=2.0)    # forward, turning left
    ax, ay, az = robot.imu_sample().accel_g
    assert ay == pytest.approx(2.0 / 9.81, rel=0.01) and az == 1.0
    robot.dt_state = replace(base, v_m_s=1.0, omega_rad_s=-2.0)   # turning right
    assert robot.imu_sample().accel_g[1] < 0


@pytest.mark.parametrize("field, waiting, auton", [(True, 0xA9, 0x91), (False, 0x29, 0x11)])
def test_competition_starts_disabled_and_follows_the_buttons(field, waiting, auton):
    from v5sim.smart_controller import Competition
    from v5sim.smart_radio import SmartRadio
    robot = Robot([], None)
    robot.sensors.append(Sensor(Port(1, None, None, SmartRadio()), robot.radio_sample))
    robot.start_competition(field=field)
    robot.step(0.5)
    assert robot.controller_sample().competition == Competition.DISABLED
    assert robot.radio_sample().status == waiting      # plugged in, waiting
    robot.set_input({"competition": "AUTONOMOUS"})
    robot.step(1.0)
    assert robot.radio_sample().status == auton


def test_competition_option_needs_a_controller_and_no_match(capsys):
    from v5sim.robot import main
    with pytest.raises(SystemExit):
        main(["--imu", "1", "--competition", "field"])
    assert "--competition reports" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        main(["--radio", "11", "--view", "0", "--match", "5", "--competition", "field"])
    assert "use one or the other" in capsys.readouterr().err


def test_field_page_starts_and_stops_a_match():
    from v5sim.smart_controller import Competition
    robot = Robot([], None)
    robot.start_competition(field=True)
    robot.step(10.0)
    robot.set_input({"field": "match"})
    snap = robot.field_snapshot(10.0)
    assert snap["kind"] == "match" and snap["match"]["start"] == 10.0
    assert snap["match"]["end"] == 10.0 + 15 + 2 + 105
    robot.step(11.0)
    assert robot.controller_sample().competition == Competition.AUTONOMOUS
    robot.set_input({"field": "stop", "competition": "DRIVER"})   # manual override
    robot.step(12.0)
    assert robot.match is None
    assert robot.controller_sample().competition == Competition.DRIVER
    assert robot.field_snapshot(12.0)["competition"] == "DRIVER"


@pytest.mark.parametrize("kind, auton, driver", [("skills_driver", 0, 60),
                                                 ("skills_auton", 60, 0)])
def test_field_page_skills_runs(kind, auton, driver):
    robot = Robot([], None)
    robot.step(3.0)
    robot.set_input({"field": kind})
    m = robot.field_snapshot(3.0)["match"]
    assert (m["auton"], m["driver"]) == (auton, driver)
    assert robot.field_control


def test_field_input_is_validated():
    robot = Robot([], None)
    with pytest.raises(ValueError):
        robot.set_input({"field": "overtime"})


def test_open_in_browser_respects_no_browser(monkeypatch):
    from v5sim import robot as robot_mod
    calls = []
    monkeypatch.setattr(robot_mod.subprocess, "Popen", lambda *a, **k: calls.append(a))
    monkeypatch.setattr(robot_mod.shutil, "which", lambda name: "/bin/" + name)
    monkeypatch.setenv("V5SIM_NO_BROWSER", "1")
    robot_mod.open_in_browser("http://localhost:8765/field")
    assert calls == []
    monkeypatch.delenv("V5SIM_NO_BROWSER")
    robot_mod.open_in_browser("http://localhost:8765/field")
    assert calls == [(["explorer.exe", "http://localhost:8765/field"],)]
