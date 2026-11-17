"""The device dashboard (dashboard.py, /devices.json) and the brain button
(brainctl.py, /input "brain_button")."""
import json
import socket
import threading
import urllib.error
import urllib.request

import pytest

from v5sim import brainctl, drivetrain
from v5sim.fieldview import FieldView
from v5sim.robot import Port, Robot, Sensor
from v5sim.smart_imu import SmartImu
from v5sim.smart_motor import SmartMotor
from v5sim.motor import Cartridge


def robot_with_devices():
    robot = Robot([Port(7, None, None, SmartMotor(Cartridge.GREEN_18))],
                  (drivetrain.DrivetrainSpec(motors_per_side=1),
                   [Port(1, None, None, SmartMotor())],
                   [Port(2, None, None, SmartMotor(), sign=-1)]))
    robot.intake = (7, 3)
    robot.sensors.append(Sensor(Port(10, None, None, SmartImu()), robot.imu_sample))
    return robot


def test_snapshot_lists_motors_with_roles_and_sensors_with_readings():
    robot = robot_with_devices()
    devices = robot.device_snapshot(0.0)
    assert [d["port"] for d in devices] == [1, 2, 7, 10]
    by_port = {d["port"]: d for d in devices}
    assert by_port[1]["role"] == "drive left" and by_port[2]["role"] == "drive right"
    assert by_port[7]["role"] == "intake" and by_port[7]["cartridge"] == "green_18"
    m = by_port[7]
    for key in ("position_deg", "velocity_rpm", "voltage_v", "current_a", "temperature_c",
                "mode", "brake", "target_deg", "target_rpm", "link"):
        assert key in m
    imu = by_port[10]
    assert imu["kind"] == "imu" and "heading_deg" in imu and imu["text"]
    json.dumps(devices)                                   # it must serialise


def test_snapshot_reports_the_motor_command_in_degrees_and_rpm():
    from v5sim.smart_motor import Command, ControlMode
    robot = robot_with_devices()
    fw = robot.free_ports[0].firmware
    fw.command = Command(mode=ControlMode.POSITION, target=1800, velocity=1500)
    m = [d for d in robot.device_snapshot(0.0) if d["port"] == 7][0]
    assert m["mode"] == "position"
    assert m["target_deg"] == pytest.approx(720.0)        # green: 2.5 counts/deg
    assert m["target_rpm"] == pytest.approx(100.0)


def test_status_line_still_reads_the_same():
    robot = robot_with_devices()
    line = robot.describe(0.0)
    assert "imu p10[INFO]" in line and "p7[INFO]" in line


@pytest.mark.parametrize("bad", [0, -1, 11, "1", True, None])
def test_brain_button_press_length_is_validated(bad):
    robot = Robot([], None)
    robot.qmp_path = "/tmp/x.sock"
    if bad is None:
        robot.qmp_path = ""
        bad = 0.2
    with pytest.raises(ValueError):
        robot.set_input({"brain_button": bad})


def test_brain_button_presses_through_qmp(monkeypatch):
    calls = []
    monkeypatch.setattr(brainctl, "press_async", lambda path, secs: calls.append((path, secs)))
    robot = Robot([], None)
    robot.qmp_path = "/tmp/v5qmp.sock"
    robot.set_input({"brain_button": 1.5})
    assert calls == [("/tmp/v5qmp.sock", 1.5)]


def test_press_holds_bit_7_low_then_releases(sock_dir):
    path = f"{sock_dir}/qmp.sock"
    srv = socket.socket(socket.AF_UNIX)
    srv.bind(path)
    srv.listen(1)
    seen = []

    def serve():
        conn, _ = srv.accept()
        conn.sendall(b'{"QMP": {}}\n')
        f = conn.makefile("rb")
        for line in f:
            msg = json.loads(line)
            seen.append(msg)
            conn.sendall(b'{"return": {}}\n')
        conn.close()

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    brainctl.press(path, 0.05)
    t.join(2)
    sets = [m["arguments"] for m in seen if m["execute"] == "qom-set"]
    assert sets == [{"path": "/machine/gpio1", "property": "in1", "value": 0x00},
                    {"path": "/machine/gpio1", "property": "in1", "value": 0x80}]
    srv.close()


def get(url):
    with urllib.request.urlopen(url, timeout=2) as r:
        return r.headers.get_content_type(), r.read()


def test_field_view_serves_the_dashboard_and_device_json():
    view = FieldView(0, lambda msg: None)
    port = view._server.server_address[1]
    try:
        ctype, page = get(f"http://127.0.0.1:{port}/dashboard")
        assert ctype == "text/html" and b"<title>V5 Devices</title>" in page
        view.publish_devices(3.5, [{"port": 1, "kind": "motor"}])
        ctype, body = get(f"http://127.0.0.1:{port}/devices.json")
        assert json.loads(body) == {"t": 3.5, "devices": [{"port": 1, "kind": "motor"}]}
        _, field = get(f"http://127.0.0.1:{port}/")
        assert b'href="dashboard"' in field
    finally:
        view.close()
