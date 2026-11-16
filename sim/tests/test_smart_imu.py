import math
import struct

import pytest

from v5sim import protocol
from v5sim.smart_imu import (CALIBRATION_S, ImuSample, LinkMode, SmartImu)


def hexframe(s: str) -> bytes:
    return bytes.fromhex(s)


def brain(payload: str, length: int = 16) -> bytes:
    return protocol.build(bytes.fromhex(payload), length)


def sdk_euler(frame: bytes) -> tuple[float, float, float]:
    """The SDK's decode (imuUpdateEulerFromQuaternion) for orientation 8:
    returns (heading 0..360, pitch, roll) in degrees."""
    assert (frame[1] & 0x1F) >> 1 & 7 == 0
    a, b, c, d = struct.unpack_from("<4f", frame, 4)
    w, x, y, z = b, -a, -d, c                      # remap (x, -w, -z, y)
    yaw = math.degrees(math.atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - x * z)))))
    roll = math.degrees(math.atan2(2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    return yaw % 360, pitch, roll


def linked(now: float = 0.0) -> SmartImu:
    imu = SmartImu()
    imu.on_brain_frame(hexframe("03 01 64 00 00 00 00 0A 00 00 00 00 00 00 2C D0"), now)
    imu.on_brain_frame(hexframe("02 13 01 00 00 00 00 00 00 00 00 00 00 00 85 CC"), now)
    imu.on_brain_frame(brain("04 00 0A 00", 64), now)
    return imu


def test_info_frame_matches_real_imu():
    assert SmartImu().info_frame() == hexframe(
        "0A 0F 00 00 06 00 00 40 78 56 34 12 00 00 80 B0")


def test_handshake_switches_to_64_byte_frames_on_02_13():
    imu = SmartImu()
    r = imu.on_brain_frame(hexframe("03 01 64 00 00 00 00 0A 00 00 00 00 00 00 2C D0"), 0.0)
    assert r == [hexframe("13 40 00 00 00 00 00 00 00 00 00 00 00 00 B8 EE")]
    assert imu.frame_len == 16                      # not before the brain's 02 13
    imu.on_brain_frame(hexframe("02 13 01 00 00 00 00 00 00 00 00 00 00 00 85 CC"), 0.0)
    assert imu.frame_len == 64
    keepalive = imu.due_frames(0.2, ImuSample())[0]
    assert len(keepalive) == 64 and keepalive[0] == 0x11 and keepalive[-2:] == b"\x7f\x20"
    r = imu.on_brain_frame(brain("04 00 0A 00", 64), 0.2)
    assert len(r[0]) == 64 and r[0][:2] == b"\x02\x04" and r[0][-2:] == b"\x48\x82"
    assert imu.link == LinkMode.DATA and imu.interval_s == pytest.approx(0.010)


def test_calibrating_frame_matches_example():
    imu = linked()
    f = imu.status_frame(0.5, ImuSample())
    assert f == hexframe(
        "9D 11 00 19 00 00 00 00 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00"
        " 00 00 00 00 00 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
        " 08 08 08 01 00 00 00 00 00 00 00 00 00 00 C5 78")


def test_heading_90_frame_matches_example():
    imu = linked()
    imu.status_frame(CALIBRATION_S + 0.1, ImuSample(heading_deg=0.0))  # zero here
    f = imu.status_frame(CALIBRATION_S + 0.2,
                         ImuSample(heading_deg=90.0, gyro_dps=(0, 0, 10.0)))
    assert f[1] == 0x10
    assert f == hexframe(
        "9D 10 00 19 00 00 00 00 F3 04 35 3F F3 04 35 3F 00 00 00 00 00 00 00 00"
        " 00 00 00 00 00 20 00 00 00 00 48 01 00 00 00 00 00 00 00 00 00 00 00 00"
        " 08 08 08 01 00 00 00 00 00 00 00 00 00 00 12 83")


def test_calibration_lasts_401_samples_then_heading_counts_from_there():
    imu = linked()
    assert imu.calibrating(CALIBRATION_S - 0.01)
    assert not imu.calibrating(CALIBRATION_S + 0.01)
    imu.status_frame(CALIBRATION_S + 0.01, ImuSample(heading_deg=30.0))
    f = imu.status_frame(CALIBRATION_S + 0.5, ImuSample(heading_deg=75.0))
    assert sdk_euler(f)[0] == pytest.approx(45.0, abs=1e-3)


@pytest.mark.parametrize("heading, pitch, roll", [(10, 0, 0), (350, 0, 0),
                                                  (123, 5, -3), (200, -10, 8)])
def test_sdk_decodes_what_we_encode(heading, pitch, roll):
    imu = linked()
    imu.status_frame(CALIBRATION_S + 0.01, ImuSample())
    f = imu.status_frame(CALIBRATION_S + 0.1,
                         ImuSample(heading_deg=heading, pitch_deg=pitch, roll_deg=roll))
    h, p, r = sdk_euler(f)
    assert (h, p, r) == pytest.approx((heading, pitch, roll), abs=1e-3)


def test_reset_recalibrates_and_rezeroes_heading():
    imu = linked()
    imu.status_frame(CALIBRATION_S + 0.01, ImuSample(heading_deg=0.0))
    t = CALIBRATION_S + 1.0
    imu.on_brain_frame(brain("50 08 47 00 08 08 08", 64), t)
    assert imu.status_frame(t + 0.1, ImuSample(heading_deg=120.0))[1] & 0x01
    imu.status_frame(t + CALIBRATION_S + 0.01, ImuSample(heading_deg=120.0))
    f = imu.status_frame(t + CALIBRATION_S + 0.1, ImuSample(heading_deg=130.0))
    assert sdk_euler(f)[0] == pytest.approx(10.0, abs=1e-3)


def test_data_rate_change_in_data_mode():
    imu = linked()
    assert imu.on_brain_frame(brain("04 00 14 00", 64), 1.0) == []
    assert imu.interval_s == pytest.approx(0.020)


def test_accel_range_scales_raw_accel():
    imu = linked()
    cfg = bytearray(62)
    cfg[0], cfg[2], cfg[0x18] = 0x50, 0x80, 3
    imu.on_brain_frame(protocol.build(bytes(cfg), 64), 0.0)
    f = imu.status_frame(0.1, ImuSample(accel_g=(0, 0, 1.0)))
    assert f[51] == 3 and struct.unpack_from("<h", f, 28)[0] == 2048


def test_short_config_frame_is_ignored_not_fatal():
    imu = linked()
    imu.on_brain_frame(brain("50 00 80 00"), 0.0)   # 16 B, no range byte
    assert imu.accel_range == SmartImu().accel_range
