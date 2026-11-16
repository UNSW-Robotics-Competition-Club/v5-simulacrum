"""Device restarts: the brain-set watchdog and what a reboot forgets."""
import pytest

from v5sim import protocol
from v5sim.smart_device import LinkMode
from v5sim.smart_optical import SmartOptical
from v5sim.smart_sensors import RotationSample, SmartRotation


def brain(payload: str, length: int = 16) -> bytes:
    return protocol.build(bytes.fromhex(payload), length)


def linked_rotation(watchdog_byte: int = 0, rate_ms: int = 20) -> SmartRotation:
    r = SmartRotation()
    r.on_brain_frame(brain("03 00 64 00 00 00 00 0A"), 0.0)
    ack = bytearray(14)
    ack[0], ack[1], ack[2], ack[13] = 0x02, 0x13, 0x01, watchdog_byte
    r.on_brain_frame(protocol.build(bytes(ack)), 0.0)
    r.on_brain_frame(brain(f"04 00 {rate_ms:02x} 00"), 0.0)
    return r


def test_watchdog_follows_the_brains_02_13_byte_13():
    r = linked_rotation(watchdog_byte=8)                 # 8 s
    r.due_frames(5.0, RotationSample())
    assert r.link == LinkMode.DATA
    r.due_frames(8.5, RotationSample())
    assert r.link == LinkMode.INFO


def test_slow_data_rate_never_looks_like_silence():
    r = linked_rotation(rate_ms=0xFA)                     # 250 ms frames
    r.interval_s = 3.0                                    # slower than 2 s
    r.due_frames(2.5, RotationSample())                   # brain only answers frames
    assert r.link == LinkMode.DATA


def test_reboot_restores_power_up_state():
    o = SmartOptical()
    o.on_brain_frame(brain("03 00 64 00 00 00 00 0A"), 0.0)
    o.on_brain_frame(brain("02 13 01 00"), 0.0)
    o.on_brain_frame(protocol.build(bytes.fromhex("50 00 21 00 64 00 EE"), 32), 0.0)
    assert (o.led, o.atime) == (100, 0xEE)
    o.reboot(1.0)
    assert (o.led, o.atime, o.frame_len, o.link) == (0, 0xDB, 16, LinkMode.INFO)
    assert o.due_frames(1.0, None) == [o.info_frame()]
