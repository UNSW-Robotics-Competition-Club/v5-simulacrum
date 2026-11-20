"""scripts/v5gdb_bridge.py: v5gdb's user-port channels split between GDB,
the log and the program's output; the usual host side keeps working."""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import v5gdb_bridge as vb  # noqa: E402


def packet(channel: str, body: bytes) -> bytes:
    return vb.cobs_encode(channel.encode() + body) + b"\0"


@pytest.mark.parametrize("data", [b"", b"a", b"\0", b"\0\0", b"ab\0cd", bytes(range(1, 255)) * 3,
                                  bytes(300), bytes(range(256)) * 2])
def test_cobs_round_trips(data):
    enc = vb.cobs_encode(data)
    assert 0 not in enc
    assert vb.cobs_decode(enc) == data


def test_cobs_rejects_what_isnt():
    assert vb.cobs_decode(b"\x05ab") is None           # block runs past the end
    assert vb.cobs_decode(b"\x03a\0") is None


def test_packets_go_to_their_channels_split_anywhere():
    stream = packet("d", b"$qSupported#37") + packet("l", b"stopped") + packet("u", b"hello\n")
    for cut in range(len(stream)):
        d = vb.Demux()
        got = d.feed(stream[:cut], 0) + d.feed(stream[cut:], 0)
        assert got == [("d", b"$qSupported#37"), ("l", b"stopped"), ("u", b"hello\n")]


def test_bytes_that_arent_packets_are_program_output():
    d = vb.Demux()
    assert d.feed(vb.cobs_encode(b"x123") + b"\0", 0) == [("u", vb.cobs_encode(b"x123"))]   # unknown channel
    assert d.feed(b"plain printf", 1.0) == []                     # no delimiter yet
    assert d.idle(1.1) == []
    assert d.idle(1.0 + vb.IDLE_FLUSH_S) == [("u", b"plain printf")]
    big = b"x" * (vb.MAX_FRAME + 1)
    assert d.feed(big, 2.0) == [("u", big)]


def test_pros_output_inside_user_packets_is_unwrapped():
    inner = vb.cobs_encode(b"sout" + b"x = 1\n") + b"\0" + vb.cobs_encode(b"kdbg" + b"internal") + b"\0"
    d = vb.Demux()
    assert d.feed(packet("u", inner), 0) == [("u", b"x = 1\n")]
    assert vb.pros_text(b"no frames here") == b"no frames here"


def test_bridge_routes_gdb_output_and_the_usual_host_side(sock_dir):
    brain_path, down_path = os.path.join(sock_dir, "user"), os.path.join(sock_dir, "user-out")
    port = free_port()
    qemu = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    qemu.bind(brain_path)
    qemu.listen(1)
    p = subprocess.Popen([sys.executable, "-I", str(SCRIPTS / "v5gdb_bridge.py"), brain_path, down_path, str(port)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        qemu.settimeout(5)
        brain, _ = qemu.accept()
        brain.settimeout(2)
        down = connect_unix(down_path)
        out = connect_tcp(port + 1)

        # no GDB yet: the usual host side reaches the brain
        down.sendall(b"typed")
        assert brain.recv(100) == b"typed"

        gdb = connect_tcp(port)
        time.sleep(0.2)
        gdb.sendall(b"$g#67")
        assert brain.recv(100) == b"$g#67"
        down.sendall(b"ignored while debugging")
        brain.sendall(packet("d", b"+$OK#9a") + packet("u", b"robot says hi\n"))
        assert gdb.recv(100) == b"+$OK#9a"
        assert out.recv(100) == b"robot says hi\n"
        # the usual host side got the bytes unchanged
        raw = packet("d", b"+$OK#9a") + packet("u", b"robot says hi\n")
        got = b""
        while len(got) < len(raw):
            got += down.recv(100)
        assert got == raw
        brain.settimeout(0.3)
        with pytest.raises(socket.timeout):
            brain.recv(100)                              # "ignored while debugging" didn't arrive

        gdb.close()
        time.sleep(0.2)
        brain.settimeout(2)
        down.sendall(b"again")
        assert brain.recv(100) == b"again"
    finally:
        p.terminate()
        p.wait(5)


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port if port < 65000 else free_port()


def connect_unix(path: str) -> socket.socket:
    for _ in range(50):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.connect(path)
            s.settimeout(2)
            return s
        except OSError:
            time.sleep(0.1)
    raise AssertionError(f"no {path}")


def connect_tcp(port: int) -> socket.socket:
    for _ in range(50):
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=2)
            return s
        except OSError:
            time.sleep(0.1)
    raise AssertionError(f"nothing on {port}")


def test_plain_text_that_runs_into_a_packet_keeps_the_packet():
    d = vb.Demux()
    got = d.feed(b"printed without a newline" + packet("d", b"$T05#b9"), 0)
    assert got == [("u", b"printed without a newline"), ("d", b"$T05#b9")]


def test_a_half_packet_is_forgotten_when_the_brain_reconnects():
    d = vb.Demux()
    d.feed(packet("d", b"$OK#9a")[:3], 0)
    d.reset()
    assert d.feed(packet("l", b"hello"), 1) == [("l", b"hello")]
