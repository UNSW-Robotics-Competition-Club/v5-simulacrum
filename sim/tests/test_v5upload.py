"""scripts/v5upload.py (Run on Sim without a restart) and scripts/v5usb_share.py,
against a fake brain that answers CDC2 file transfers as VEXos does."""
from __future__ import annotations

import gzip
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import v5upload as up  # noqa: E402
from mkspiffs_vex import crc32_v5  # noqa: E402

XVX5 = b"XVX5" + bytes(24) + struct.pack("<I", 7) + bytes(64)


def reply(cmd: int, payload: bytes) -> bytes:
    body = up.REPLY + bytes([up.EXT]) + up.varint(len(payload) + 3) + bytes([cmd]) + payload
    return body + up.crc16(body).to_bytes(2, "big")


class FakeBrain:
    """Parses each packet the uploader writes (checking its CRC, as the
    brain does) and answers it; keeps the files it was sent."""

    MAX_PACKET = 0x1000

    def __init__(self, have: dict | None = None, drop_first: set[int] | None = None):
        self.buf = bytearray()
        self.out = bytearray()
        self.files: dict[tuple[int, str], dict] = dict(have or {})
        self.commands: list[int] = []
        self.drop_first = set(drop_first or ())        # commands whose first packet is "lost"
        self.current: dict | None = None

    # the Port interface Brain uses
    def write(self, data: bytes) -> None:
        self.buf += data
        while self._packet():
            pass

    def read_some(self, timeout: float) -> bytes:
        data, self.out = bytes(self.out), bytearray()
        return data

    def drain(self) -> None:
        pass

    def _packet(self) -> bool:
        if len(self.buf) < 7:
            return False
        assert bytes(self.buf[:4]) == up.HEADER and self.buf[4] == up.EXT
        cmd, n, hdr = self.buf[5], self.buf[6], 7
        if n & 0x80:
            n, hdr = ((n & 0x7F) << 8) | self.buf[7], 8
        if len(self.buf) < hdr + n + 2:
            return False
        frame = bytes(self.buf[:hdr + n + 2])
        del self.buf[:hdr + n + 2]
        assert up.crc16(frame) == 0, "bad CRC"
        if cmd in self.drop_first:
            self.drop_first.discard(cmd)
            return True
        self.commands.append(cmd)
        self.out += reply(cmd, self.handle(cmd, frame[hdr:hdr + n]))
        return True

    def handle(self, cmd: int, p: bytes) -> bytes:
        ack = bytes([up.ACK])
        if cmd == up.FT_INIT:
            fn, target, vid, opts, size, addr, crc, kind, stamp, version, name = struct.unpack("<4B3I4s2I24s", p)
            self.current = dict(vid=vid, size=size, addr=addr, crc=crc, kind=kind.rstrip(b"\0"),
                                version=version, name=name.rstrip(b"\0").decode(), data=bytearray(), link=None)
            return ack + struct.pack("<H2I", self.MAX_PACKET, size, crc)
        if cmd == up.FT_SET_LINK:
            vid, _opts, name = struct.unpack("<2B24s", p)
            self.current["link"] = (name.rstrip(b"\0").decode(), vid)
            return ack
        if cmd == up.FT_WRITE:
            addr = struct.unpack_from("<I", p)[0]
            assert addr == self.current["addr"] + len(self.current["data"]), "packet address mismatch"
            assert (len(p) - 4) % 4 == 0, "not a multiple of 4"
            assert len(p) - 4 <= self.MAX_PACKET
            self.current["data"] += p[4:]
            return ack
        if cmd == up.FT_COMPLETE:
            f = self.current
            data = bytes(f["data"][:f["size"]])
            if crc32_v5(data) != f["crc"]:
                return bytes([0xD2])
            f["data"], f["after"] = data, p[0]
            self.files[(f["vid"], f["name"])] = f
            return ack
        if cmd == up.FT_META_BY_NAME:
            vid, _opts, name = struct.unpack("<2B24s", p)
            f = self.files.get((vid, name.rstrip(b"\0").decode()))
            if not f:
                return bytes([0xD3])
            return ack + struct.pack("<B3I4sII24s", 0, f["size"], f["addr"], f["crc"], b"bin\0", 0, 0, b"")
        return bytes([0xFF])


def test_packets_are_framed_as_the_brain_parses_them():
    p = up.ext_packet(0x12, b"\x01")
    assert p[:6] == up.HEADER + bytes([0x56, 0x12]) and p[6] == 1
    assert up.crc16(p) == 0
    assert up.varint(0x7F) == b"\x7f" and up.varint(0x80) == b"\x80\x80" and up.varint(0x1004) == b"\x90\x04"


def test_replies_are_found_among_noise_and_nacks_raise():
    buf = bytearray(b"\x00junk" + reply(0x13, b"\x76") + reply(0x12, b"\x76\x01"))
    assert up.parse_reply(buf, 0x12) == b"\x01"
    assert up.parse_reply(bytearray(reply(0x13, b"\x76")[:5]), 0x13) is None      # incomplete
    with pytest.raises(up.UploadError, match="CRC error"):
        up.parse_reply(bytearray(reply(0x11, b"\xce")), 0x11)
    corrupt = bytearray(reply(0x11, b"\x76\x00\x10"))
    corrupt[-1] ^= 1
    assert up.parse_reply(corrupt, 0x11) is None


def test_a_bin_uploads_its_ini_then_itself_compressed_and_runs(tmp_path):
    prog = tmp_path / "auton.bin"
    prog.write_bytes(XVX5 + os.urandom(10_001))                    # not a multiple of 4
    brain = FakeBrain()
    written = up.upload(up.Brain(brain), up.plan(prog, None, 3, "auton", "run"))
    assert written == ["slot_3.ini", "slot_3.bin"]
    b = brain.files[(up.VID_USER, "slot_3.bin")]
    assert gzip.decompress(b["data"]) == prog.read_bytes()
    assert b["addr"] == 0x03800000 and b["after"] == 1 and b["link"] is None
    ini = brain.files[(up.VID_USER, "slot_3.ini")]
    assert b"slot = 2" in ini["data"] and ini["kind"] == b"ini" and ini["after"] == 0


def test_hot_cold_links_the_hot_package_and_skips_a_cold_package_already_there(tmp_path):
    hot, cold = tmp_path / "hot.package.bin", tmp_path / "cold.package.bin"
    hot.write_bytes(os.urandom(3000))
    cold.write_bytes(XVX5 + os.urandom(50_000))
    brain = FakeBrain()
    first = up.upload(up.Brain(brain), up.plan(hot, cold, 1, "slot_1", "run"))
    lib = f"cold_{crc32_v5(cold.read_bytes()):08x}"
    assert first == [lib, "slot_1.ini", "slot_1.bin"]
    h = brain.files[(up.VID_USER, "slot_1.bin")]
    assert h["addr"] == 0x07800000 and h["link"] == (lib, up.VID_PROS)
    assert brain.files[(up.VID_PROS, lib)]["addr"] == 0x03800000
    # the next build only changes the hot package
    hot.write_bytes(os.urandom(3000))
    again = up.upload(up.Brain(brain), up.plan(hot, cold, 1, "slot_1", "run"))
    assert again == ["slot_1.ini", "slot_1.bin"]


def test_python_links_the_vm(tmp_path):
    main, vm = tmp_path / "main.py", tmp_path / "python_vm.bin"
    main.write_text("print('hi')\n")
    vm.write_bytes(XVX5 + os.urandom(1000))
    brain = FakeBrain()
    up.upload(up.Brain(brain), up.plan(main, vm, 2, "main", "run"))
    p = brain.files[(up.VID_USER, "slot_2.bin")]
    assert p["addr"] == 0x07000000 and p["kind"] == b"bina" and p["link"] == ("python_vm.bin", up.VID_VXVM)
    assert brain.files[(up.VID_VXVM, "python_vm.bin")]["version"] == 7          # the VM's header version


def test_wrong_pairs_and_non_programs_are_refused(tmp_path):
    (tmp_path / "x.bin").write_bytes(b"not a program")
    (tmp_path / "main.py").write_text("")
    (tmp_path / "cold.package.bin").write_bytes(XVX5)
    (tmp_path / "python_vm.bin").write_bytes(XVX5)
    with pytest.raises(up.UploadError, match="no XVX5"):
        up.plan(tmp_path / "x.bin", None, 1, "x", "run")
    with pytest.raises(up.UploadError, match="needs --link"):
        up.plan(tmp_path / "main.py", None, 1, "m", "run")
    with pytest.raises(up.UploadError, match="cold package"):
        up.plan(tmp_path / "main.py", tmp_path / "cold.package.bin", 1, "m", "run")
    with pytest.raises(up.UploadError, match="Python VM"):
        up.plan(tmp_path / "x.bin", tmp_path / "python_vm.bin", 1, "x", "run")


def test_a_lost_packet_is_sent_again(tmp_path):
    prog = tmp_path / "p.bin"
    prog.write_bytes(XVX5 + bytes(100))
    brain = FakeBrain(drop_first={up.FT_WRITE})
    up.upload(up.Brain(brain, timeout=0.05), up.plan(prog, None, 1, "p", "none"))
    assert (up.VID_USER, "slot_1.bin") in brain.files


def test_share_gives_an_uploader_the_port_to_itself(sock_dir):
    """v5usb_share.py: the shared side talks to the brain, until an uploader
    connects; then the brain's replies go to the uploader only, and the
    shared side's bytes are dropped."""
    brain_path, shared_path, host_path = (os.path.join(sock_dir, n) for n in ("brain", "shared", "host"))
    qemu = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    qemu.bind(brain_path)
    qemu.listen(1)
    share = subprocess.Popen([sys.executable, "-I", str(SCRIPTS / "v5usb_share.py"), brain_path, shared_path, host_path],
                             stdout=subprocess.DEVNULL)
    try:
        qemu.settimeout(5)
        brain, _ = qemu.accept()
        brain.settimeout(2)
        shared = connect(shared_path)
        shared.sendall(b"poll")
        assert brain.recv(100) == b"poll"
        brain.sendall(b"status")
        assert shared.recv(100) == b"status"

        host = connect(host_path)
        time.sleep(0.2)                          # let the share see the uploader first
        shared.sendall(b"poll2")                 # dropped: the uploader has the port
        host.sendall(b"upload")
        assert brain.recv(100) == b"upload"
        brain.sendall(b"ack")
        assert host.recv(100) == b"ack"
        shared.settimeout(0.3)
        with pytest.raises(socket.timeout):
            shared.recv(100)

        host.close()
        time.sleep(0.2)
        shared.sendall(b"poll3")
        assert brain.recv(100) == b"poll3"
    finally:
        share.terminate()
        share.wait(5)


def connect(path: str) -> socket.socket:
    for _ in range(50):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.connect(path)
            s.settimeout(2)
            return s
        except OSError:
            time.sleep(0.1)
    raise AssertionError(f"no {path}")
