import json
import socket
import struct
import threading
import urllib.error
import urllib.request
import zlib

import pytest

from v5sim.brainscreen import (SCREEN_H, SCREEN_W, BrainScreen, crop_screen,
                               encode_png, parse_ppm)
from v5sim.fieldview import FieldView


def decode_png(data: bytes) -> tuple[int, int, bytes]:
    """(w, h, RGB) of an 8-bit RGB, filter-0 PNG (what encode_png writes)."""
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    i, idat = 8, b""
    w = h = 0
    while i < len(data):
        n, kind = struct.unpack(">I4s", data[i:i + 8])
        body = data[i + 8:i + 8 + n]
        crc, = struct.unpack(">I", data[i + 8 + n:i + 12 + n])
        assert crc == zlib.crc32(kind + body) & 0xFFFFFFFF
        if kind == b"IHDR":
            w, h, depth, ctype = struct.unpack(">IIBB", body[:10])
            assert (depth, ctype) == (8, 2)
        elif kind == b"IDAT":
            idat += body
        i += 12 + n
    raw = zlib.decompress(idat)
    rows = [raw[r * (w * 3 + 1):(r + 1) * (w * 3 + 1)] for r in range(h)]
    assert all(row[0] == 0 for row in rows)
    return w, h, b"".join(row[1:] for row in rows)


def ppm(w, h, pixels: bytes) -> bytes:
    return f"P6\n{w} {h}\n255\n".encode() + pixels


class FakeQmp:
    """A QMP server on a unix socket: a canvas with the screen at (sx, sy),
    screendump writes it, input-send-event is recorded."""

    def __init__(self, path, canvas=(SCREEN_W, SCREEN_H, 0, 0)):
        self.path, self.canvas = path, canvas
        self.events: list[list[dict]] = []
        self.dumps = 0
        self.colour = b"\x10\x20\x30"
        self.srv = socket.socket(socket.AF_UNIX)
        self.srv.bind(path)
        self.srv.listen(1)
        threading.Thread(target=self._serve, daemon=True).start()

    def picture(self) -> bytes:
        w, h, sx, sy = self.canvas
        px = bytearray(b"\xff\x00\x00" * (w * h))       # red brain skin
        for r in range(SCREEN_H):
            for x in (0, SCREEN_W - 1):
                px[((sy + r) * w + sx + x) * 3:((sy + r) * w + sx + x) * 3 + 3] = self.colour
            start = ((sy + r) * w + sx + 1) * 3
            px[start:start + (SCREEN_W - 2) * 3] = self.colour * (SCREEN_W - 2)
        return bytes(px)

    def _serve(self):
        while True:
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            with conn:
                f = conn.makefile("rwb")
                f.write(b'{"QMP": {}}\n'); f.flush()
                for line in f:
                    msg = json.loads(line)
                    name, args = msg["execute"], msg.get("arguments", {})
                    ret: object = {}
                    if name == "qom-get":
                        w, h, sx, sy = self.canvas
                        ret = {"canvas-w": w, "canvas-h": h,
                               "screen-x": sx, "screen-y": sy}[args["property"]]
                    elif name == "screendump":
                        self.dumps += 1
                        with open(args["filename"], "wb") as out:
                            out.write(ppm(self.canvas[0], self.canvas[1], self.picture()))
                    elif name == "input-send-event":
                        self.events.append(args["events"])
                    f.write(json.dumps({"return": ret}).encode() + b"\n"); f.flush()

    def close(self):
        self.srv.close()


def test_png_round_trip():
    rgb = bytes(range(256)) * 3                 # 16 x 16 pixels
    assert decode_png(encode_png(rgb, 16, 16)) == (16, 16, rgb)
    with pytest.raises(ValueError):
        encode_png(rgb[:-1], 16, 16)


def test_parse_ppm_skips_comments():
    data = b"P6\n# by qemu\n2 1\n255\n" + b"\x01\x02\x03\x04\x05\x06"
    assert parse_ppm(data) == (2, 1, b"\x01\x02\x03\x04\x05\x06")
    with pytest.raises(ValueError):
        parse_ppm(b"P5\n2 1\n255\n\x00\x00")


def test_crop_takes_the_screen_out_of_the_skin():
    w = SCREEN_W + 10
    px = bytes(range(256)) * ((w * (SCREEN_H + 5) * 3) // 256 + 1)
    px = px[:w * (SCREEN_H + 5) * 3]
    out = crop_screen(px, w, 4, 3)
    assert len(out) == SCREEN_W * SCREEN_H * 3
    assert out[:3] == px[(3 * w + 4) * 3:(3 * w + 4) * 3 + 3]


@pytest.mark.parametrize("canvas", [(SCREEN_W, SCREEN_H, 0, 0), (600, 400, 40, 60)])
def test_frames_and_touch(sock_dir, canvas):
    qmp = FakeQmp(f"{sock_dir}/q.sock", canvas)
    screen = BrainScreen(qmp.path, f"{sock_dir}/s.ppm", max_fps=float("inf"))   # no rate limit
    try:
        n, png = screen.frame()
        w, h, rgb = decode_png(png)
        assert (w, h) == (SCREEN_W, SCREEN_H)
        assert rgb == qmp.colour * (SCREEN_W * SCREEN_H)   # no skin in it
        assert screen.frame()[0] == n                      # unchanged picture
        qmp.colour = b"\x40\x50\x60"
        assert screen.frame()[0] == n + 1

        screen.touch(100, 50, True)
        screen.touch(110, 60, None)
        screen.touch(110, 60, False)
        cw, ch, sx, sy = canvas
        def pos(x, y):
            return [{"type": "abs", "data": {"axis": "x", "value": ((x + sx) * 0x8000 + cw // 2) // cw}},
                    {"type": "abs", "data": {"axis": "y", "value": ((y + sy) * 0x8000 + ch // 2) // ch}}]
        btn = lambda d: {"type": "btn", "data": {"button": "left", "down": d}}
        assert qmp.events == [pos(100, 50) + [btn(True)], pos(110, 60),
                              pos(110, 60) + [btn(False)]]
        with pytest.raises(ValueError):
            screen.touch(480, 0, True)
    finally:
        screen.close()
        qmp.close()


def test_frames_are_rate_limited(sock_dir):
    qmp = FakeQmp(f"{sock_dir}/q.sock")
    screen = BrainScreen(qmp.path, f"{sock_dir}/s.ppm", max_fps=1)
    try:
        for _ in range(5):
            screen.frame()
        assert qmp.dumps == 1
    finally:
        screen.close()
        qmp.close()


def test_http_screen_routes(sock_dir):
    qmp = FakeQmp(f"{sock_dir}/q.sock")
    screen = BrainScreen(qmp.path, f"{sock_dir}/s.ppm", max_fps=float("inf"))   # no rate limit
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    view = FieldView(port, None, screen)
    base = f"http://127.0.0.1:{port}"
    try:
        with urllib.request.urlopen(base + "/screen", timeout=5) as r:
            assert b"<canvas" in r.read()
        with urllib.request.urlopen(base + "/screen.png", timeout=5) as r:
            frame = int(r.headers["X-Frame"])
            assert r.headers.get_content_type() == "image/png"
            assert decode_png(r.read())[:2] == (SCREEN_W, SCREEN_H)
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(f"{base}/screen.png?since={frame}", timeout=5)
        assert e.value.code == 304

        def touch(body, ctype="application/json"):
            req = urllib.request.Request(base + "/touch", data=json.dumps(body).encode(),
                                         method="POST", headers={"Content-Type": ctype})
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status
        assert touch({"x": 10, "y": 20, "down": True}) == 204
        assert qmp.events[-1][-1]["data"]["down"] is True
        for bad in ({"x": 10}, {"x": 500, "y": 1}, {"x": 1, "y": 1, "down": "yes"}):
            with pytest.raises(urllib.error.HTTPError) as e:
                touch(bad)
            assert e.value.code == 400
        with pytest.raises(urllib.error.HTTPError) as e:
            touch({"x": 1, "y": 1}, "text/plain")
        assert e.value.code == 415
    finally:
        view.close()
        screen.close()
        qmp.close()


def test_no_screen_routes_without_a_screen():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    view = FieldView(port)
    try:
        for path in ("/screen", "/screen.png"):
            with pytest.raises(urllib.error.HTTPError) as e:
                urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=5)
            assert e.value.code == 404
    finally:
        view.close()
