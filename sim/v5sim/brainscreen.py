"""The brain's screen and touch over the field view's HTTP server.

QEMU's own window (SDL) shows the screen only on the machine running it. For
pages that show it elsewhere (the VS Code extension, a browser), this keeps a
QMP connection to the brain open (its own socket, v5qmp-screen.sock: QMP takes
one client at a time) and

- grabs frames with `screendump`, cuts the 480x272 screen out of the brain
  picture a windowed brain draws around it, and encodes them as PNG;
- presses the screen with `input-send-event` (absolute x/y plus the left
  button), the same events a click in the window sends.

Frames are taken on demand, at most MAX_FPS a second, so nothing runs while
no page is looking. The geometry follows scripts/v5ctl.py (geometry, _frame).
"""
from __future__ import annotations

import os
import struct
import threading
import time
import zlib

from .brainctl import Qmp

SCREEN_W, SCREEN_H = 480, 272
ABS_MAX = 0x7FFF
MAX_FPS = 20.0


def encode_png(rgb: bytes, w: int, h: int, level: int = 1) -> bytes:
    """8-bit RGB PNG from raw rows (filter 0 on every row)."""
    stride = w * 3
    if len(rgb) != stride * h:
        raise ValueError(f"expected {stride * h} bytes of RGB, got {len(rgb)}")
    raw = b"".join(b"\x00" + rgb[r * stride:(r + 1) * stride] for r in range(h))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, level))
            + chunk(b"IEND", b""))


def parse_ppm(data: bytes) -> tuple[int, int, bytes]:
    """(width, height, RGB bytes) of a binary P6 PPM with maxval 255, as
    QEMU's screendump writes it."""
    fields: list[bytes] = []
    i = 0
    while len(fields) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":                 # comment line
            i = data.index(b"\n", i) + 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    if fields[0] != b"P6" or fields[3] != b"255":
        raise ValueError("not an 8-bit P6 PPM")
    w, h = int(fields[1]), int(fields[2])
    pixels = data[i + 1:i + 1 + w * h * 3]
    if len(pixels) != w * h * 3:
        raise ValueError("PPM is short")
    return w, h, pixels


def crop_screen(pixels: bytes, canvas_w: int, sx: int, sy: int) -> bytes:
    """The 480x272 screen at (sx, sy) out of a canvas canvas_w wide."""
    if (canvas_w, sx, sy) == (SCREEN_W, 0, 0):
        return pixels[:SCREEN_W * SCREEN_H * 3]
    return b"".join(
        pixels[((sy + r) * canvas_w + sx) * 3:((sy + r) * canvas_w + sx + SCREEN_W) * 3]
        for r in range(SCREEN_H))


def _abs(axis: str, pixels: int, size: int) -> dict:
    return {"type": "abs", "data": {"axis": axis,
                                    "value": (pixels * (ABS_MAX + 1) + size // 2) // size}}


def _btn(down: bool) -> dict:
    return {"type": "btn", "data": {"button": "left", "down": down}}


class BrainScreen:
    def __init__(self, qmp_path: str, dump_path: str | None = None,
                 max_fps: float = MAX_FPS):
        self.qmp_path = qmp_path
        self.dump_path = dump_path or qmp_path + ".ppm"
        self.min_interval = 1.0 / max_fps
        self._lock = threading.Lock()
        self._qmp: Qmp | None = None
        self._geom: tuple[int, int, int, int] | None = None
        self._png: bytes | None = None
        self._rgb: bytes | None = None
        self._taken = 0.0
        self.frame_no = 0

    # -- QMP ------------------------------------------------------------
    def _conn(self) -> Qmp:
        if self._qmp is None:
            self._qmp = Qmp(self.qmp_path)
            self._geom = None
        return self._qmp

    def _cmd(self, name: str, **args):
        """One command, reconnecting once if the brain dropped the link."""
        for attempt in (0, 1):
            try:
                return self._conn().cmd(name, **args)
            except (OSError, ConnectionError):
                self._drop()
                if attempt:
                    raise

    def _drop(self) -> None:
        if self._qmp is not None:
            try:
                self._qmp.close()
            except OSError:
                pass
        self._qmp = None

    def geometry(self) -> tuple[int, int, int, int]:
        """(canvas w, h, screen x, y): a windowed brain draws its screen
        inside a picture of the brain; touches cover the whole picture."""
        if self._geom is None:
            try:
                self._geom = tuple(  # type: ignore[assignment]
                    int(self._cmd("qom-get", path="/machine/display", property=name))
                    for name in ("canvas-w", "canvas-h", "screen-x", "screen-y"))
            except RuntimeError:                 # an older machine: bare screen
                self._geom = (SCREEN_W, SCREEN_H, 0, 0)
        return self._geom                        # type: ignore[return-value]

    # -- frames -----------------------------------------------------------
    def frame(self) -> tuple[int, bytes]:
        """(frame number, PNG). A frame newer than 1/max_fps is reused; the
        number only goes up when the picture changed."""
        with self._lock:
            now = time.monotonic()
            if self._png is None or now - self._taken >= self.min_interval:
                self._taken = now
                self._cmd("screendump", filename=self.dump_path)
                w, _, sx, sy = self.geometry()
                with open(self.dump_path, "rb") as f:
                    cw, _, pixels = parse_ppm(f.read())
                rgb = crop_screen(pixels, cw if cw else w, sx, sy)
                if rgb != self._rgb:
                    self._rgb = rgb
                    self._png = encode_png(rgb, SCREEN_W, SCREEN_H)
                    self.frame_no += 1
            return self.frame_no, self._png    # type: ignore[return-value]

    # -- touch --------------------------------------------------------------
    def touch(self, x: float, y: float, down: bool | None) -> None:
        """Finger at screen pixel (x, y): down=True presses, False lifts,
        None moves a finger already down."""
        if not (0 <= x < SCREEN_W and 0 <= y < SCREEN_H):
            raise ValueError(f"touch must be inside {SCREEN_W}x{SCREEN_H}")
        with self._lock:
            w, h, sx, sy = self.geometry()
            events = [_abs("x", int(x) + sx, w), _abs("y", int(y) + sy, h)]
            if down is not None:
                events.append(_btn(down))
            self._cmd("input-send-event", events=events)

    def close(self) -> None:
        with self._lock:
            self._drop()
        try:
            os.unlink(self.dump_path)
        except OSError:
            pass


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>V5 Brain Screen</title>
<style>
  :root { --bg:#111317; --card:#1a1d23; --card2:#22262d; --line:#2f343d; --fg:#e8eaed;
          --muted:#9aa0a6; --dim:#6b7079; --accent:#4ea1ff; --chrome:72px; }
  * { box-sizing:border-box; }
  html, body { margin:0; height:100%; background:var(--bg); color:var(--fg);
               font:14px/1.4 system-ui, sans-serif; }
  body { display:flex; flex-direction:column; }
  body.menu-open { --chrome:122px; }
  header { display:flex; align-items:center; gap:12px; flex-wrap:wrap; padding:10px 16px;
           border-bottom:1px solid var(--line); background:var(--card); }
  header[hidden] { display:none; }
  header h1 { font-size:15px; margin:0 8px 0 0; letter-spacing:.02em; }
  nav { display:flex; gap:4px; flex-wrap:wrap; }
  nav a { color:var(--muted); text-decoration:none; padding:4px 10px; border-radius:999px; }
  nav a:hover { color:var(--fg); background:var(--card2); }
  nav a.on { color:var(--fg); background:var(--card2); box-shadow:inset 0 0 0 1px var(--line); }
  main { flex:1; min-height:0; display:flex; flex-direction:column; align-items:center;
         justify-content:center; gap:10px; padding:10px; }
  #frame { position:relative; width:min(100%, calc((100vh - var(--chrome)) * 480 / 272));
           aspect-ratio:480/272; background:#000; border:8px solid var(--card);
           box-shadow:0 0 0 1px var(--line); border-radius:12px; box-sizing:content-box; }
  canvas { width:100%; height:100%; display:block; image-rendering:pixelated;
           touch-action:none; cursor:pointer; border-radius:4px; }
  #msg { position:absolute; inset:0; display:flex; align-items:center;
         justify-content:center; color:var(--muted); pointer-events:none; }
  #msg[hidden] { display:none; }   /* display:flex above would beat [hidden] */
  #bar { display:flex; gap:6px; align-items:center; flex-wrap:wrap; justify-content:center;
         font-size:12px; color:var(--muted); }
  #bar .sep { width:1px; align-self:stretch; background:var(--line); margin:0 4px; }
  button { font:inherit; color:var(--fg); background:var(--card2); border:1px solid var(--line);
           border-radius:6px; padding:4px 10px; cursor:pointer; }
  button:hover { border-color:var(--accent); }
  #menubtn[aria-expanded="true"] { border-color:var(--accent); color:var(--accent); }
  #fps { min-width:5em; font-variant-numeric:tabular-nums; }
</style></head><body>
<header id="menu" hidden>
  <h1>V5 Simulator</h1>
  <nav><a href="./">Field</a><a href="dashboard">Devices</a><a href="controller">Controller</a>
       <a href="field">Field control</a><a class="on" href="screen">Brain screen</a></nav>
</header>
<main>
  <div id="frame"><canvas id="c" width="480" height="272"></canvas>
    <div id="msg">waiting for the brain...</div></div>
  <div id="bar">
    <button id="menubtn" aria-controls="menu" aria-expanded="false"
            title="Show or hide the page menu">&#9776; Menu</button>
    <span class="sep"></span>
    <span>Brain button</span>
    <button data-secs="0.2" title="Back / Home">Tap</button>
    <button data-secs="1.5" title="stops a running program">Hold 1.5 s</button>
    <button data-secs="5" title="with no program running: powers off (ends the sim)">Hold 5 s</button>
    <span id="fps"></span>
  </div>
</main>
<script>
const c = document.getElementById('c'), g = c.getContext('2d');
const msg = document.getElementById('msg'), fps = document.getElementById('fps');
let frame = 0, frames = 0, last = performance.now();

async function poll() {
  for (;;) {
    try {
      const r = await fetch('screen.png?since=' + frame, { cache: 'no-store' });
      if (r.status === 200) {
        frame = +r.headers.get('X-Frame') || frame + 1;
        g.drawImage(await createImageBitmap(await r.blob()), 0, 0);
        msg.hidden = true; frames++;
      } else if (r.status !== 304) {
        throw new Error('HTTP ' + r.status);
      }
      await new Promise(ok => setTimeout(ok, 40));
    } catch (e) {
      msg.hidden = false; msg.textContent = 'waiting for the brain...';
      await new Promise(ok => setTimeout(ok, 1000));
    }
    const now = performance.now();
    if (now - last > 1000) { fps.textContent = frames + ' fps'; frames = 0; last = now; }
  }
}

// posts in order: a move must not overtake its press
let queue = Promise.resolve();
function post(path, body) {
  queue = queue.then(() => fetch(path, { method: 'POST',
    headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
    .catch(() => {}));
}
function at(e) {
  const b = c.getBoundingClientRect();
  return { x: Math.max(0, Math.min(479, Math.floor((e.clientX - b.left) * 480 / b.width))),
           y: Math.max(0, Math.min(271, Math.floor((e.clientY - b.top) * 272 / b.height))) };
}
let down = false, lastMove = 0;
c.addEventListener('pointerdown', e => {
  c.setPointerCapture(e.pointerId); down = true; post('touch', { ...at(e), down: true });
});
c.addEventListener('pointermove', e => {
  const now = performance.now();
  if (!down || now - lastMove < 30) return;
  lastMove = now; post('touch', at(e));
});
const up = e => { if (!down) return; down = false; post('touch', { ...at(e), down: false }); };
c.addEventListener('pointerup', up);
c.addEventListener('pointercancel', up);
document.querySelectorAll('button[data-secs]').forEach(b =>
  b.onclick = () => post('input', { brain_button: +b.dataset.secs }));

// the page menu (links to the other pages): hidden by default so the screen
// gets the room; the choice is remembered in this browser
const menu = document.getElementById('menu'), menubtn = document.getElementById('menubtn');
const MENU_KEY = 'v5sim.screen.menu';
function showMenu(open) {
  menu.hidden = !open;
  menubtn.setAttribute('aria-expanded', String(open));
  document.body.classList.toggle('menu-open', open);
}
try { showMenu(localStorage.getItem(MENU_KEY) === 'open'); } catch (e) { showMenu(false); }
menubtn.onclick = () => {
  menu.hidden = !menu.hidden;
  showMenu(!menu.hidden);
  try { localStorage.setItem(MENU_KEY, menu.hidden ? 'closed' : 'open'); } catch (e) {}
};
poll();
</script></body></html>
"""
