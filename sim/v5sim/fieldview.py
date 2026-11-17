"""Live top-down field view for the simulated robot, served over HTTP.

The physics loop publishes snapshots with publish(); a background HTTP
server hands the latest one to a small canvas page that polls /state.json.
Open http://localhost:<port>/ in a browser (WSL forwards localhost).

The page draws in audience view (RW-Template's field frame) and can overlay
the V5RC Override starting layout (the game manual's Figure FO-2, served
from assets/override-field.png). With a controller attached, the page
also drives it: keys and a browser gamepad (Gamepad API, standard mapping,
e.g. Xbox) become sticks and buttons, posted to /input as JSON {"axes": {"Axis3": 127, ...},
"buttons": ["A", ...], "competition": "NONE|DRIVER|AUTONOMOUS|DISABLED"}.
Scripts can post the same JSON with Content-Type: application/json
(curl -H 'Content-Type: application/json' -d ... localhost:<port>/input);
other types get 415. {"gesture": "up|down|left|right"} swipes in front of
the optical sensors (keys 1-4) and {"tower": true|false} holds the Light
Tower button (key T).

With a BrainScreen (robot.py --screen-qmp), /screen shows the brain's screen:
/screen.png?since=N is its latest frame (304 while it is still frame N) and
POST /touch {"x": 0..479, "y": 0..271, "down": true|false} presses it (no
"down": a move while pressed).
"""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable
from urllib.parse import parse_qs, urlsplit

from . import brainscreen, dashboard, fieldcontrol

OVERRIDE_FIELD_PNG = Path(__file__).parent / "assets" / "override-field.png"
TRAIL_POINTS = 2000
TRAIL_MIN_STEP_M = 0.01
MAX_INPUT_BYTES = 4096

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>V5 Field View</title>
<style>
  :root { --bg:#14161a; --tile:#2a2e35; --line:#3b4049; --fg:#e8eaed;
          --robot:#4ea1ff; --opponent:#c05cff; --trail:#f2b84b; --muted:#9aa0a6;
          --red:#e5484d; --blue:#3e8ef7; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:14px system-ui, sans-serif; display:flex; flex-direction:column;
         align-items:center; padding:16px; gap:8px; }
  canvas { width:min(92vw, 75vh); height:min(92vw, 75vh); border-radius:6px; }
  #info { font-variant-numeric:tabular-nums; }
  #screen { margin:0; padding:6px 10px; background:#0b0c0e; color:#9ef0a8;
            border:1px solid var(--line); border-radius:4px;
            font:13px ui-monospace, monospace; min-width:19ch; }
  #pad { color:var(--muted); text-align:center; max-width:640px; }
  #pad button { background:var(--tile); color:var(--fg); border:1px solid var(--line);
                border-radius:4px; padding:4px 10px; margin:2px; cursor:pointer; }
  #pad button.on { background:var(--robot); color:#000; }
  #pad label { cursor:pointer; }
  #gamepad { color:var(--fg); }
</style></head><body>
<canvas id="c" width="720" height="720"></canvas>
<div id="info">waiting for the sim...</div>
<div><a href="dashboard" style="color:var(--robot)">device dashboard</a> &middot;
  <a href="field" style="color:var(--robot)">field control</a> &middot;
  <a href="screen" style="color:var(--robot)">brain screen</a> &middot;
  brain button: <button id="tapbtn" style="background:var(--tile);color:var(--fg);border:1px solid var(--line);border-radius:4px">tap</button></div>
<pre id="screen" hidden></pre>
<div id="pad">
  Controller (click the field first): W/S left stick Y, A/D right stick X,
  arrows right Y / left X, J=A K=B U=X I=Y, Q=L1 E=R1 Z=L2 C=R2.
  Or plug in an Xbox (or any standard) gamepad and press a button:
  sticks are the V5 sticks, A B X Y, LB RB = L1 R1, LT RT = L2 R2, d-pad = arrows.
  <div id="gamepad">no gamepad</div>
  <div><label><input type="checkbox" id="override"> Override field</label></div>
  <div id="comp"></div>
</div>
<script>
// Audience view, in the field frame VEX/RW-Template use: red wall left,
// audience at the bottom, +Y up. The sim's frame is turned: sim +x = RW +Y,
// sim +y = RW -X.
const IN = 0.0254, HALF_IN = 70.25, TILE_IN = 2 * HALF_IN / 6;
const FIELD = 2 * HALF_IN * IN, PAD = 12, TILES = 6, ROBOT = 0.4572, BLOCK = 0.045;
const c = document.getElementById('c'), g = c.getContext('2d');
const info = document.getElementById('info');
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n);
const px = m => m / FIELD * (c.width - 2 * PAD);
function rwCanvas(xin, yin) { return [c.width / 2 + px(xin * IN), c.height / 2 - px(yin * IN)]; }
function toCanvas(x, y) { return rwCanvas(-y / IN, x / IN); }
const overrideBox = document.getElementById('override');
try { overrideBox.checked = localStorage.getItem('override') !== 'off'; }
catch (e) { overrideBox.checked = true; }
overrideBox.onchange = () => {
  try { localStorage.setItem('override', overrideBox.checked ? 'on' : 'off'); } catch (e) {}
};

// ---- V5RC Override (2026-27) starting layout: Game Manual v1.1 Figure FO-2,
// cropped to the field plus WALL_PX of perimeter on each side.
const FIELD_PX = 686, WALL_PX = 12;
const overrideImg = new Image();
overrideImg.src = 'override-field.png';
function drawOverride(fx, fy, side) {
  if (!overrideImg.complete || !overrideImg.naturalWidth) return;
  const m = side * WALL_PX / FIELD_PX;
  g.drawImage(overrideImg, fx - m, fy - m, side + 2 * m, side + 2 * m);
}

function draw(s) {
  g.fillStyle = css('--bg'); g.fillRect(0, 0, c.width, c.height);
  const [fx, fy] = rwCanvas(-HALF_IN, HALF_IN), side = px(FIELD);
  g.fillStyle = css('--tile'); g.fillRect(fx, fy, side, side);
  g.strokeStyle = css('--line'); g.lineWidth = 2;
  for (let i = 1; i < TILES; i++) {
    const p = i * side / TILES;
    g.beginPath(); g.moveTo(fx + p, fy); g.lineTo(fx + p, fy + side); g.stroke();
    g.beginPath(); g.moveTo(fx, fy + p); g.lineTo(fx + side, fy + p); g.stroke();
  }
  if (overrideBox.checked) drawOverride(fx, fy, side);
  g.strokeStyle = css('--muted'); g.lineWidth = 4; g.strokeRect(fx, fy, side, side);
  g.strokeStyle = css('--trail'); g.lineWidth = 2; g.beginPath();
  s.trail.forEach(([x, y], i) => { const [a, b] = toCanvas(x, y);
    i ? g.lineTo(a, b) : g.moveTo(a, b); });
  g.stroke();
  g.strokeStyle = css('--muted'); g.lineWidth = 2; g.setLineDash([6, 4]);
  g.fillStyle = css('--muted'); g.font = '12px system-ui';
  for (const [name, [x0, y0, x1, y1]] of s.goals || []) {
    const [ax, ay] = toCanvas(x0, y0), [bx, by] = toCanvas(x1, y1);
    const [lx, ty] = [Math.min(ax, bx), Math.min(ay, by)];
    g.strokeRect(lx, ty, Math.abs(bx - ax), Math.abs(by - ay));
    g.fillText(name, lx + 3, ty + 13);
  }
  g.setLineDash([]);
  g.fillStyle = css('--muted');
  for (const [kind, x, y, sx, sy, r, ang] of s.obstacles || []) {
    const [ox, oy] = toCanvas(x, y);
    g.save(); g.translate(ox, oy); g.rotate(-ang); g.globalAlpha = 0.8;
    g.beginPath();
    if (kind === 'box') g.rect(-px(sy * 2) / 2, -px(sx * 2) / 2,
                               px(sy * 2), px(sx * 2));
    else g.arc(0, 0, px(r), 0, 2 * Math.PI);
    g.fill(); g.restore();
  }
  for (const [x, y, h] of s.opponents || []) {
    const [ox, oy] = toCanvas(x, y);
    g.save(); g.translate(ox, oy); g.rotate(-h - Math.PI / 2);
    g.fillStyle = css('--opponent'); g.globalAlpha = 0.85;
    g.fillRect(-px(ROBOT) / 2, -px(ROBOT) / 2, px(ROBOT), px(ROBOT));
    g.globalAlpha = 1; g.strokeStyle = css('--fg'); g.lineWidth = 2;
    g.beginPath(); g.moveTo(0, 0); g.lineTo(px(ROBOT) * 0.6, 0); g.stroke();
    g.restore();
  }
  for (const [x, y, color] of s.blocks || []) {
    const [bx, by] = toCanvas(x, y);
    g.fillStyle = css(color === 'red' ? '--red' : '--blue');
    g.beginPath(); g.arc(bx, by, px(BLOCK), 0, 2 * Math.PI); g.fill();
  }
  const [rx, ry] = toCanvas(s.x, s.y);
  g.save(); g.translate(rx, ry); g.rotate(-s.heading - Math.PI / 2);   // sim +x is up
  g.fillStyle = css('--robot'); g.globalAlpha = 0.85;
  g.fillRect(-px(ROBOT) / 2, -px(ROBOT) / 2, px(ROBOT), px(ROBOT));
  g.globalAlpha = 1; g.strokeStyle = css('--fg'); g.lineWidth = 3;
  g.beginPath(); g.moveTo(0, 0); g.lineTo(px(ROBOT) * 0.6, 0); g.stroke();
  g.restore();
  // RW-Template / mikLib coordinates: inches, heading clockwise from +Y
  const rwHeading = ((-s.heading * 180 / Math.PI) % 360 + 360) % 360;
  info.textContent = `t ${s.t.toFixed(1)} s   x ${(-s.y / IN).toFixed(1)} in   ` +
    `y ${(s.x / IN).toFixed(1)} in   heading ${rwHeading.toFixed(1)}°   ` +
    `v ${s.v.toFixed(2)} m/s` +
    (s.held && s.held.length ? `   holding ${s.held.join(' ')}` : '') +
    (s.score ? `   score red ${s.score.red} blue ${s.score.blue}` : '');
  const scr = document.getElementById('screen');
  scr.hidden = !(s.screen && s.screen.length);
  if (!scr.hidden) scr.textContent = s.screen.map(l => l.padEnd(19)).join('\\n');
}
async function poll() {
  try { draw(await (await fetch('state.json')).json()); }
  catch (e) { info.textContent = 'sim not reachable'; }
  setTimeout(poll, 50);
}
poll();

const AXES = { w: ['Axis3', 127], s: ['Axis3', -127], d: ['Axis1', 127],
  a: ['Axis1', -127], ArrowUp: ['Axis2', 127], ArrowDown: ['Axis2', -127],
  ArrowRight: ['Axis4', 127], ArrowLeft: ['Axis4', -127] };
const BUTTONS = { j: 'A', k: 'B', u: 'X', i: 'Y', q: 'L1', e: 'R1', z: 'L2',
  c: 'R2' };
const held = new Set();
let competition = 'NONE';
let pad = { axes: {}, buttons: [] };       // the gamepad's share of the input
function send() {
  const axes = { ...pad.axes }, buttons = new Set(pad.buttons);
  for (const k of held) {
    if (AXES[k]) axes[AXES[k][0]] = (axes[AXES[k][0]] || 0) + AXES[k][1];
    if (BUTTONS[k]) buttons.add(BUTTONS[k]);
  }
  for (const a in axes) axes[a] = Math.max(-127, Math.min(127, Math.round(axes[a])));
  const body = JSON.stringify({ axes, buttons: [...buttons], competition });
  fetch('input', { method: 'POST',
    headers: { 'Content-Type': 'application/json' }, body }).catch(() => {});
}

// Gamepads (Xbox and other "standard" mappings) through the Gamepad API.
// Browsers only reveal a pad after a button press on it.
const DEADZONE = 0.08;
const PAD_BUTTONS = { 0: 'A', 1: 'B', 2: 'X', 3: 'Y', 4: 'L1', 5: 'R1', 6: 'L2', 7: 'R2',
  12: 'Up', 13: 'Down', 14: 'Left', 15: 'Right' };
const padInfo = document.getElementById('gamepad');
const stick = v => Math.abs(v) < DEADZONE ? 0 : Math.round(v * 127);   // whole steps: no resend on jitter
function readPad() {
  const gp = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find(p => p && p.connected);
  let next = { axes: {}, buttons: [] };
  if (gp) {
    const ax = gp.axes;
    next.axes = { Axis4: stick(ax[0] || 0), Axis3: -stick(ax[1] || 0),
                  Axis1: stick(ax[2] || 0), Axis2: -stick(ax[3] || 0) };
    gp.buttons.forEach((b, i) => {
      if (PAD_BUTTONS[i] && (b.pressed || b.value > 0.5)) next.buttons.push(PAD_BUTTONS[i]);
    });
    const live = Object.entries(next.axes).filter(([, v]) => v).map(([a, v]) => `${a} ${v}`)
      .concat(next.buttons).join(', ');
    padInfo.textContent = `gamepad: ${gp.id}` +
      (gp.mapping === 'standard' ? '' : ' (non-standard mapping: buttons may be off)') +
      ` | ${live || 'centred, no buttons'}` +
      (competition === 'DISABLED' ? ' | competition is DISABLED: the robot ignores input' : '');
  } else {
    padInfo.textContent = 'no gamepad seen: with this tab in front, press A on the controller' +
      (navigator.getGamepads ? '' : ' (this browser has no Gamepad API)');
  }
  const before = JSON.stringify(pad);
  pad = next;
  if (JSON.stringify(pad) !== before) send();
  requestAnimationFrame(readPad);
}
requestAnimationFrame(readPad);
addEventListener('gamepadconnected', e => console.log('gamepad connected:', e.gamepad.id, e.gamepad.mapping));
addEventListener('gamepaddisconnected', e => console.log('gamepad disconnected:', e.gamepad.id));
const GESTURES = { '1': 'up', '2': 'down', '3': 'left', '4': 'right' };
function post(msg) {
  fetch('input', { method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(msg) }).catch(() => {});
}
function key(e, down) {
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (k in GESTURES || k === 't') {
    e.preventDefault();
    if (e.repeat) return;
    if (k === 't') post({ tower: down });
    else if (down) post({ gesture: GESTURES[k] });
    return;
  }
  if (!(k in AXES) && !(k in BUTTONS)) return;
  e.preventDefault();
  const had = held.has(k);
  down ? held.add(k) : held.delete(k);
  if (had !== down) send();
}
document.getElementById('tapbtn').onclick = () => post({ brain_button: 0.2 });
addEventListener('keydown', e => key(e, true));
addEventListener('keyup', e => key(e, false));
addEventListener('blur', () => { held.clear(); send(); });
const comp = document.getElementById('comp');
for (const mode of ['NONE', 'DRIVER', 'AUTONOMOUS', 'DISABLED']) {
  const b = document.createElement('button');
  b.textContent = mode.toLowerCase();
  b.onclick = () => { competition = mode;
    comp.querySelectorAll('button').forEach(x => x.classList.toggle('on', x === b));
    send(); };
  if (mode === 'NONE') b.classList.add('on');
  comp.appendChild(b);
}
</script></body></html>
"""


class FieldView:
    def __init__(self, port: int,
                 on_input: Callable[[dict], None] | None = None,
                 screen: "brainscreen.BrainScreen | None" = None):
        self._lock = threading.Lock()
        self._state = {"t": 0.0, "x": 0.0, "y": 0.0, "heading": 0.0,
                       "v": 0.0, "trail": []}
        self._trail: list[tuple[float, float]] = []
        self._devices: dict = {"t": 0.0, "devices": []}
        self._field: dict = {"t": 0.0, "competition": "NONE", "match": None}
        self._on_input = on_input
        self._screen = screen
        view = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith("/state.json"):
                    with view._lock:
                        body = json.dumps(view._state).encode()
                    ctype = "application/json"
                elif self.path.startswith("/devices.json"):
                    with view._lock:
                        body = json.dumps(view._devices).encode()
                    ctype = "application/json"
                elif self.path.startswith("/field.json"):
                    with view._lock:
                        body = json.dumps(view._field).encode()
                    ctype = "application/json"
                elif self.path in ("/field", "/field.html"):
                    body, ctype = fieldcontrol.PAGE.encode(), "text/html; charset=utf-8"
                elif self.path in ("/", "/index.html"):
                    body, ctype = PAGE.encode(), "text/html; charset=utf-8"
                elif self.path in ("/dashboard", "/dashboard.html"):
                    body, ctype = dashboard.PAGE.encode(), "text/html; charset=utf-8"
                elif self.path == "/override-field.png":
                    body, ctype = OVERRIDE_FIELD_PNG.read_bytes(), "image/png"
                elif view._screen and self.path in ("/screen", "/screen.html"):
                    body, ctype = brainscreen.PAGE.encode(), "text/html; charset=utf-8"
                elif view._screen and self.path.startswith("/screen.png"):
                    self._screen_png()
                    return
                else:
                    self.send_error(404)
                    return
                self._reply(200, body, ctype)

            def _screen_png(self):
                """The latest frame; 304 when it is still frame ?since=N."""
                query = parse_qs(urlsplit(self.path).query)
                try:
                    since = int(query.get("since", ["-1"])[0])
                except ValueError:
                    self.send_error(400, "since is a frame number")
                    return
                try:
                    n, png = view._screen.frame()
                except (OSError, RuntimeError, ValueError) as e:
                    self.send_error(503, f"brain screen: {e}")
                    return
                if n == since:
                    self.send_response(304)
                    self.send_header("X-Frame", str(n))
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Frame", str(n))
                self.send_header("Content-Length", str(len(png)))
                self.end_headers()
                self.wfile.write(png)

            def do_POST(self):
                if self.path == "/touch" and view._screen:
                    self._post_json(self._touch)
                    return
                if self.path != "/input" or view._on_input is None:
                    self.send_error(404)
                    return
                self._post_json(view._on_input)

            def _touch(self, msg):
                down = msg.get("down")
                if down is not None and not isinstance(down, bool):
                    raise ValueError("down is true, false or absent (a move)")
                x, y = msg["x"], msg["y"]
                if not all(isinstance(v, (int, float)) for v in (x, y)):
                    raise ValueError("x and y are screen pixels")
                try:
                    view._screen.touch(x, y, down)
                except (OSError, RuntimeError) as e:
                    raise ValueError(f"brain screen: {e}") from e

            def _post_json(self, handle):
                # Not a CORS "simple" type, so other web pages can't post here.
                if self.headers.get_content_type() != "application/json":
                    self.send_error(415, "need Content-Type: application/json")
                    return
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                except ValueError:
                    length = 0
                if not 0 < length <= MAX_INPUT_BYTES:
                    self.send_error(400, "bad length")
                    return
                try:
                    msg = json.loads(self.rfile.read(length))
                    if not isinstance(msg, dict):
                        raise ValueError("expected an object")
                    handle(msg)
                except (ValueError, KeyError, TypeError) as e:
                    self.send_error(400, str(e))
                    return
                self._reply(204, b"", "text/plain")

            def _reply(self, code, body, ctype):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        self._server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def publish(self, t: float, x: float, y: float, heading: float,
                v: float, blocks: list[tuple[float, float, str]] = (),
                held: list[str] = (), screen: list[str] = (),
                goals: list[tuple[str, tuple]] = (),
                score: dict | None = None, obstacles: list[list] = (),
                opponents: list[tuple[float, float, float]] = ()) -> None:
        last = self._trail[-1] if self._trail else None
        if last is None or (x - last[0]) ** 2 + (y - last[1]) ** 2 >= TRAIL_MIN_STEP_M ** 2:
            self._trail = (self._trail + [(round(x, 4), round(y, 4))])[-TRAIL_POINTS:]
        with self._lock:
            self._state = {"t": t, "x": x, "y": y, "heading": heading, "v": v,
                           "trail": self._trail,
                           "blocks": [[round(bx, 4), round(by, 4), c]
                                      for bx, by, c in blocks],
                           "held": list(held), "screen": list(screen),
                           "goals": [[n, list(r)] for n, r in goals],
                           "score": score, "obstacles": list(obstacles),
                           "opponents": [[round(v, 4) for v in o] for o in opponents]}

    def publish_devices(self, t: float, devices: list[dict]) -> None:
        """Every device's state for /devices.json (the dashboard)."""
        with self._lock:
            self._devices = {"t": t, "devices": devices}

    def publish_field(self, t: float, field: dict) -> None:
        """The competition state for /field.json (the field-control page)."""
        with self._lock:
            self._field = {**field, "t": t}

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
