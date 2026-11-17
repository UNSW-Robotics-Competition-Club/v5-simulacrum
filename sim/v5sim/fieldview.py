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

from . import brainscreen, controllerpage, dashboard, fieldcontrol

OVERRIDE_FIELD_PNG = Path(__file__).parent / "assets" / "override-field.png"
TRAIL_POINTS = 2000
TRAIL_MIN_STEP_M = 0.01
MAX_INPUT_BYTES = 4096

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>V5 Field View</title>
<style>
  :root { --bg:#111317; --card:#1a1d23; --card2:#22262d; --line:#2f343d; --fg:#e8eaed;
          --muted:#9aa0a6; --dim:#6b7079; --accent:#4ea1ff;
          --tile:#2a2e35; --tile2:#2e333b; --seam:#3a404a;
          --robot:#4ea1ff; --opponent:#c05cff; --trail:#f2b84b;
          --red:#e5484d; --blue:#3e8ef7; --auton:#f2a93b; --driver:#3cc46c; --off:#6b7079; }
  * { box-sizing:border-box; }
  [hidden] { display:none !important; }
  body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.4 system-ui, sans-serif; }
  header { display:flex; align-items:center; gap:12px; flex-wrap:wrap; padding:10px 16px;
           border-bottom:1px solid var(--line); background:var(--card); position:sticky; top:0; z-index:1; }
  header h1 { font-size:15px; margin:0 8px 0 0; letter-spacing:.02em; }
  nav { display:flex; gap:4px; flex-wrap:wrap; }
  nav a { color:var(--muted); text-decoration:none; padding:4px 10px; border-radius:999px; }
  nav a:hover { color:var(--fg); background:var(--card2); }
  nav a.on { color:var(--fg); background:var(--card2); box-shadow:inset 0 0 0 1px var(--line); }
  header .spacer { flex:1; }
  button { font:inherit; color:var(--fg); background:var(--card2); border:1px solid var(--line);
           border-radius:6px; padding:5px 10px; cursor:pointer; }
  button:hover { border-color:var(--accent); }
  main { display:grid; grid-template-columns:minmax(0, 1fr) 320px; gap:16px; padding:16px;
         max-width:1280px; margin:0 auto; align-items:start; }
  @media (max-width: 900px) { main { grid-template-columns:1fr; } }
  .fieldwrap { position:relative; background:var(--card); border:1px solid var(--line);
               border-radius:10px; padding:12px; }
  canvas { display:block; width:min(100%, max(320px, calc(100vh - 120px))); aspect-ratio:1;
           margin:0 auto; border-radius:6px; outline:none; }
  canvas:focus-visible { box-shadow:0 0 0 2px var(--accent); }
  #info { position:absolute; left:24px; bottom:22px; padding:4px 10px; border-radius:6px;
          background:rgba(17,19,23,.82); color:var(--muted); font-size:12px; pointer-events:none; }
  #focus { position:absolute; right:24px; bottom:22px; padding:4px 10px; border-radius:6px;
           background:rgba(17,19,23,.82); color:var(--muted); font-size:12px; pointer-events:none; }
  aside { display:grid; gap:12px; }
  .card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:12px; }
  .card h2 { margin:0 0 10px; font-size:11px; font-weight:600; letter-spacing:.08em;
             text-transform:uppercase; color:var(--muted); }
  .stats { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
  .stat { background:var(--card2); border-radius:8px; padding:8px 10px; }
  .stat span { display:block; font-size:11px; color:var(--muted); }
  .stat b { font-size:20px; font-weight:600; font-variant-numeric:tabular-nums; }
  .stat small { color:var(--muted); font-size:12px; margin-left:2px; }
  .chips { display:flex; gap:6px; flex-wrap:wrap; margin-top:8px; }
  .chip { font-size:12px; padding:2px 8px; border-radius:999px; background:var(--card2); color:var(--muted); }
  .score { display:flex; gap:8px; margin-top:8px; }
  .score div { flex:1; text-align:center; border-radius:8px; padding:6px; font-weight:700;
               font-size:18px; font-variant-numeric:tabular-nums; }
  .score .r { background:color-mix(in srgb, var(--red) 25%, transparent); color:#ffb3b5; }
  .score .b { background:color-mix(in srgb, var(--blue) 25%, transparent); color:#b3d0ff; }
  #comp { display:grid; grid-template-columns:repeat(4, 1fr); background:var(--card2);
          border-radius:8px; padding:3px; gap:3px; }
  #comp button { border:0; background:none; color:var(--muted); padding:6px 0; font-size:12px;
                 border-radius:6px; text-transform:capitalize; }
  #comp button.on { color:#0b0c0e; font-weight:600; }
  #comp button.on[data-mode=NONE] { background:var(--fg); }
  #comp button.on[data-mode=DRIVER] { background:var(--driver); }
  #comp button.on[data-mode=AUTONOMOUS] { background:var(--auton); }
  #comp button.on[data-mode=DISABLED] { background:var(--off); color:var(--fg); }
  #screen { margin:0 0 8px; padding:8px 10px; background:#0b1a0e; color:#9ef0a8;
            border:1px solid #1f3b25; border-radius:6px; font:13px/1.35 ui-monospace, monospace;
            white-space:pre; overflow:hidden; }
  #gamepad { font-size:12px; color:var(--muted); }
  .keys { display:grid; grid-template-columns:auto 1fr; gap:4px 10px; font-size:12px;
          color:var(--muted); margin-top:8px; }
  kbd { font:11px ui-monospace, monospace; color:var(--fg); background:var(--card2);
        border:1px solid var(--line); border-bottom-width:2px; border-radius:4px; padding:0 5px; }
  details summary { cursor:pointer; color:var(--muted); font-size:12px; margin-top:8px; }
  label.toggle { display:flex; align-items:center; gap:8px; cursor:pointer; }
</style></head><body>
<header>
  <h1>V5 Simulator</h1>
  <nav><a class="on" href="./">Field</a><a href="dashboard">Devices</a><a href="controller">Controller</a>
       <a href="field">Field control</a><a href="screen">Brain screen</a></nav>
  <span class="spacer"></span>
  <button id="tapbtn" title="Tap the brain's button: Back / Home">Brain button</button>
</header>
<main>
  <section class="fieldwrap">
    <canvas id="c" width="720" height="720" tabindex="0" aria-label="Field, audience view"></canvas>
    <div id="info">waiting for the sim...</div>
    <div id="focus">click the field to drive</div>
  </section>
  <aside>
    <div class="card">
      <h2>Robot</h2>
      <div class="stats">
        <div class="stat"><span>X</span><b id="sx">-</b><small>in</small></div>
        <div class="stat"><span>Y</span><b id="sy">-</b><small>in</small></div>
        <div class="stat"><span>Heading</span><b id="sh">-</b><small>°</small></div>
        <div class="stat"><span>Speed</span><b id="sv">-</b><small>m/s</small></div>
      </div>
      <div class="chips"><span class="chip" id="st">t 0.0 s</span><span class="chip" id="sheld" hidden></span></div>
      <div class="score" id="score" hidden><div class="r" id="scr">0</div><div class="b" id="scb">0</div></div>
    </div>
    <div class="card">
      <h2>Competition</h2>
      <div id="comp"></div>
    </div>
    <div class="card" id="pad">
      <h2>Controller</h2>
      <pre id="screen" hidden></pre>
      <div id="gamepad">no gamepad</div>
      <details><summary>Keyboard and gamepad keys</summary>
        <div class="keys">
          <span><kbd>W</kbd><kbd>S</kbd></span><span>left stick up / down (Axis3)</span>
          <span><kbd>A</kbd><kbd>D</kbd></span><span>right stick left / right (Axis1)</span>
          <span><kbd>↑</kbd><kbd>↓</kbd></span><span>right stick up / down (Axis2)</span>
          <span><kbd>←</kbd><kbd>→</kbd></span><span>left stick left / right (Axis4)</span>
          <span><kbd>J</kbd><kbd>K</kbd><kbd>U</kbd><kbd>I</kbd></span><span>A B X Y</span>
          <span><kbd>Q</kbd><kbd>E</kbd><kbd>Z</kbd><kbd>C</kbd></span><span>L1 R1 L2 R2</span>
          <span><kbd>1</kbd>–<kbd>4</kbd></span><span>optical gesture up / down / left / right</span>
          <span><kbd>T</kbd></span><span>hold the Light Tower button</span>
        </div>
        <p style="font-size:12px;color:var(--muted);margin:8px 0 0">A gamepad (Xbox or any
        standard one): its sticks are the V5 sticks; A B X Y, LB RB = L1 R1, LT RT = L2 R2,
        d-pad = arrows. Press a button on it with this page in front.</p>
      </details>
    </div>
    <div class="card">
      <h2>Display</h2>
      <label class="toggle"><input type="checkbox" id="override"> V5RC Override field layout</label>
    </div>
  </aside>
</main>
<script>
// Audience view, in the field frame VEX/RW-Template use: red wall left,
// audience at the bottom, +Y up. The sim's frame is turned: sim +x = RW +Y,
// sim +y = RW -X.
const IN = 0.0254, HALF_IN = 70.25;
const FIELD = 2 * HALF_IN * IN, TILES = 6, ROBOT = 0.4572, BLOCK = 0.045;
const c = document.getElementById('c'), g = c.getContext('2d');
const info = document.getElementById('info');
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let PAD = 28, K = 1;                       // margin and line scale, in canvas pixels
const px = m => m / FIELD * (c.width - 2 * PAD);
function rwCanvas(xin, yin) { return [c.width / 2 + px(xin * IN), c.height / 2 - px(yin * IN)]; }
function toCanvas(x, y) { return rwCanvas(-y / IN, x / IN); }
function fit() {                           // sharp on high-DPI screens
  const w = Math.round(c.clientWidth * (window.devicePixelRatio || 1));
  if (w > 0 && c.width !== w) { c.width = c.height = w; }
  K = c.width / 720; PAD = 28 * K;
}
addEventListener('resize', fit);
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
function robotShape(color, outline) {
  const s = px(ROBOT), r = s * 0.12;
  g.fillStyle = color; g.globalAlpha = 0.9;
  g.beginPath(); g.roundRect(-s / 2, -s / 2, s, s, r); g.fill();
  g.globalAlpha = 1; g.lineWidth = 2 * K; g.strokeStyle = outline; g.stroke();
  g.fillStyle = outline;                   // heading: an arrow to the front
  g.beginPath(); g.moveTo(s * 0.42, 0); g.lineTo(s * 0.08, -s * 0.2); g.lineTo(s * 0.08, s * 0.2);
  g.closePath(); g.fill();
}

function draw(s) {
  fit();
  g.fillStyle = css('--card'); g.fillRect(0, 0, c.width, c.height);
  const [fx, fy] = rwCanvas(-HALF_IN, HALF_IN), side = px(FIELD), t = side / TILES;
  for (let i = 0; i < TILES; i++) for (let j = 0; j < TILES; j++) {
    g.fillStyle = css((i + j) % 2 ? '--tile2' : '--tile');
    g.fillRect(fx + i * t, fy + j * t, t, t);
  }
  g.strokeStyle = css('--seam'); g.lineWidth = 1.5 * K;
  for (let i = 1; i < TILES; i++) {
    g.beginPath(); g.moveTo(fx + i * t, fy); g.lineTo(fx + i * t, fy + side); g.stroke();
    g.beginPath(); g.moveTo(fx, fy + i * t); g.lineTo(fx + side, fy + i * t); g.stroke();
  }
  if (overrideBox.checked) drawOverride(fx, fy, side);
  // the perimeter, red alliance wall on the left and blue on the right
  const w = 6 * K;
  g.lineWidth = w; g.strokeStyle = css('--dim'); g.strokeRect(fx - w / 2, fy - w / 2, side + w, side + w);
  g.strokeStyle = css('--red'); g.beginPath(); g.moveTo(fx - w / 2, fy); g.lineTo(fx - w / 2, fy + side); g.stroke();
  g.strokeStyle = css('--blue'); g.beginPath(); g.moveTo(fx + side + w / 2, fy); g.lineTo(fx + side + w / 2, fy + side); g.stroke();
  g.fillStyle = css('--muted'); g.font = `${11 * K}px system-ui`; g.textAlign = 'center';
  g.fillText('AUDIENCE', c.width / 2, fy + side + 23 * K);
  g.textAlign = 'left';
  // the trail, fading from old to new
  const tr = s.trail || [];
  g.lineWidth = 2.5 * K; g.lineCap = 'round'; g.strokeStyle = css('--trail');
  for (let i = 1; i < tr.length; i++) {
    const [a, b] = toCanvas(...tr[i - 1]), [d, e] = toCanvas(...tr[i]);
    g.globalAlpha = 0.15 + 0.75 * i / tr.length;
    g.beginPath(); g.moveTo(a, b); g.lineTo(d, e); g.stroke();
  }
  g.globalAlpha = 1;
  g.font = `${12 * K}px system-ui`;
  for (const [name, [x0, y0, x1, y1]] of s.goals || []) {
    const [ax, ay] = toCanvas(x0, y0), [bx, by] = toCanvas(x1, y1);
    const [lx, ty] = [Math.min(ax, bx), Math.min(ay, by)], gw = Math.abs(bx - ax), gh = Math.abs(by - ay);
    g.fillStyle = 'rgba(242,184,75,.08)'; g.fillRect(lx, ty, gw, gh);
    g.strokeStyle = css('--trail'); g.lineWidth = 1.5 * K; g.setLineDash([6 * K, 4 * K]);
    g.strokeRect(lx, ty, gw, gh); g.setLineDash([]);
    g.fillStyle = css('--fg'); g.fillText(name, lx + 4 * K, ty + 14 * K);
  }
  g.fillStyle = css('--dim');
  for (const [kind, x, y, sx, sy, r, ang] of s.obstacles || []) {
    const [ox, oy] = toCanvas(x, y);
    g.save(); g.translate(ox, oy); g.rotate(-ang); g.globalAlpha = 0.9;
    g.beginPath();
    if (kind === 'box') g.rect(-px(sy * 2) / 2, -px(sx * 2) / 2, px(sy * 2), px(sx * 2));
    else g.arc(0, 0, px(r), 0, 2 * Math.PI);
    g.fill(); g.restore();
  }
  for (const [x, y, h] of s.opponents || []) {
    const [ox, oy] = toCanvas(x, y);
    g.save(); g.translate(ox, oy); g.rotate(-h - Math.PI / 2);
    robotShape(css('--opponent'), css('--fg')); g.restore();
  }
  for (const [x, y, color] of s.blocks || []) {
    const [bx, by] = toCanvas(x, y);
    g.fillStyle = css(color === 'red' ? '--red' : '--blue');
    g.beginPath(); g.arc(bx, by, px(BLOCK), 0, 2 * Math.PI); g.fill();
    g.lineWidth = 1.5 * K; g.strokeStyle = 'rgba(0,0,0,.45)'; g.stroke();
  }
  const [rx, ry] = toCanvas(s.x, s.y);
  g.save(); g.translate(rx, ry); g.rotate(-s.heading - Math.PI / 2);   // sim +x is up
  g.shadowColor = 'rgba(0,0,0,.5)'; g.shadowBlur = 10 * K;
  robotShape(css('--robot'), '#ffffff');
  g.restore();
  // RW-Template / mikLib coordinates: inches, heading clockwise from +Y
  const rwHeading = ((-s.heading * 180 / Math.PI) % 360 + 360) % 360;
  document.getElementById('sx').textContent = (-s.y / IN).toFixed(1);
  document.getElementById('sy').textContent = (s.x / IN).toFixed(1);
  document.getElementById('sh').textContent = rwHeading.toFixed(1);
  document.getElementById('sv').textContent = s.v.toFixed(2);
  document.getElementById('st').textContent = `t ${s.t.toFixed(1)} s`;
  const heldEl = document.getElementById('sheld');
  heldEl.hidden = !(s.held && s.held.length);
  if (!heldEl.hidden) heldEl.textContent = `holding ${s.held.join(' ')}`;
  document.getElementById('score').hidden = !s.score;
  if (s.score) {
    document.getElementById('scr').textContent = `Red ${s.score.red}`;
    document.getElementById('scb').textContent = `Blue ${s.score.blue}`;
  }
  info.hidden = true;
  const scr = document.getElementById('screen');
  scr.hidden = !(s.screen && s.screen.length);
  if (!scr.hidden) scr.textContent = s.screen.map(l => l.padEnd(19)).join('\\n');
}
async function poll() {
  try { draw(await (await fetch('state.json')).json()); }
  catch (e) { info.hidden = false; info.textContent = 'sim not reachable'; }
  setTimeout(poll, 50);
}
poll();
// keys reach the page once it has focus (a click anywhere on it)
const focusHint = document.getElementById('focus');
const showHint = () => { focusHint.hidden = document.hasFocus(); };
addEventListener('focus', showHint); addEventListener('blur', showHint); showHint();
c.addEventListener('mousedown', () => c.focus());

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
    padInfo.textContent = `Gamepad: ${gp.id}` +
      (gp.mapping === 'standard' ? '' : ' (non-standard mapping: buttons may be off)') +
      ` · ${live || 'centred'}` +
      (competition === 'DISABLED' ? ' · DISABLED: the robot ignores input' : '');
  } else {
    padInfo.textContent = 'Keyboard: click the field, then drive. Gamepad: press A on it with this page in front.' +
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
for (const [mode, label] of [['NONE', 'None'], ['DRIVER', 'Driver'], ['AUTONOMOUS', 'Auton'], ['DISABLED', 'Disabled']]) {
  const b = document.createElement('button');
  b.textContent = label; b.dataset.mode = mode;
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
                elif self.path in ("/controller", "/controller.html"):
                    body, ctype = controllerpage.PAGE.encode(), "text/html; charset=utf-8"
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
                opponents: list[tuple[float, float, float]] = (),
                controller: dict | None = None) -> None:
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
                           "opponents": [[round(v, 4) for v in o] for o in opponents],
                           "controller": controller}

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
