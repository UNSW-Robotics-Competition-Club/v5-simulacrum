"""The Controller page (served by fieldview.FieldView at /controller).

A V5 controller drawn as an SVG (original artwork laid out like the real one:
screen top centre, sticks at the top corners, arrow buttons lower left,
X Y A B lower right, shoulder buttons on the top edge). Overlaid on it, live:
its screen, a monochrome LCD like the real one (a status strip, then the
three rows of 19 characters the program prints, e.g. an autonomous
selector), its rumble, and its sticks and buttons from the controller state
(state.json "controller", whatever drives it: this page, the Field page's
keyboard, a gamepad). Its buttons can be clicked, or pressed with the keys
listed on the page, to work menus on the controller screen. It posts
{"buttons": [...]} alone to /input: the sticks and the competition mode set
elsewhere are kept (robot.parse_controller_input).
"""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>V5 Controller</title>
<style>
  :root { --bg:#111317; --card:#1a1d23; --card2:#22262d; --line:#2f343d; --fg:#e8eaed;
          --muted:#9aa0a6; --dim:#6b7079; --accent:#4ea1ff;
          --auton:#f2a93b; --driver:#3cc46c; --off:#6b7079;
          /* the controller */
          --body:#5d6064; --body-edge:#4a4d51; --plate:#b8babd; --plate-edge:#a3a6a9;
          --key:#9c9fa3; --key-edge:#86898d; --glyph:#e0565b; --well:#45484c; --cap:#55585c;
          --lcd:#c9cdc2; --lcd-ink:#1d201b; --lcd-dim:#9fa498; }
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
  main { max-width:1180px; margin:0 auto; padding:16px; display:grid;
         grid-template-columns:minmax(0, 1fr) 280px; gap:16px; align-items:start; }
  @media (max-width: 860px) { main { grid-template-columns:1fr; } }
  .card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:12px; }
  .card h2 { margin:0 0 10px; font-size:11px; font-weight:600; letter-spacing:.08em;
             text-transform:uppercase; color:var(--muted); }
  aside { display:grid; gap:12px; }

  #pad { display:block; width:100%; max-width:820px; margin:0 auto; user-select:none;
         -webkit-user-select:none; touch-action:none; }
  #pad.buzz { animation:buzz .07s linear infinite alternate; }
  @keyframes buzz { from { transform:translateX(-1.5px); } to { transform:translateX(1.5px); } }
  [data-btn] { cursor:pointer; }
  [data-btn] .face { fill:var(--key); stroke:var(--key-edge); stroke-width:1.5; transition:fill .05s; }
  [data-btn]:hover .face { stroke:var(--accent); stroke-width:2.5; }
  [data-btn] .glyph { fill:var(--glyph); }
  [data-btn].on .face { fill:var(--accent); stroke:var(--accent); }
  [data-btn].on .glyph { fill:#fff; }
  .shoulder .face { fill:#4f5256; stroke:#3f4245; }
  .shoulder .glyph { fill:#d7d9db; }
  .lcdtext { font:600 11.2px ui-monospace, Consolas, "Courier New", monospace; fill:var(--lcd-ink);
             white-space:pre; }
  .lcdstatus { font:700 9px system-ui, sans-serif; fill:var(--lcd-ink); }
  .stickcap { transition:transform .05s linear; }
  .stickcap.moved circle.top { fill:var(--accent); }
  /* narrow panes (a VS Code side pane): the drawn screen gets small, so the
     same LCD is repeated large under the controller */
  #bigscreen { display:none; margin:10px auto 0; max-width:420px; background:var(--lcd);
               color:var(--lcd-ink); border:6px solid #2c2f33; border-radius:8px; padding:8px 10px;
               font:600 clamp(13px, 4.6vw, 20px)/1.5 ui-monospace, Consolas, monospace;
               white-space:pre; overflow:hidden; }
  @media (max-width: 640px) { #bigscreen { display:block; } }
  #rumble { text-align:center; margin-top:6px; min-height:20px; font:13px ui-monospace, monospace;
            color:var(--muted); }
  #rumble.on { color:var(--auton); }

  .kv { display:flex; justify-content:space-between; gap:8px; padding:8px 10px;
        background:var(--card2); border-radius:8px; margin-bottom:6px; }
  .kv span:first-child { color:var(--muted); }
  .chip { font-size:12px; padding:2px 8px; border-radius:999px; background:var(--off); color:#0b0c0e; font-weight:600; }
  .keys { display:grid; grid-template-columns:auto 1fr; gap:4px 10px; font-size:12px; color:var(--muted); }
  kbd { font:11px ui-monospace, monospace; color:var(--fg); background:var(--card2);
        border:1px solid var(--line); border-bottom-width:2px; border-radius:4px; padding:0 5px; }
  .note { font-size:12px; color:var(--muted); margin:8px 0 0; }
</style></head><body>
<header>
  <h1>V5 Simulator</h1>
  <nav><a href="./">Field</a><a href="dashboard">Devices</a>
       <a class="on" href="controller">Controller</a><a href="field">Field control</a>
       <a href="screen">Brain screen</a></nav>
</header>
<main>
  <section class="card">
    <svg id="pad" viewBox="-10 -26 580 440" role="img" aria-label="V5 controller">
      <!-- shoulder buttons on the top edge (hidden from the front on the real one) -->
      <g class="shoulder" data-btn="L2"><rect class="face" x="22" y="-22" width="58" height="34" rx="9"/>
        <text class="glyph" x="51" y="-1" text-anchor="middle" font-size="13" font-weight="700">L2</text></g>
      <g class="shoulder" data-btn="L1"><rect class="face" x="86" y="-18" width="64" height="34" rx="9"/>
        <text class="glyph" x="118" y="3" text-anchor="middle" font-size="13" font-weight="700">L1</text></g>
      <g class="shoulder" data-btn="R1"><rect class="face" x="410" y="-18" width="64" height="34" rx="9"/>
        <text class="glyph" x="442" y="3" text-anchor="middle" font-size="13" font-weight="700">R1</text></g>
      <g class="shoulder" data-btn="R2"><rect class="face" x="480" y="-22" width="58" height="34" rx="9"/>
        <text class="glyph" x="509" y="-1" text-anchor="middle" font-size="13" font-weight="700">R2</text></g>

      <!-- body and faceplate -->
      <path fill="var(--body)" stroke="var(--body-edge)" stroke-width="2"
            d="M95 20 C160 4 400 4 465 20 C520 32 552 75 558 140 C565 230 560 330 545 375
               C535 405 470 405 450 370 C425 320 400 292 360 292 L200 292
               C160 292 135 320 110 370 C90 405 25 405 15 375 C0 330 -5 230 2 140
               C8 75 40 32 95 20 Z"/>
      <path fill="var(--plate)" stroke="var(--plate-edge)" stroke-width="1.5"
            d="M112 38 C180 22 380 22 448 38 C472 46 472 72 456 96 C442 120 446 140 472 152
               C507 168 506 222 470 246 C430 272 130 272 90 246 C54 222 53 168 88 152
               C114 140 118 120 104 96 C88 72 88 46 112 38 Z"/>

      <!-- sticks: wells, then caps that follow the input -->
      <circle cx="73" cy="126" r="48" fill="var(--well)"/>
      <circle cx="487" cy="126" r="48" fill="var(--well)"/>
      <g class="stickcap" id="lstick" transform="translate(73 126)">
        <circle r="35" fill="var(--cap)" stroke="#3b3e42" stroke-width="2"/>
        <circle class="top" r="26" fill="#606368"/></g>
      <g class="stickcap" id="rstick" transform="translate(487 126)">
        <circle r="35" fill="var(--cap)" stroke="#3b3e42" stroke-width="2"/>
        <circle class="top" r="26" fill="#606368"/></g>

      <!-- the screen: a monochrome LCD, status strip then 3 rows x 19 -->
      <rect x="194" y="36" width="172" height="104" rx="5" fill="#2c2f33"/>
      <g id="lcd">
        <rect x="200" y="42" width="160" height="92" rx="2" fill="var(--lcd)"/>
        <rect x="200" y="42" width="160" height="17" fill="var(--lcd-dim)"/>
        <text class="lcdstatus" x="205" y="54" id="lcdtitle">V5 Controller</text>
        <g id="lcdbatt" transform="translate(330 46)">
          <rect width="18" height="9" rx="1.5" fill="none" stroke="var(--lcd-ink)" stroke-width="1.3"/>
          <rect x="18.5" y="3" width="2" height="3" fill="var(--lcd-ink)"/>
          <rect x="2" y="2" width="14" height="5" fill="var(--lcd-ink)"/></g>
        <g id="lcdlink" transform="translate(312 46)">
          <rect x="0" y="6" width="3" height="3" fill="var(--lcd-ink)"/>
          <rect x="4.5" y="3.5" width="3" height="5.5" fill="var(--lcd-ink)"/>
          <rect x="9" y="1" width="3" height="8" fill="var(--lcd-ink)"/></g>
        <text class="lcdtext" id="lcd0" x="204" y="80"></text>
        <text class="lcdtext" id="lcd1" x="204" y="103"></text>
        <text class="lcdtext" id="lcd2" x="204" y="126"></text>
      </g>

      <!-- arrow buttons -->
      <g data-btn="Up"><circle class="face" cx="157" cy="163" r="19"/>
        <path class="glyph" d="M157 155 L165 168 L149 168 Z"/></g>
      <g data-btn="Left"><circle class="face" cx="120" cy="203" r="19"/>
        <path class="glyph" d="M112 203 L125 195 L125 211 Z"/></g>
      <g data-btn="Right"><circle class="face" cx="194" cy="203" r="19"/>
        <path class="glyph" d="M202 203 L189 195 L189 211 Z"/></g>
      <g data-btn="Down"><circle class="face" cx="157" cy="241" r="19"/>
        <path class="glyph" d="M157 249 L165 236 L149 236 Z"/></g>

      <!-- X Y A B -->
      <g data-btn="X"><circle class="face" cx="400" cy="163" r="19"/>
        <text class="glyph" x="400" y="169" text-anchor="middle" font-size="17" font-weight="700">X</text></g>
      <g data-btn="Y"><circle class="face" cx="362" cy="203" r="19"/>
        <text class="glyph" x="362" y="209" text-anchor="middle" font-size="17" font-weight="700">Y</text></g>
      <g data-btn="A"><circle class="face" cx="438" cy="203" r="19"/>
        <text class="glyph" x="438" y="209" text-anchor="middle" font-size="17" font-weight="700">A</text></g>
      <g data-btn="B"><circle class="face" cx="400" cy="241" r="19"/>
        <text class="glyph" x="400" y="247" text-anchor="middle" font-size="17" font-weight="700">B</text></g>

      <!-- power button (not an input the program sees) -->
      <rect x="258" y="196" width="44" height="44" rx="8" fill="#55585c" stroke="#45484c" stroke-width="1.5"/>
      <!-- power symbol: a circle open at the top (centre 280,220, r 10, gap 35 degrees
           each side of vertical) and a line down through the gap -->
      <path d="M285.74 211.81 A10 10 0 1 1 274.26 211.81" fill="none" stroke="var(--glyph)"
            stroke-width="3" stroke-linecap="round"/>
      <path d="M280 206 L280 219" fill="none" stroke="var(--glyph)" stroke-width="3" stroke-linecap="round"/>
    </svg>
    <div id="bigscreen" aria-hidden="true"></div>
    <div id="rumble">no rumble</div>
  </section>
  <aside>
    <div class="card">
      <h2>Status</h2>
      <div class="kv"><span>Controller</span><span id="link">-</span></div>
      <div class="kv"><span>Competition</span><span class="chip" id="comp">-</span></div>
      <div class="kv"><span>Sim time</span><span id="t">waiting for the sim...</span></div>
    </div>
    <div class="card">
      <h2>Buttons</h2>
      <div class="keys">
        <span><kbd>&uarr;</kbd><kbd>&darr;</kbd><kbd>&larr;</kbd><kbd>&rarr;</kbd></span><span>arrow buttons</span>
        <span><kbd>J</kbd><kbd>K</kbd><kbd>U</kbd><kbd>I</kbd></span><span>A B X Y</span>
        <span><kbd>Q</kbd><kbd>E</kbd><kbd>Z</kbd><kbd>C</kbd></span><span>L1 R1 L2 R2</span>
      </div>
      <p class="note">Click a button or press its key with this page in front. Sticks and
      the competition mode come from the Field page or a gamepad.</p>
    </div>
  </aside>
</main>
<script>
const KEYS = { ArrowUp: 'Up', ArrowDown: 'Down', ArrowLeft: 'Left', ArrowRight: 'Right',
  j: 'A', k: 'B', u: 'X', i: 'Y', q: 'L1', e: 'R1', z: 'L2', c: 'R2' };
const COMP = { NONE: 'var(--fg)', DRIVER: 'var(--driver)', AUTONOMOUS: 'var(--auton)', DISABLED: 'var(--off)' };
const COLS = 19;                      // characters a row shows
const TRAVEL = 13;                    // stick cap travel at full deflection (SVG units)
const held = new Set();               // pressed here (mouse or keys)
const buttons = document.querySelectorAll('[data-btn]');
const $ = id => document.getElementById(id);

function send() {
  fetch('input', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                   body: JSON.stringify({ buttons: [...held] }) }).catch(() => {});
}
function press(name, down) {
  if (held.has(name) === down) return;
  down ? held.add(name) : held.delete(name);
  send();
}
buttons.forEach(b => {
  const name = b.dataset.btn;
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); press(name, true); });
  const up = () => press(name, false);
  b.addEventListener('pointerup', up);
  b.addEventListener('pointercancel', up);
  b.addEventListener('lostpointercapture', up);
});
function key(e, down) {
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (!(k in KEYS)) return;
  e.preventDefault();
  if (e.repeat) return;
  press(KEYS[k], down);
}
addEventListener('keydown', e => key(e, true));
addEventListener('keyup', e => key(e, false));
addEventListener('blur', () => { if (held.size) { held.clear(); send(); } });

function stick(el, cx, cy, x, y) {
  const dx = (x || 0) / 127 * TRAVEL, dy = -(y || 0) / 127 * TRAVEL;
  el.setAttribute('transform', `translate(${cx + dx} ${cy + dy})`);
  el.classList.toggle('moved', !!(x || y));
}
function draw(s) {
  $('t').textContent = `${s.t.toFixed(1)} s`;
  const c = s.controller, connected = !!(c && c.connected);
  const rows = connected ? (s.screen || []).concat(['', '', '']).slice(0, 3)
                         : ['', '  no controller', ''];
  const shown = rows.map(l => l.slice(0, COLS).padEnd(COLS));
  shown.forEach((l, i) => { $('lcd' + i).textContent = l; });
  $('bigscreen').textContent = shown.join('\n');
  $('lcdbatt').style.visibility = $('lcdlink').style.visibility = connected ? 'visible' : 'hidden';
  $('link').textContent = c ? (connected ? 'connected' : 'none in this robot') : '-';
  $('comp').textContent = c ? c.competition : '-';
  $('comp').style.background = c ? COMP[c.competition] || 'var(--off)' : 'var(--off)';
  const rumbling = !!(c && c.rumble);
  $('rumble').textContent = rumbling ? `rumble  ${c.rumble}` : 'no rumble';
  $('rumble').classList.toggle('on', rumbling);
  $('pad').classList.toggle('buzz', rumbling);
  const lit = new Set([...(c ? c.buttons : []), ...held]);
  buttons.forEach(b => b.classList.toggle('on', lit.has(b.dataset.btn)));
  const a = c ? c.axes : {};
  stick($('lstick'), 73, 126, a.Axis4, a.Axis3);
  stick($('rstick'), 487, 126, a.Axis1, a.Axis2);
}
async function poll() {
  try { draw(await (await fetch('state.json')).json()); }
  catch (e) { $('t').textContent = 'sim not reachable'; }
  setTimeout(poll, 100);
}
poll();
</script></body></html>
"""
