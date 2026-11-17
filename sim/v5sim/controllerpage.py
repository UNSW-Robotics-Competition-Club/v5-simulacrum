"""The Controller page (served by fieldview.FieldView at /controller).

A V5 controller: its three-line screen (what the program prints there, e.g.
an autonomous selector), its rumble pattern, and its sticks and buttons lit
from the live controller state (state.json "controller", whatever drives it:
this page, the Field page's keyboard, a gamepad). Its buttons can be clicked,
or pressed with the keys listed on the page, to work menus on the controller
screen. It posts {"buttons": [...]} alone to /input: the sticks and the
competition mode set elsewhere are kept (robot.parse_controller_input).
"""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>V5 Controller</title>
<style>
  :root { --bg:#111317; --card:#1a1d23; --card2:#22262d; --line:#2f343d; --fg:#e8eaed;
          --muted:#9aa0a6; --dim:#6b7079; --accent:#4ea1ff;
          --lcd:#0b1a0e; --lcdfg:#9ef0a8; --lcdline:#1f3b25; --shell:#2a2e35;
          --auton:#f2a93b; --driver:#3cc46c; --off:#6b7079; }
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
  main { max-width:1080px; margin:0 auto; padding:16px; display:grid;
         grid-template-columns:minmax(0, 1fr) 300px; gap:16px; align-items:start; }
  @media (max-width: 860px) { main { grid-template-columns:1fr; } }
  .card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:12px; }
  .card h2 { margin:0 0 10px; font-size:11px; font-weight:600; letter-spacing:.08em;
             text-transform:uppercase; color:var(--muted); }
  aside { display:grid; gap:12px; }

  /* the controller */
  .pad { background:var(--shell); border:1px solid var(--line); border-radius:28px;
         padding:18px 22px 26px; max-width:720px; margin:0 auto; user-select:none; }
  .shoulders { display:flex; justify-content:space-between; margin-bottom:14px; }
  .shoulders div { display:flex; gap:6px; }
  .face { display:grid; grid-template-columns:1fr minmax(0, 2.2fr) 1fr; gap:18px; align-items:center; }
  .side { display:grid; gap:18px; justify-items:center; }
  /* narrow panes (a VS Code side pane): the screen gets its own row on top */
  @media (max-width: 640px) {
    .pad { padding:14px 12px 18px; }
    .face { grid-template-columns:1fr 1fr; }
    .mid { grid-column:1 / -1; order:-1; }
  }
  .lcdwrap { position:relative; }
  #lcd { margin:0; padding:12px 14px; background:var(--lcd); color:var(--lcdfg);
         border:2px solid var(--lcdline); border-radius:8px; white-space:pre; overflow:hidden;
         font:clamp(13px, 2.3vw, 22px)/1.45 ui-monospace, Consolas, monospace; min-height:5.2em; }
  #nolink { position:absolute; inset:0; display:flex; align-items:center; justify-content:center;
            color:var(--muted); font-size:13px; background:rgba(11,26,14,.85); border-radius:8px; }
  #rumble { display:flex; align-items:center; justify-content:center; gap:6px; margin-top:10px;
            min-height:24px; font:13px ui-monospace, monospace; color:var(--muted); }
  #rumble.on { color:var(--auton); animation:buzz .15s linear infinite alternate; }
  @keyframes buzz { from { transform:translateX(-1px); } to { transform:translateX(1px); } }
  .stick { width:86px; height:86px; border-radius:50%; background:var(--card2);
           border:1px solid var(--line); position:relative; }
  .stick i { position:absolute; left:50%; top:50%; width:34px; height:34px; margin:-17px 0 0 -17px;
             border-radius:50%; background:#3a404a; border:1px solid #4a515c;
             transition:transform .05s linear; }
  .stick.moved i { background:var(--accent); border-color:var(--accent); }
  .cluster { display:grid; grid-template-columns:repeat(3, 38px); grid-template-rows:repeat(3, 38px); gap:3px; }
  .cluster button:nth-child(1) { grid-area:1 / 2; }
  .cluster button:nth-child(2) { grid-area:2 / 1; }
  .cluster button:nth-child(3) { grid-area:2 / 3; }
  .cluster button:nth-child(4) { grid-area:3 / 2; }
  button.k { font:600 13px system-ui, sans-serif; color:var(--fg); background:var(--card2);
             border:1px solid var(--line); border-radius:8px; cursor:pointer; padding:0;
             touch-action:none; }
  .shoulders button.k { width:58px; height:30px; border-radius:10px; }
  .face button.k { width:38px; height:38px; }
  .abxy button.k { border-radius:50%; }
  button.k:hover { border-color:var(--accent); }
  button.k.on { background:var(--accent); border-color:var(--accent); color:#0b0c0e; }

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
    <div class="pad">
      <div class="shoulders">
        <div><button class="k" data-btn="L2">L2</button><button class="k" data-btn="L1">L1</button></div>
        <div><button class="k" data-btn="R1">R1</button><button class="k" data-btn="R2">R2</button></div>
      </div>
      <div class="face">
        <div class="side">
          <div class="stick" id="lstick" title="left stick: Axis4 / Axis3"><i></i></div>
          <div class="cluster" aria-label="arrow buttons">
            <button class="k" data-btn="Up" aria-label="Up">&#9650;</button>
            <button class="k" data-btn="Left" aria-label="Left">&#9664;</button>
            <button class="k" data-btn="Right" aria-label="Right">&#9654;</button>
            <button class="k" data-btn="Down" aria-label="Down">&#9660;</button>
          </div>
        </div>
        <div class="mid">
          <div class="lcdwrap"><pre id="lcd" aria-label="controller screen"></pre>
            <div id="nolink" hidden>no controller</div></div>
          <div id="rumble">no rumble</div>
        </div>
        <div class="side">
          <div class="stick" id="rstick" title="right stick: Axis1 / Axis2"><i></i></div>
          <div class="cluster abxy" aria-label="A B X Y">
            <button class="k" data-btn="X">X</button>
            <button class="k" data-btn="Y">Y</button>
            <button class="k" data-btn="A">A</button>
            <button class="k" data-btn="B">B</button>
          </div>
        </div>
      </div>
    </div>
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
const held = new Set();               // pressed here (mouse or keys)
const buttons = document.querySelectorAll('button[data-btn]');

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
  b.addEventListener('pointerdown', e => { b.setPointerCapture(e.pointerId); press(name, true); });
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

function stick(el, x, y) {
  const r = 26;                        // px of travel for full deflection
  el.querySelector('i').style.transform = `translate(${(x || 0) / 127 * r}px, ${-(y || 0) / 127 * r}px)`;
  el.classList.toggle('moved', !!(x || y));
}
function draw(s) {
  document.getElementById('t').textContent = `${s.t.toFixed(1)} s`;
  const c = s.controller;
  const lines = (s.screen || []).concat(['', '', '']).slice(0, 3);
  document.getElementById('lcd').textContent = lines.map(l => l.padEnd(19)).join('\n');
  document.getElementById('nolink').hidden = !!(c && c.connected);
  document.getElementById('link').textContent = c ? (c.connected ? 'connected' : 'none in this robot') : '-';
  const comp = document.getElementById('comp');
  comp.textContent = c ? c.competition : '-';
  comp.style.background = c ? COMP[c.competition] || 'var(--off)' : 'var(--off)';
  const rumble = document.getElementById('rumble');
  rumble.textContent = c && c.rumble ? `rumble  ${c.rumble}` : 'no rumble';
  rumble.classList.toggle('on', !!(c && c.rumble));
  const lit = new Set([...(c ? c.buttons : []), ...held]);
  buttons.forEach(b => b.classList.toggle('on', lit.has(b.dataset.btn)));
  const a = c ? c.axes : {};
  stick(document.getElementById('lstick'), a.Axis4, a.Axis3);
  stick(document.getElementById('rstick'), a.Axis1, a.Axis2);
}
async function poll() {
  try { draw(await (await fetch('state.json')).json()); }
  catch (e) { document.getElementById('t').textContent = 'sim not reachable'; }
  setTimeout(poll, 100);
}
poll();
</script></body></html>
"""
