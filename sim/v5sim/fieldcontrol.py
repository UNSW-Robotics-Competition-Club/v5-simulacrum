"""The field-control page (served by fieldview.FieldView at /field).

A browser stand-in for the field-control app a V5 brain runs as an event's
field controller: Match / Driver Skills / Auton Skills / Manual tabs, the
countdown with the phase and its progress, Start and Stop, and the robot's
station (radio link, program, battery). The sim keeps the
clock: Start posts {"field": "match" | "skills_driver" | "skills_auton"}
to /input and the robot sim runs that schedule (robot.FIELD_RUNS), so the
brain's banner timer and the radio's field byte stay right even when the
tab is in the background. Stop posts {"field": "stop"}; the Manual tab's
buttons post {"field": "stop", "competition": ...}. It polls /field.json
(Robot.field_snapshot). The layout follows the real app's; the drawing is
this project's own.
"""

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>V5 Field Control</title>
<style>
  :root { --bg:#111317; --card:#1a1d23; --card2:#22262d; --line:#2f343d; --fg:#f1f3f4;
          --muted:#9aa0a6; --dim:#6b7079; --accent:#4ea1ff;
          --red:#e5484d; --blue:#3e8ef7; --auton:#f2a93b; --driver:#3cc46c; --off:#6b7079; }
  * { box-sizing:border-box; }
  [hidden] { display:none !important; }
  body { margin:0; min-height:100vh; background:var(--bg); color:var(--fg);
         font:14px/1.4 system-ui, sans-serif; }
  header { display:flex; align-items:center; gap:12px; flex-wrap:wrap; padding:10px 16px;
           border-bottom:1px solid var(--line); background:var(--card); }
  header h1 { font-size:15px; margin:0 8px 0 0; letter-spacing:.02em; }
  nav { display:flex; gap:4px; flex-wrap:wrap; }
  nav a { color:var(--muted); text-decoration:none; padding:4px 10px; border-radius:999px; }
  nav a:hover { color:var(--fg); background:var(--card2); }
  nav a.on { color:var(--fg); background:var(--card2); box-shadow:inset 0 0 0 1px var(--line); }
  main { max-width:980px; margin:0 auto; padding:20px 16px; display:grid; gap:16px; }
  .tabs { display:grid; grid-template-columns:repeat(4, 1fr); background:var(--card);
          border:1px solid var(--line); border-radius:10px; padding:4px; gap:4px; }
  .tab { font:inherit; border:0; background:none; color:var(--muted); padding:8px 0;
         border-radius:7px; cursor:pointer; }
  .tab:hover { color:var(--fg); }
  .tab.on { background:var(--card2); color:var(--fg); box-shadow:inset 0 0 0 1px var(--line); font-weight:600; }
  .panel { display:grid; grid-template-columns:minmax(0, 1.7fr) minmax(240px, 1fr); gap:16px; }
  @media (max-width: 760px) { .panel { grid-template-columns:1fr; } }
  .card { background:var(--card); border:1px solid var(--line); border-radius:12px; padding:16px; }
  .card h2 { margin:0 0 12px; font-size:11px; font-weight:600; letter-spacing:.08em;
             text-transform:uppercase; color:var(--muted); }
  .clock { display:flex; flex-direction:column; align-items:center; gap:12px; padding:28px 16px; }
  #phase { font-size:13px; font-weight:700; letter-spacing:.12em; padding:4px 14px;
           border-radius:999px; background:var(--off); color:#0b0c0e; }
  #timer { font-size:clamp(64px, 14vw, 128px); font-weight:700; line-height:1;
           font-variant-numeric:tabular-nums; letter-spacing:-.02em; }
  #period { color:var(--muted); text-transform:capitalize; }
  .track { width:100%; max-width:520px; height:8px; border-radius:999px; background:var(--card2); overflow:hidden; }
  #progress { height:100%; width:0; background:var(--off); border-radius:999px; transition:width .2s linear; }
  .row { display:flex; gap:10px; flex-wrap:wrap; justify-content:center; margin-top:6px; }
  .big { font:inherit; font-weight:700; letter-spacing:.06em; border:0; border-radius:10px;
         padding:12px 28px; cursor:pointer; color:#0b0c0e; min-width:130px; }
  .big:disabled { opacity:.3; cursor:default; }
  #start { background:var(--driver); } #stop { background:var(--red); color:#fff; }
  .mode { background:var(--card2); color:var(--fg); box-shadow:inset 0 0 0 1px var(--line); }
  .mode.on[data-mode=DISABLED] { background:var(--off); }
  .mode.on[data-mode=AUTONOMOUS] { background:var(--auton); color:#0b0c0e; }
  .mode.on[data-mode=DRIVER] { background:var(--driver); color:#0b0c0e; }
  .station { border-top:3px solid var(--red); display:grid; gap:10px; align-content:start; }
  .station h2 { color:#ffb3b5; }
  .kv { display:flex; justify-content:space-between; gap:8px; padding:8px 10px;
        background:var(--card2); border-radius:8px; }
  .kv span:first-child { color:var(--muted); }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%;
         background:var(--off); margin-right:6px; vertical-align:1px; }
  .dot.ok { background:var(--driver); box-shadow:0 0 0 3px color-mix(in srgb, var(--driver) 25%, transparent); }
  .muted { color:var(--muted); }
  footer { color:var(--muted); font-size:13px; line-height:1.5; }
</style></head><body>
<header>
  <h1>V5 Simulator</h1>
  <nav><a href="./">Field</a><a href="dashboard">Devices</a><a href="controller">Controller</a>
       <a class="on" href="field">Field control</a><a href="screen">Brain screen</a></nav>
</header>
<main>
  <div class="tabs">
    <button class="tab on" data-tab="match">Match</button>
    <button class="tab" data-tab="skills_driver">Driver Skills</button>
    <button class="tab" data-tab="skills_auton">Auton Skills</button>
    <button class="tab" data-tab="manual">Manual</button>
  </div>
  <div class="panel">
    <div class="card clock">
      <div id="phase">DISABLED</div>
      <div id="timer">0:15</div>
      <div class="track"><div id="progress"></div></div>
      <div id="period"></div>
      <div class="row" id="runbtns">
        <button class="big" id="start">START</button>
        <button class="big" id="stop">STOP</button>
      </div>
      <div class="row" id="modebtns" hidden>
        <button class="big mode" data-mode="DISABLED">DISABLE</button>
        <button class="big mode" data-mode="AUTONOMOUS">AUTON</button>
        <button class="big mode" data-mode="DRIVER">DRIVER</button>
      </div>
    </div>
    <div class="card station">
      <h2>Red 1 · Robot</h2>
      <div class="kv"><span>Radio</span><span><span class="dot" id="linkdot"></span><span id="link">no radio</span></span></div>
      <div class="kv"><span>Program</span><span id="prog" class="muted">-</span></div>
      <div class="kv"><span>Battery</span><span id="batt" class="muted">-</span></div>
      <div class="kv"><span>Control</span><span class="muted" id="field">-</span></div>
    </div>
  </div>
  <footer>
    The sim runs the clock: a match is 0:15 autonomous, a 2 s pause, then 1:45 driver
    control; skills runs are 1:00. Manual switches the robot yourself.
  </footer>
</main>
<script>
const RUNS = { match: [15, 105], skills_driver: [0, 60], skills_auton: [60, 0] };
const COLORS = { DISABLED: 'var(--off)', AUTONOMOUS: 'var(--auton)',
                 DRIVER: 'var(--driver)', NONE: 'var(--off)' };
let tab = 'match', last = null;

async function post(msg) {
  await fetch('input', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                         body: JSON.stringify(msg) });
}
const clock = s => { s = Math.max(0, Math.ceil(s));
  return Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0'); };

// [seconds left, period name, length of that period]
function remaining(f) {
  const m = f.match;
  if (!m) { const [a, d] = RUNS[tab] || [0, 0]; return [a || d, a ? 'autonomous' : 'driver control', a || d]; }
  const t = f.t, autonEnd = m.start + m.auton, driverStart = autonEnd + m.pause;
  if (t < autonEnd) return [autonEnd - t, 'autonomous', m.auton];
  if (t < driverStart) return [m.driver, 'pause', m.driver];
  if (t < m.end) return [m.end - t, 'driver control', m.driver];
  return [0, 'ended', 1];
}

function draw(f) {
  last = f;
  const [secs, period, total] = remaining(f);
  document.getElementById('timer').textContent = clock(secs);
  const color = COLORS[f.competition] || 'var(--off)';
  const phase = document.getElementById('phase');
  phase.textContent = f.competition === 'NONE' ? 'NO COMPETITION' : f.competition;
  phase.style.background = color;
  const bar = document.getElementById('progress');
  bar.style.width = (f.match && total ? 100 * (1 - secs / total) : 0) + '%';
  bar.style.background = color;
  document.getElementById('period').textContent =
    f.match ? `${f.kind.replace('_', ' ')} · ${period}` : 'ready';
  document.getElementById('start').disabled = !!(f.match && f.t < f.match.end);
  document.getElementById('stop').disabled = !f.match;
  document.querySelectorAll('.mode').forEach(b =>
    b.classList.toggle('on', !f.match && b.dataset.mode === f.competition));
  const r = f.radio;
  document.getElementById('linkdot').classList.toggle('ok', !!(r && r.linked));
  document.getElementById('link').textContent = !r ? 'no radio' : r.linked ? 'linked' : 'searching';
  document.getElementById('prog').textContent = !r ? '-' : r.running ? 'running' : 'stopped';
  document.getElementById('batt').textContent = r && r.battery != null ? r.battery + ' %' : '-';
  document.getElementById('field').textContent = f.field_control ? 'field control' : 'competition switch';
}

document.querySelectorAll('.tab').forEach(b => b.onclick = () => {
  tab = b.dataset.tab;
  document.querySelectorAll('.tab').forEach(x => x.classList.toggle('on', x === b));
  document.getElementById('runbtns').hidden = tab === 'manual';
  document.getElementById('modebtns').hidden = tab !== 'manual';
  if (last) draw(last);
});
document.getElementById('start').onclick = () => post({ field: tab });
document.getElementById('stop').onclick = () => post({ field: 'stop' });
document.querySelectorAll('.mode').forEach(b =>
  b.onclick = () => post({ field: 'stop', competition: b.dataset.mode }));

async function poll() {
  try { draw(await (await fetch('field.json')).json()); }
  catch (e) { document.getElementById('period').textContent = 'waiting for the sim...'; }
  setTimeout(poll, 200);
}
poll();
</script></body></html>
"""
