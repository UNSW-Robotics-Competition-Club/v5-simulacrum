"""The field-control page (served by fieldview.FieldView at /field).

A browser stand-in for the field-control app a V5 brain runs as an event's
field controller: a brain-screen-sized panel with Match / Driver Skills /
Auton Skills / Manual tabs, the countdown, the phase, Start and Stop, and
the robot's station (radio link, program, battery). The sim keeps the
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
  :root { --bg:#0f1114; --bezel:#22252b; --screen:#000; --bar:#1d2026; --line:#353a43;
          --fg:#f1f3f4; --muted:#9aa0a6; --red:#d6403a; --blue:#2f7fd8;
          --auton:#f2a93b; --driver:#3cc46c; --off:#6b7079; }
  * { box-sizing:border-box; }
  [hidden] { display:none !important; }
  body { margin:0; min-height:100vh; background:var(--bg); color:var(--fg);
         font:14px system-ui, sans-serif; display:flex; flex-direction:column;
         align-items:center; padding:16px; gap:12px; }
  .bezel { width:min(960px, 100%); background:var(--bezel); border-radius:18px; padding:14px; }
  .screen { aspect-ratio:480/272; background:var(--screen); border-radius:4px;
            display:grid; grid-template-rows:auto 1fr; overflow:hidden;
            font-size:clamp(9px, 2.1vw, 20px); }
  .bar { display:flex; align-items:center; gap:.4em; background:var(--bar);
         padding:.35em .6em; border-bottom:1px solid var(--line); }
  .bar h1 { font-size:1em; margin:0 auto 0 0; letter-spacing:.04em; }
  .tab { background:none; color:var(--muted); border:1px solid transparent; border-radius:.3em;
         font:inherit; font-size:.8em; padding:.25em .6em; cursor:pointer; }
  .tab.on { color:var(--fg); border-color:var(--line); background:#2a2e35; }
  .body { display:grid; grid-template-columns:1.6fr 1fr; gap:.6em; padding:.6em; }
  .clock { display:flex; flex-direction:column; align-items:center; justify-content:center; gap:.3em; }
  #timer { font-size:4.2em; font-weight:700; font-variant-numeric:tabular-nums; line-height:1; }
  #phase { font-size:.95em; font-weight:700; letter-spacing:.08em; padding:.2em .8em;
           border-radius:1em; background:var(--off); color:#000; }
  #period { color:var(--muted); font-size:.75em; }
  .row { display:flex; gap:.5em; margin-top:.4em; }
  .big { font:inherit; font-weight:700; font-size:.9em; letter-spacing:.05em; border:0;
         border-radius:.35em; padding:.55em 1.2em; cursor:pointer; color:#000; }
  .big:disabled { opacity:.35; cursor:default; }
  #start { background:var(--driver); } #stop { background:var(--red); color:#fff; }
  .mode { background:#2a2e35; color:var(--fg); border:1px solid var(--line); }
  .mode.on { outline:2px solid var(--fg); }
  .station { border:2px solid var(--red); border-radius:.4em; padding:.5em; align-self:start;
             display:grid; gap:.35em; font-size:.8em; }
  .station h2 { margin:0; font-size:1em; color:var(--red); }
  .dot { display:inline-block; width:.7em; height:.7em; border-radius:50%;
         background:var(--off); margin-right:.4em; vertical-align:middle; }
  .dot.ok { background:var(--driver); }
  .muted { color:var(--muted); }
  footer { color:var(--muted); max-width:960px; line-height:1.5; }
  footer a { color:var(--blue); }
</style></head><body>
<div class="bezel"><div class="screen">
  <div class="bar">
    <h1>FIELD CONTROL</h1>
    <button class="tab on" data-tab="match">Match</button>
    <button class="tab" data-tab="skills_driver">Driver Skills</button>
    <button class="tab" data-tab="skills_auton">Auton Skills</button>
    <button class="tab" data-tab="manual">Manual</button>
  </div>
  <div class="body">
    <div class="clock">
      <div id="timer">0:15</div>
      <div id="phase">DISABLED</div>
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
    <div class="station">
      <h2>RED 1 &middot; ROBOT</h2>
      <div><span class="dot" id="linkdot"></span><span id="link">no radio</span></div>
      <div>Program: <span id="prog" class="muted">-</span></div>
      <div>Battery: <span id="batt" class="muted">-</span></div>
      <div class="muted" id="field">-</div>
    </div>
  </div>
</div></div>
<footer>
  The sim runs the clock: a match is 0:15 autonomous, a 2 s pause, then 1:45 driver
  control; skills runs are 1:00. Manual switches the robot yourself.
  <a href="./">field view</a> &middot; <a href="dashboard">device dashboard</a>
</footer>
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

function remaining(f) {
  const m = f.match;
  if (!m) { const [a, d] = RUNS[tab] || [0, 0]; return [a || d, a ? 'autonomous' : 'driver control']; }
  const t = f.t, autonEnd = m.start + m.auton, driverStart = autonEnd + m.pause;
  if (t < autonEnd) return [autonEnd - t, 'autonomous'];
  if (t < driverStart) return [m.driver, 'pause'];
  if (t < m.end) return [m.end - t, 'driver control'];
  return [0, 'ended'];
}

function draw(f) {
  last = f;
  const [secs, period] = remaining(f);
  document.getElementById('timer').textContent = clock(secs);
  const phase = document.getElementById('phase');
  phase.textContent = f.competition === 'NONE' ? 'NO COMPETITION' : f.competition;
  phase.style.background = COLORS[f.competition] || 'var(--off)';
  document.getElementById('period').textContent =
    f.match ? `${f.kind.replace('_', ' ')}: ${period}` : 'ready';
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
