"""The device dashboard page (served by fieldview.FieldView at /dashboard).

Polls /devices.json (Robot.device_snapshot: every motor's measured state
and command as the brain sees it, every sensor's reading) and draws live
charts of each motor's velocity against its target and its position, with
the electrical numbers beside them. The brain-button controls post
{"brain_button": SECONDS} to /input: a tap is Back/Home, about 1 s held
stops a running program, about 3 s with no program powers the brain off,
which closes the sim (the dashboard holds it 3.5 s, just past that threshold).
"""

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>V5 Devices</title>
<style>
  :root { --bg:#111317; --card:#1a1d23; --card2:#22262d; --line:#2f343d; --fg:#e8eaed;
          --muted:#9aa0a6; --dim:#6b7079; --accent:#4ea1ff;
          --vel:#4ea1ff; --tgt:#f2b84b; --pos:#57d38c; --warn:#e5484d; --ok:#3cc46c; }
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
  button:disabled { opacity:.4; cursor:default; }
  .num, #t { font-variant-numeric:tabular-nums; }
  .chip { font-size:12px; padding:2px 8px; border-radius:999px; background:var(--card2); color:var(--muted); }
  .btngroup { display:flex; align-items:center; gap:6px; font-size:12px; color:var(--muted); }
  #msg { color:var(--muted); font-size:12px; }
  main { max-width:1280px; margin:0 auto; padding:16px; display:grid; gap:16px; }
  .card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:12px; }
  h2 { margin:0 0 10px; font-size:11px; font-weight:600; letter-spacing:.08em;
       text-transform:uppercase; color:var(--muted); }
  .grid { display:grid; grid-template-columns:repeat(auto-fill, minmax(330px, 1fr)); gap:12px; }
  .head { display:flex; justify-content:space-between; align-items:center; gap:8px; margin-bottom:8px; }
  .name { font-weight:600; display:flex; align-items:center; gap:8px; }
  .badge { font-size:11px; padding:1px 8px; border-radius:999px; background:var(--card2); color:var(--muted); }
  .badge.DATA { background:color-mix(in srgb, var(--ok) 22%, transparent); color:#9ef0a8; }
  .badge.INFO, .badge.ANNOUNCE { background:color-mix(in srgb, var(--tgt) 22%, transparent); color:#f2c97a; }
  .stats { display:grid; grid-template-columns:repeat(3, 1fr); gap:6px; margin-bottom:8px; }
  .stats span { background:var(--card2); border-radius:8px; padding:5px 8px; font-size:11px;
                color:var(--muted); min-height:1.4em; }
  .stats span:empty, .stats span:has(b:empty) { visibility:hidden; }
  .stats b { display:block; color:var(--fg); font-size:13px; font-weight:600; }
  .warn { color:var(--warn) !important; }
  canvas { width:100%; height:90px; display:block; background:var(--bg); border-radius:6px; }
  .legend { font-size:11px; color:var(--muted); margin:4px 0 8px; }
  .legend i { display:inline-block; width:10px; height:3px; margin:0 4px 2px 8px; vertical-align:middle; }
  .legend i:first-child { margin-left:0; }
  table { width:100%; border-collapse:collapse; }
  td, th { text-align:left; padding:8px 10px; border-bottom:1px solid var(--line); vertical-align:top; }
  tr:last-child td { border-bottom:0; }
  th { color:var(--muted); font-weight:500; font-size:12px; }
  td.reading { font-family:ui-monospace, monospace; font-size:12px; word-break:break-word; }
  #motors:empty::before { content:"no motors"; color:var(--muted); }
</style></head><body>
<header>
  <h1>V5 Simulator</h1>
  <nav><a href="./">Field</a><a class="on" href="dashboard">Devices</a><a href="controller">Controller</a>
       <a href="field">Field control</a><a href="screen">Brain screen</a></nav>
  <span class="spacer"></span>
  <span id="t" class="chip num">waiting for the sim...</span>
  <span class="btngroup">Brain button
    <button data-secs="0.2" title="Back / Home">Tap</button>
    <button data-secs="1.5" title="stops a running program">Hold 1.5 s</button>
    <button data-secs="3.5" title="with no program running: the brain powers off and the sim closes">Hold 3.5 s</button>
  </span>
  <span id="msg"></span>
</header>
<main>
  <section><h2>Motors</h2><div id="motors" class="grid"></div></section>
  <section class="card"><h2>Sensors and other devices</h2>
    <table><thead><tr><th>Port</th><th>Device</th><th>Link</th><th>Reading</th></tr></thead>
    <tbody id="sensors"></tbody></table></section>
</main>
<script>
const HISTORY_S = 15, POLL_MS = 100;
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n);
const history = {};                       // port -> [{t, rpm, tgt, pos}]
const cards = {};
const fmt = (v, d = 1) => (v === null || v === undefined) ? '-' : Number(v).toFixed(d);

for (const b of document.querySelectorAll('button[data-secs]')) {
  b.onclick = async () => {
    const secs = Number(b.dataset.secs);
    const msg = document.getElementById('msg');
    try {
      const r = await fetch('input', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                       body: JSON.stringify({ brain_button: secs }) });
      msg.textContent = r.ok ? `pressed for ${secs} s` : `button: ${r.status} ${await r.text()}`;
    } catch (e) { msg.textContent = 'sim not reachable'; }
  };
}

function card(m) {
  const el = document.createElement('div');
  el.className = 'card';
  el.innerHTML = `<div class="head"><span class="name">Port ${m.port} <span class="badge role"></span></span>
    <span class="badge link"></span></div>
    <div class="stats"></div>
    <canvas class="vel" width="600" height="160"></canvas>
    <div class="legend"><i style="background:var(--vel)"></i>velocity rpm<i style="background:var(--tgt)"></i>target</div>
    <canvas class="pos" width="600" height="160"></canvas>
    <div class="legend"><i style="background:var(--pos)"></i>position deg<i style="background:var(--tgt)"></i>target</div>`;
  document.getElementById('motors').appendChild(el);
  return el;
}

function chart(canvas, pts, key, tkey, color, show) {
  const g = canvas.getContext('2d'), W = canvas.width, H = canvas.height;
  g.clearRect(0, 0, W, H);
  if (pts.length < 2) return;
  const vals = pts.flatMap(p => show(p) ? [p[key], p[tkey]] : [p[key]]);
  let lo = Math.min(...vals), hi = Math.max(...vals);
  if (hi - lo < 1) { hi += 0.5; lo -= 0.5; }
  const pad = (hi - lo) * 0.1; lo -= pad; hi += pad;
  const t1 = pts[pts.length - 1].t, t0 = t1 - HISTORY_S;
  const X = t => (t - t0) / HISTORY_S * W, Y = v => H - (v - lo) / (hi - lo) * H;
  g.strokeStyle = css('--line'); g.lineWidth = 1;
  if (lo < 0 && hi > 0) { g.beginPath(); g.moveTo(0, Y(0)); g.lineTo(W, Y(0)); g.stroke(); }
  g.fillStyle = css('--muted'); g.font = '20px system-ui';
  g.fillText(hi.toFixed(0), 4, 20); g.fillText(lo.toFixed(0), 4, H - 6);
  const line = (k, c, dash, when) => {
    g.strokeStyle = c; g.lineWidth = 2.5; g.setLineDash(dash); g.beginPath();
    let on = false;
    for (const p of pts) {
      if (!when(p)) { on = false; continue; }
      on ? g.lineTo(X(p.t), Y(p[k])) : g.moveTo(X(p.t), Y(p[k])); on = true;
    }
    g.stroke(); g.setLineDash([]);
  };
  line(tkey, css('--tgt'), [8, 6], show);
  line(key, color, [], () => true);
}

function stat(label, value, warn) {
  return `<span>${label}<b class="num${warn ? ' warn' : ''}">${value}</b></span>`;
}

function drawMotor(m, t) {
  const el = cards[m.port] || (cards[m.port] = card(m));
  el.querySelector('.role').textContent = `${m.role}, ${m.cartridge}`;
  const link = el.querySelector('.link');
  link.textContent = m.link; link.className = 'badge link ' + m.link;
  const cmd = m.mode === 'velocity' ? `${fmt(m.target_rpm)} rpm`
            : m.mode === 'position' || m.mode === 'profile' ? `${fmt(m.target_deg)}&deg;`
            : m.mode === 'voltage' ? `${m.percent}%` : '-';
  el.querySelector('.stats').innerHTML =
    stat('pos', `${fmt(m.position_deg)}&deg;`) + stat('vel', `${fmt(m.velocity_rpm)} rpm`) +
    stat('mode', `${m.mode} ${cmd}`) +
    stat('volts', fmt(m.voltage_v, 2)) + stat('amps', fmt(m.current_a, 2), m.limited) +
    stat('power', `${fmt(m.power_w)} W`) +
    stat('temp', `${fmt(m.temperature_c)} &deg;C`, m.temperature_c > 55) +
    stat('brake', m.brake) + stat('', m.limited ? 'current limited' : '', m.limited);
  const h = history[m.port] || (history[m.port] = []);
  h.push({ t, rpm: m.velocity_rpm, tgt: m.target_rpm, pos: m.position_deg, tdeg: m.target_deg, mode: m.mode });
  while (h.length && h[0].t < t - HISTORY_S) h.shift();
  chart(el.querySelector('.vel'), h, 'rpm', 'tgt', css('--vel'), p => p.mode === 'velocity');
  chart(el.querySelector('.pos'), h, 'pos', 'tdeg', css('--pos'),
        p => p.mode === 'position' || p.mode === 'profile');
}

function drawSensors(list) {
  document.getElementById('sensors').innerHTML = list.map(d =>
    `<tr><td class="num">${d.port}</td><td>${d.kind}</td>` +
    `<td><span class="badge ${d.link}">${d.link}</span></td>` +
    `<td class="reading">${d.text.replace(/[&<>]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;'}[c]))}</td></tr>`
  ).join('') || '<tr><td colspan="4">none</td></tr>';
}

let lastT = -1;
async function poll() {
  try {
    const s = await (await fetch('devices.json')).json();
    if (s.t < lastT) { for (const k in history) history[k].length = 0; }   // sim restarted
    lastT = s.t;
    document.getElementById('t').textContent = `t = ${s.t.toFixed(1)} s`;
    for (const m of s.devices.filter(d => d.kind === 'motor')) drawMotor(m, s.t);
    drawSensors(s.devices.filter(d => d.kind !== 'motor'));
    const want = new Set(s.devices.filter(d => d.kind === 'motor').map(d => String(d.port)));
    for (const p of Object.keys(cards)) if (!want.has(p)) { cards[p].remove(); delete cards[p]; }
  } catch (e) {
    document.getElementById('t').textContent = 'sim not reachable';
  }
  setTimeout(poll, POLL_MS);
}
poll();
</script></body></html>
"""
