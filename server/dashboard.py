"""Debug dashboard: shared state, broadcast loop, and inline HTML dashboard."""

import asyncio
import json
import time

import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse


class DashboardState:
    """Singleton holding the latest telemetry snapshot for dashboard clients."""

    def __init__(self):
        self.twist = [0.0] * 6
        self.q = [0.0, -1.5707963, 1.5707963, -1.5707963, -1.5707963, 0.0]  # Q_HOME
        self.qdot = [0.0] * 6
        self.ee_pos = [0.0, 0.0, 0.0]
        self.ee_dist = 0.0
        self.mu = 0.0
        self.lam = 0.0
        self.feedback = {
            "manipulability": 0.0,
            "joint_limit_proximity": [0.0] * 6,
            "workspace_proximity": 0.0,
            "is_feasible": True,
        }
        self.dt = 0.0
        self.n_connections = 0
        self._msg_count = 0
        self._rate_window_start = time.time()
        self.msgs_per_sec = 0.0
        self.timestamp = time.time()

    def update_msg_rate(self):
        self._msg_count += 1
        now = time.time()
        elapsed = now - self._rate_window_start
        if elapsed >= 1.0:
            self.msgs_per_sec = self._msg_count / elapsed
            self._msg_count = 0
            self._rate_window_start = now
        self.timestamp = now

    def to_dict(self) -> dict:
        return {
            "twist": self.twist,
            "q": self.q,
            "qdot": self.qdot,
            "ee_pos": self.ee_pos,
            "ee_dist": self.ee_dist,
            "mu": self.mu,
            "lam": self.lam,
            "feedback": self.feedback,
            "dt": self.dt,
            "n_connections": self.n_connections,
            "msgs_per_sec": round(self.msgs_per_sec, 1),
            "timestamp": self.timestamp,
        }


dashboard_state = DashboardState()
dashboard_clients: set[WebSocket] = set()


def get_static_config() -> dict:
    """UR10e constants for the config panel (previously from kinematics.py)."""
    import math
    pi = math.pi
    return {
        "dh_a": [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0],
        "dh_d": [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
        "dh_alpha": [pi / 2, 0.0, 0.0, pi / 2, -pi / 2, 0.0],
        "q_min": [-2 * pi] * 6,
        "q_max": [2 * pi] * 6,
        "qdot_max": [pi, pi, pi, 2 * pi, 2 * pi, 2 * pi],
        "q_home": [0.0, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0.0],
        "max_reach": 1.1843,
        "mu_threshold": 0.005,
        "twist_scale": 1.0,
        "max_dt": 0.1,
        "min_dt": 0.001,
        "damping_mu_threshold": 0.01,
        "damping_lambda_min": 0.001,
        "damping_lambda_max": 0.1,
    }


async def broadcast_loop():
    """Push state to all dashboard clients at ~10 Hz."""
    while True:
        if dashboard_clients:
            payload = json.dumps({"type": "state", **dashboard_state.to_dict()})
            dead = set()
            for ws in dashboard_clients:
                try:
                    await ws.send_text(payload)
                except Exception:
                    dead.add(ws)
            dashboard_clients.difference_update(dead)
        await asyncio.sleep(0.1)


def register_dashboard_routes(app: FastAPI):
    """Add dashboard endpoints to the FastAPI app."""

    @app.on_event("startup")
    async def _start_broadcast():
        asyncio.create_task(broadcast_loop())

    @app.get("/dashboard")
    async def dashboard_page():
        return HTMLResponse(DASHBOARD_HTML)

    @app.websocket("/ws/dashboard")
    async def dashboard_ws(websocket: WebSocket):
        await websocket.accept()
        # Send static config once
        try:
            config = get_static_config()
            await websocket.send_text(json.dumps({"type": "config", **config}))
        except Exception:
            return
        dashboard_clients.add(websocket)
        try:
            while True:
                await websocket.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            dashboard_clients.discard(websocket)


# ---------------------------------------------------------------------------
# Inline HTML dashboard
# ---------------------------------------------------------------------------

DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>UR10e Debug Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  :root {
    --bg: #1a1a2e; --card: #16213e; --accent: #0f3460;
    --text: #e0e0e0; --muted: #888; --green: #4caf50;
    --yellow: #ff9800; --red: #f44336; --blue: #2196f3;
  }
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: 'SF Mono', 'Fira Code', monospace;
    background: var(--bg); color: var(--text); font-size: 13px;
    padding: 12px;
  }
  h1 { font-size: 18px; margin-bottom: 12px; display: flex; align-items: center; gap: 10px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; }
  .dot.on { background: var(--green); box-shadow: 0 0 6px var(--green); }
  .dot.off { background: var(--red); }
  .grid {
    display: grid; grid-template-columns: 1fr 1fr; gap: 12px;
  }
  @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  .card {
    background: var(--card); border-radius: 8px; padding: 12px;
    border: 1px solid var(--accent);
  }
  .card h2 { font-size: 13px; color: var(--muted); margin-bottom: 8px; text-transform: uppercase; letter-spacing: 1px; }
  table { width: 100%; border-collapse: collapse; }
  th, td { padding: 3px 8px; text-align: right; }
  th { color: var(--muted); font-weight: normal; text-align: left; }
  td { font-variant-numeric: tabular-nums; }
  .val { color: var(--blue); }
  .mu-bar {
    height: 6px; border-radius: 3px; margin-top: 4px;
    background: #333; overflow: hidden;
  }
  .mu-bar-fill { height: 100%; border-radius: 3px; transition: width 0.1s; }
  canvas { width: 100% !important; height: 180px !important; }
  details { margin-top: 12px; }
  summary {
    cursor: pointer; color: var(--muted); font-size: 13px;
    padding: 8px; background: var(--card); border-radius: 8px;
    border: 1px solid var(--accent);
  }
  .config-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 12px; }
  .status-row { display: flex; gap: 16px; align-items: center; margin-bottom: 8px; flex-wrap: wrap; }
  .stat { display: flex; align-items: center; gap: 4px; }
  .stat-label { color: var(--muted); }
</style>
</head>
<body>

<h1>
  <span>UR10e Debug Dashboard</span>
  <span class="dot off" id="ws-dot"></span>
  <span id="ws-status" style="color:var(--muted);font-size:12px;">disconnected</span>
</h1>

<div class="status-row" id="status-bar">
  <div class="stat"><span class="stat-label">iOS Connections:</span> <span class="val" id="n-conn">0</span></div>
  <div class="stat"><span class="stat-label">Msgs/s:</span> <span class="val" id="msg-rate">0</span></div>
  <div class="stat"><span class="stat-label">dt:</span> <span class="val" id="dt-val">0</span> <span class="stat-label">ms</span></div>
</div>

<div class="grid">
  <!-- LEFT COLUMN -->
  <div>
    <div class="card" style="margin-bottom:12px;">
      <h2>Incoming Twist</h2>
      <table>
        <tr><th></th><th>Linear (m/s)</th><th>Angular (rad/s)</th></tr>
        <tr><th>X</th><td class="val" id="vx">0</td><td class="val" id="wx">0</td></tr>
        <tr><th>Y</th><td class="val" id="vy">0</td><td class="val" id="wy">0</td></tr>
        <tr><th>Z</th><td class="val" id="vz">0</td><td class="val" id="wz">0</td></tr>
      </table>
    </div>

    <div class="card" style="margin-bottom:12px;">
      <h2>Joint State</h2>
      <table>
        <tr><th>Joint</th><th>q (rad)</th><th>q (deg)</th><th>qdot (rad/s)</th></tr>
        <tr id="j0"><th>1</th><td>0</td><td>0</td><td>0</td></tr>
        <tr id="j1"><th>2</th><td>0</td><td>0</td><td>0</td></tr>
        <tr id="j2"><th>3</th><td>0</td><td>0</td><td>0</td></tr>
        <tr id="j3"><th>4</th><td>0</td><td>0</td><td>0</td></tr>
        <tr id="j4"><th>5</th><td>0</td><td>0</td><td>0</td></tr>
        <tr id="j5"><th>6</th><td>0</td><td>0</td><td>0</td></tr>
      </table>
    </div>

    <div class="card" style="margin-bottom:12px;">
      <h2>End Effector</h2>
      <table>
        <tr><th>X</th><td class="val" id="ee-x">0</td><th style="padding-left:16px">Distance</th><td class="val" id="ee-dist">0</td></tr>
        <tr><th>Y</th><td class="val" id="ee-y">0</td><th></th><td></td></tr>
        <tr><th>Z</th><td class="val" id="ee-z">0</td><th></th><td></td></tr>
      </table>
    </div>

    <div class="card" style="margin-bottom:12px;">
      <h2>Manipulability & Damping</h2>
      <table>
        <tr><th>mu</th><td class="val" id="mu-val">0</td><th>lambda</th><td class="val" id="lam-val">0</td></tr>
        <tr><th>Feasible</th><td id="feasible-val">--</td><th>WS Prox</th><td class="val" id="ws-prox">0</td></tr>
      </table>
      <div class="mu-bar"><div class="mu-bar-fill" id="mu-bar" style="width:0%;background:var(--green);"></div></div>
    </div>

    <div class="card">
      <h2>Joint Limit Proximity</h2>
      <table>
        <tr><th>Joint</th><th>1</th><th>2</th><th>3</th><th>4</th><th>5</th><th>6</th></tr>
        <tr id="jl-row"><th>Prox</th><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td></tr>
      </table>
    </div>
  </div>

  <!-- RIGHT COLUMN: Charts -->
  <div>
    <div class="card" style="margin-bottom:12px;">
      <h2>Joint Angles</h2>
      <canvas id="chart-joints"></canvas>
    </div>
    <div class="card" style="margin-bottom:12px;">
      <h2>Manipulability</h2>
      <canvas id="chart-mu"></canvas>
    </div>
    <div class="card" style="margin-bottom:12px;">
      <h2>Twist Magnitude</h2>
      <canvas id="chart-twist"></canvas>
    </div>
    <div class="card">
      <h2>Joint Limit Proximity</h2>
      <canvas id="chart-jl"></canvas>
    </div>
  </div>
</div>

<!-- Config Constants (collapsible) -->
<details id="config-section">
  <summary>Configuration Constants</summary>
  <div class="config-grid">
    <div class="card">
      <h2>DH Parameters</h2>
      <table id="dh-table">
        <tr><th>Joint</th><th>a (m)</th><th>d (m)</th><th>alpha (rad)</th></tr>
      </table>
    </div>
    <div class="card">
      <h2>Joint Limits & Velocities</h2>
      <table id="limits-table">
        <tr><th>Joint</th><th>q_min</th><th>q_max</th><th>qdot_max</th><th>q_home</th></tr>
      </table>
    </div>
    <div class="card">
      <h2>Server Constants</h2>
      <table id="server-table"></table>
    </div>
    <div class="card">
      <h2>Damping Parameters</h2>
      <table id="damping-table"></table>
    </div>
  </div>
</details>

<script>
const MAX_POINTS = 200;
const JOINT_COLORS = ['#e6194b','#3cb44b','#4363d8','#f58231','#911eb4','#42d4f4'];
const f = (v, d=4) => Number(v).toFixed(d);
const rad2deg = r => (r * 180 / Math.PI);

// --- Chart.js setup ---
const chartOpts = (yLabel) => ({
  animation: false,
  responsive: true,
  maintainAspectRatio: false,
  scales: {
    x: { display: false },
    y: { ticks: { color: '#888', font: { size: 10 } }, grid: { color: '#333' }, title: { display: true, text: yLabel, color: '#888', font: { size: 10 } } }
  },
  plugins: { legend: { display: true, labels: { color: '#ccc', font: { size: 10 }, boxWidth: 12 } } },
  elements: { point: { radius: 0 }, line: { borderWidth: 1.5 } }
});

function makeDatasets(labels, colors) {
  return labels.map((l, i) => ({ label: l, data: [], borderColor: colors[i], fill: false }));
}

const jointLabels = ['J1','J2','J3','J4','J5','J6'];

const jointChart = new Chart(document.getElementById('chart-joints'), {
  type: 'line', data: { labels: [], datasets: makeDatasets(jointLabels, JOINT_COLORS) },
  options: chartOpts('rad')
});

const muChart = new Chart(document.getElementById('chart-mu'), {
  type: 'line',
  data: { labels: [], datasets: [{ label: 'mu', data: [], borderColor: '#4caf50', fill: false }] },
  options: {
    ...chartOpts('mu'),
    plugins: {
      ...chartOpts('mu').plugins,
      annotation: undefined
    }
  }
});

const twistChart = new Chart(document.getElementById('chart-twist'), {
  type: 'line',
  data: { labels: [], datasets: [
    { label: 'Linear', data: [], borderColor: '#2196f3', fill: false },
    { label: 'Angular', data: [], borderColor: '#ff9800', fill: false }
  ]},
  options: chartOpts('magnitude')
});

const jlChart = new Chart(document.getElementById('chart-jl'), {
  type: 'line', data: { labels: [], datasets: makeDatasets(jointLabels, JOINT_COLORS) },
  options: chartOpts('proximity')
});

const charts = [jointChart, muChart, twistChart, jlChart];
let timeIdx = 0;

function pushChartData(chart, label, values) {
  chart.data.labels.push(label);
  values.forEach((v, i) => chart.data.datasets[i].data.push(v));
  if (chart.data.labels.length > MAX_POINTS) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(ds => ds.data.shift());
  }
}

// --- WebSocket ---
let ws;
function connect() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  ws = new WebSocket(`${proto}//${location.host}/ws/dashboard`);

  ws.onopen = () => {
    document.getElementById('ws-dot').className = 'dot on';
    document.getElementById('ws-status').textContent = 'connected';
  };
  ws.onclose = () => {
    document.getElementById('ws-dot').className = 'dot off';
    document.getElementById('ws-status').textContent = 'reconnecting...';
    setTimeout(connect, 2000);
  };
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === 'config') populateConfig(msg);
    if (msg.type === 'state') updateState(msg);
  };
}
connect();

// --- Config population ---
function populateConfig(cfg) {
  // DH table
  const dh = document.getElementById('dh-table');
  for (let i = 0; i < 6; i++) {
    const tr = document.createElement('tr');
    tr.innerHTML = `<th>${i+1}</th><td>${f(cfg.dh_a[i])}</td><td>${f(cfg.dh_d[i])}</td><td>${f(cfg.dh_alpha[i])}</td>`;
    dh.appendChild(tr);
  }

  // Limits table
  const lt = document.getElementById('limits-table');
  for (let i = 0; i < 6; i++) {
    const tr = document.createElement('tr');
    tr.innerHTML = `<th>${i+1}</th><td>${f(cfg.q_min[i],2)}</td><td>${f(cfg.q_max[i],2)}</td><td>${f(cfg.qdot_max[i],2)}</td><td>${f(cfg.q_home[i],4)}</td>`;
    lt.appendChild(tr);
  }

  // Server constants
  const st = document.getElementById('server-table');
  const serverConsts = [
    ['TWIST_SCALE', cfg.twist_scale],
    ['MAX_DT', cfg.max_dt + ' s'],
    ['MIN_DT', cfg.min_dt + ' s'],
    ['MAX_REACH', cfg.max_reach + ' m'],
    ['MU_THRESHOLD', cfg.mu_threshold],
  ];
  serverConsts.forEach(([k, v]) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<th>${k}</th><td class="val">${v}</td>`;
    st.appendChild(tr);
  });

  // Damping params
  const dt = document.getElementById('damping-table');
  const dampConsts = [
    ['mu_threshold', cfg.damping_mu_threshold],
    ['lambda_min', cfg.damping_lambda_min],
    ['lambda_max', cfg.damping_lambda_max],
  ];
  dampConsts.forEach(([k, v]) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<th>${k}</th><td class="val">${v}</td>`;
    dt.appendChild(tr);
  });

  // Store threshold for mu chart reference line
  window._muThreshold = cfg.mu_threshold;
}

// --- Live state update ---
function updateState(s) {
  // Twist
  document.getElementById('vx').textContent = f(s.twist[0]);
  document.getElementById('vy').textContent = f(s.twist[1]);
  document.getElementById('vz').textContent = f(s.twist[2]);
  document.getElementById('wx').textContent = f(s.twist[3]);
  document.getElementById('wy').textContent = f(s.twist[4]);
  document.getElementById('wz').textContent = f(s.twist[5]);

  // Joints
  for (let i = 0; i < 6; i++) {
    const tr = document.getElementById('j' + i);
    const tds = tr.querySelectorAll('td');
    tds[0].textContent = f(s.q[i]);
    tds[0].className = 'val';
    tds[1].textContent = f(rad2deg(s.q[i]), 1);
    tds[1].className = 'val';
    tds[2].textContent = f(s.qdot[i]);
    tds[2].className = 'val';
  }

  // EE
  document.getElementById('ee-x').textContent = f(s.ee_pos[0]);
  document.getElementById('ee-y').textContent = f(s.ee_pos[1]);
  document.getElementById('ee-z').textContent = f(s.ee_pos[2]);
  document.getElementById('ee-dist').textContent = f(s.ee_dist, 3) + ' m';

  // Manipulability
  const mu = s.mu;
  document.getElementById('mu-val').textContent = f(mu, 6);
  document.getElementById('lam-val').textContent = f(s.lam, 6);

  const muPct = Math.min(mu / 0.3 * 100, 100);
  const muColor = mu > 0.05 ? 'var(--green)' : mu > 0.01 ? 'var(--yellow)' : 'var(--red)';
  const muBar = document.getElementById('mu-bar');
  muBar.style.width = muPct + '%';
  muBar.style.background = muColor;

  // Feedback
  const fb = s.feedback;
  document.getElementById('feasible-val').textContent = fb.is_feasible ? 'YES' : 'NO';
  document.getElementById('feasible-val').style.color = fb.is_feasible ? 'var(--green)' : 'var(--red)';
  document.getElementById('ws-prox').textContent = f(fb.workspace_proximity, 3);

  // Joint limit proximity row
  const jlRow = document.getElementById('jl-row');
  const jlTds = jlRow.querySelectorAll('td');
  for (let i = 0; i < 6; i++) {
    const v = fb.joint_limit_proximity[i];
    jlTds[i].textContent = f(v, 3);
    jlTds[i].style.color = v > 0.9 ? 'var(--red)' : v > 0.7 ? 'var(--yellow)' : 'var(--text)';
  }

  // Status bar
  document.getElementById('n-conn').textContent = s.n_connections;
  document.getElementById('msg-rate').textContent = s.msgs_per_sec;
  document.getElementById('dt-val').textContent = f(s.dt * 1000, 1);

  // --- Charts ---
  const label = timeIdx++;

  pushChartData(jointChart, label, s.q);
  pushChartData(muChart, label, [mu]);
  const linMag = Math.sqrt(s.twist[0]**2 + s.twist[1]**2 + s.twist[2]**2);
  const angMag = Math.sqrt(s.twist[3]**2 + s.twist[4]**2 + s.twist[5]**2);
  pushChartData(twistChart, label, [linMag, angMag]);
  pushChartData(jlChart, label, fb.joint_limit_proximity);

  charts.forEach(c => c.update());
}
</script>
</body>
</html>"""
