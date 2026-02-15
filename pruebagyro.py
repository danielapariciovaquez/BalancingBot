#!/usr/bin/env python3
import time
import threading
import signal
import sys
import math
from dataclasses import dataclass, asdict

from smbus2 import SMBus
from flask import Flask, jsonify, Response, request

# ===================== CONFIG =====================
I2C_BUS = 1
MPU_ADDR = 0x68

READ_HZ = 200
WEB_POLL_MS = 100

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 5000

ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
# ==================================================

app = Flask(__name__)

def _to_int16(v: int) -> int:
    return v - 0x10000 if (v & 0x8000) else v

def lpf_alpha(dt: float, tau: float) -> float:
    if tau <= 0.0:
        return 1.0
    return dt / (tau + dt)

@dataclass
class Params:
    tau_acc: float = 0.10
    tau_gyro: float = 0.02
    alpha_comp: float = 0.98
    invert_gyro: bool = False
    invert_angle: bool = False

params = Params()
params_lock = threading.Lock()

@dataclass
class PitchState:
    ts: float = 0.0
    dt: float = 0.0

    ax_g_f: float = 0.0
    ay_g_f: float = 0.0
    az_g_f: float = 0.0
    gy_dps_f: float = 0.0

    pitch_acc_deg: float = 0.0
    pitch_gyro_deg: float = 0.0
    pitch_fused_deg: float = 0.0

    ok: bool = False
    err: str = ""

state = PitchState()
state_lock = threading.Lock()
stop_evt = threading.Event()
reset_evt = threading.Event()


def imu_thread():
    period = 1.0 / float(READ_HZ)

    try:
        bus = SMBus(I2C_BUS)
    except Exception as e:
        with state_lock:
            state.ok = False
            state.err = f"SMBus open failed: {repr(e)}"
        return

    try:
        bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    except Exception as e:
        with state_lock:
            state.ok = False
            state.err = f"MPU init failed: {repr(e)}"
        try:
            bus.close()
        except Exception:
            pass
        return

    ax_f = ay_f = az_f = 0.0
    gy_f = 0.0
    pitch_gyro = 0.0
    pitch_fused = 0.0

    t_prev = time.time()

    while not stop_evt.is_set():
        t0 = time.time()
        dt = t0 - t_prev
        if dt <= 0.0:
            dt = 1.0 / READ_HZ
        t_prev = t0

        if reset_evt.is_set():
            reset_evt.clear()
            pitch_gyro = 0.0
            pitch_fused = 0.0

        try:
            data = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 14)

            ax = _to_int16((data[0] << 8) | data[1]) / ACCEL_SCALE
            ay = _to_int16((data[2] << 8) | data[3]) / ACCEL_SCALE
            az = _to_int16((data[4] << 8) | data[5]) / ACCEL_SCALE

            gy = _to_int16((data[10] << 8) | data[11]) / GYRO_SCALE  # Gy

            with params_lock:
                tau_acc = float(params.tau_acc)
                tau_gyro = float(params.tau_gyro)
                alpha = float(params.alpha_comp)
                inv_g = bool(params.invert_gyro)
                inv_a = bool(params.invert_angle)

            if inv_g:
                gy = -gy

            a_acc = lpf_alpha(dt, tau_acc)
            a_gyr = lpf_alpha(dt, tau_gyro)

            ax_f = ax_f + a_acc * (ax - ax_f)
            ay_f = ay_f + a_acc * (ay - ay_f)
            az_f = az_f + a_acc * (az - az_f)
            gy_f = gy_f + a_gyr * (gy - gy_f)

            denom = math.sqrt(ay_f * ay_f + az_f * az_f)
            pitch_acc = math.degrees(math.atan2(-ax_f, denom))

            pitch_gyro += gy_f * dt
            pitch_fused = alpha * (pitch_fused + gy_f * dt) + (1.0 - alpha) * pitch_acc

            if inv_a:
                pitch_acc = -pitch_acc
                pitch_gyro = -pitch_gyro
                pitch_fused = -pitch_fused

            with state_lock:
                state.ts = t0
                state.dt = dt
                state.ax_g_f = float(ax_f)
                state.ay_g_f = float(ay_f)
                state.az_g_f = float(az_f)
                state.gy_dps_f = float(gy_f)
                state.pitch_acc_deg = float(pitch_acc)
                state.pitch_gyro_deg = float(pitch_gyro)
                state.pitch_fused_deg = float(pitch_fused)
                state.ok = True
                state.err = ""

        except Exception as e:
            with state_lock:
                state.ts = t0
                state.dt = dt
                state.ok = False
                state.err = repr(e)
            time.sleep(0.05)

        loop_dt = time.time() - t0
        sleep_s = period - loop_dt
        if sleep_s > 0:
            time.sleep(sleep_s)

    try:
        bus.close()
    except Exception:
        pass


@app.get("/api/pitch")
def api_pitch():
    with state_lock:
        s = asdict(state)
    with params_lock:
        p = asdict(params)
    return jsonify({"state": s, "params": p})


@app.post("/api/params")
def api_params():
    data = request.get_json(force=True, silent=True) or {}
    with params_lock:
        if "tau_acc" in data:
            params.tau_acc = float(data["tau_acc"])
        if "tau_gyro" in data:
            params.tau_gyro = float(data["tau_gyro"])
        if "alpha_comp" in data:
            params.alpha_comp = float(data["alpha_comp"])
        if "invert_gyro" in data:
            params.invert_gyro = bool(data["invert_gyro"])
        if "invert_angle" in data:
            params.invert_angle = bool(data["invert_angle"])
    return jsonify({"ok": True})


@app.post("/api/reset")
def api_reset():
    reset_evt.set()
    return jsonify({"ok": True})


@app.get("/")
def index():
    html = """<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>MPU6050 — Pitch (deg) + filtrado</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 20px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; max-width: 980px; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 14px; }
    .title { font-weight: 700; margin-bottom: 10px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .ok { color: #0a7; font-weight: 700; }
    .bad { color: #c22; font-weight: 700; }
    .row { display:flex; gap:12px; align-items:center; margin: 10px 0; flex-wrap: wrap; }
    input[type=range] { width: 320px; }
    .small { color:#666; font-size: 12px; }
    table { width:100%; border-collapse: collapse; }
    td { padding: 6px 0; }
    td.k { width: 170px; color:#555; }
  </style>
</head>
<body>
  <h2>MPU6050 (GY-521) — Pitch (deg) + filtrado</h2>
  <div class="small" id="meta"></div>

  <div style="margin:10px 0">
    Estado: <span id="status" class="mono">...</span>
    &nbsp;|&nbsp; ts: <span id="ts" class="mono">...</span>
    &nbsp;|&nbsp; dt: <span id="dt" class="mono">...</span>
  </div>

  <div class="grid">
    <div class="card">
      <div class="title">Señales (filtradas)</div>
      <table>
        <tr><td class="k">Ax (g)</td><td class="mono" id="ax">-</td></tr>
        <tr><td class="k">Ay (g)</td><td class="mono" id="ay">-</td></tr>
        <tr><td class="k">Az (g)</td><td class="mono" id="az">-</td></tr>
        <tr><td class="k">Gy (°/s)</td><td class="mono" id="gy">-</td></tr>
      </table>
    </div>

    <div class="card">
      <div class="title">Pitch (deg)</div>
      <table>
        <tr><td class="k">Pitch acc</td><td class="mono" id="pacc">-</td></tr>
        <tr><td class="k">Pitch gyro (int)</td><td class="mono" id="pgyr">-</td></tr>
        <tr><td class="k">Pitch fused (comp)</td><td class="mono" id="pfus">-</td></tr>
      </table>
      <div class="row">
        <button id="btnReset">Reset</button>
      </div>
      <div class="small">
        Si el signo va al revés, usa “Invert gyro” o “Invert angle”.
      </div>
    </div>

    <div class="card" style="grid-column: 1 / -1;">
      <div class="title">Ajustes</div>

      <div class="row">
        <div style="width:170px">τ accel (s)</div>
        <input id="tauAcc" type="range" min="0" max="1.0" step="0.005" value="0.10">
        <div class="mono" id="tauAccV">0.10</div>
      </div>

      <div class="row">
        <div style="width:170px">τ gyro (s)</div>
        <input id="tauGyr" type="range" min="0" max="0.5" step="0.002" value="0.02">
        <div class="mono" id="tauGyrV">0.02</div>
      </div>

      <div class="row">
        <div style="width:170px">alpha complement.</div>
        <input id="alpha" type="range" min="0" max="1.0" step="0.001" value="0.98">
        <div class="mono" id="alphaV">0.98</div>
      </div>

      <div class="row">
        <label><input id="invGyro" type="checkbox"> Invert gyro (Gy)</label>
        <label><input id="invAng" type="checkbox"> Invert angle (pitch)</label>
      </div>

      <div class="small mono" id="err">-</div>
    </div>
  </div>

<script>
const READ_HZ = __READ_HZ__;
const WEB_POLL_MS = __WEB_POLL_MS__;
const MPU_ADDR = "__MPU_ADDR__";

document.getElementById("meta").textContent =
  `Backend: ${READ_HZ} Hz · Web: ${WEB_POLL_MS} ms · I2C addr: ${MPU_ADDR} · Pitch_acc = atan2(-Ax, sqrt(Ay^2+Az^2)) · Gyro: Gy`;

const fmt = (v, n=4) => (typeof v === "number" ? v.toFixed(n) : String(v));

async function postJSON(url, obj){
  await fetch(url, {
    method: "POST",
    headers: {"Content-Type":"application/json"},
    body: JSON.stringify(obj)
  });
}

function bindSlider(id, outId, key){
  const s = document.getElementById(id);
  const o = document.getElementById(outId);
  const send = async ()=> {
    const v = Number(s.value);
    o.textContent = fmt(v, 3);
    const payload = {};
    payload[key] = v;
    await postJSON("/api/params", payload);
  };
  s.addEventListener("input", ()=>{ o.textContent = fmt(Number(s.value), 3); });
  s.addEventListener("change", send);
}

bindSlider("tauAcc","tauAccV","tau_acc");
bindSlider("tauGyr","tauGyrV","tau_gyro");
bindSlider("alpha","alphaV","alpha_comp");

document.getElementById("invGyro").addEventListener("change", async (e)=>{
  await postJSON("/api/params", {invert_gyro: e.target.checked});
});
document.getElementById("invAng").addEventListener("change", async (e)=>{
  await postJSON("/api/params", {invert_angle: e.target.checked});
});

document.getElementById("btnReset").addEventListener("click", async ()=>{
  await fetch("/api/reset", {method:"POST"});
});

async function tick(){
  try {
    const r = await fetch("/api/pitch", {cache:"no-store"});
    const d = await r.json();
    const s = d.state;

    document.getElementById("ax").textContent = fmt(s.ax_g_f);
    document.getElementById("ay").textContent = fmt(s.ay_g_f);
    document.getElementById("az").textContent = fmt(s.az_g_f);
    document.getElementById("gy").textContent = fmt(s.gy_dps_f);

    document.getElementById("pacc").textContent = fmt(s.pitch_acc_deg, 3);
    document.getElementById("pgyr").textContent = fmt(s.pitch_gyro_deg, 3);
    document.getElementById("pfus").textContent = fmt(s.pitch_fused_deg, 3);

    document.getElementById("ts").textContent = fmt(s.ts, 3);
    document.getElementById("dt").textContent = fmt(s.dt, 4);

    const st = document.getElementById("status");
    if (s.ok) {
      st.textContent = "OK";
      st.className = "mono ok";
      document.getElementById("err").textContent = "-";
    } else {
      st.textContent = "ERROR";
      st.className = "mono bad";
      document.getElementById("err").textContent = s.err || "-";
    }
  } catch (e) {
    const st = document.getElementById("status");
    st.textContent = "HTTP ERROR";
    st.className = "mono bad";
    document.getElementById("err").textContent = String(e);
  }
}

tick();
setInterval(tick, WEB_POLL_MS);
</script>
</body>
</html>
"""
    html = html.replace("__READ_HZ__", str(READ_HZ))
    html = html.replace("__WEB_POLL_MS__", str(int(WEB_POLL_MS)))
    html = html.replace("__MPU_ADDR__", f"0x{MPU_ADDR:02X}")
    return Response(html, mimetype="text/html")


def shutdown_handler(signum=None, frame=None):
    stop_evt.set()
    time.sleep(0.1)
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    t = threading.Thread(target=imu_thread, daemon=True)
    t.start()

    app.run(host=HTTP_HOST, port=HTTP_PORT, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
