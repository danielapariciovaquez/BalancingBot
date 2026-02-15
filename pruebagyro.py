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

READ_HZ = 200          # lectura IMU en backend
WEB_POLL_MS = 100      # refresco del navegador (ms)

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 5000

# Escalas por defecto (reset):
# accel FS=±2g, gyro FS=±250°/s
ACCEL_SCALE = 16384.0  # LSB/g
GYRO_SCALE = 131.0     # LSB/(°/s)

# Registros MPU6050
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
# ==================================================

app = Flask(__name__)

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def _to_int16(v: int) -> int:
    return v - 0x10000 if (v & 0x8000) else v

def lpf_alpha(dt: float, tau: float) -> float:
    # alpha = dt / (tau + dt)  (IIR 1st order, equivalente RC discreto)
    # tau<=0 => sin filtrado (alpha=1)
    if tau <= 0.0:
        return 1.0
    return dt / (tau + dt)

@dataclass
class Params:
    tau_acc: float = 0.10   # s (LPF sobre Az)
    tau_gyro: float = 0.02  # s (LPF sobre Gz)
    alpha_comp: float = 0.98  # complementario: angle = alpha*(gyro) + (1-alpha)*(acc)

params = Params()
params_lock = threading.Lock()

@dataclass
class ZState:
    ts: float = 0.0
    dt: float = 0.0

    az_g_raw: float = 0.0
    gz_dps_raw: float = 0.0

    az_g_f: float = 0.0
    gz_dps_f: float = 0.0

    angle_acc_deg: float = 0.0      # acos(Az) en grados (0..180)
    angle_gyro_deg: float = 0.0     # integral de gz
    angle_fused_deg: float = 0.0    # complementario

    ok: bool = False
    err: str = ""

state = ZState()
state_lock = threading.Lock()
stop_evt = threading.Event()

# Para reset desde la web
reset_gyro_evt = threading.Event()


def imu_thread():
    global state
    period = 1.0 / float(READ_HZ)

    try:
        bus = SMBus(I2C_BUS)
    except Exception as e:
        with state_lock:
            state.ok = False
            state.err = f"SMBus open failed: {repr(e)}"
        return

    try:
        bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)  # wake
    except Exception as e:
        with state_lock:
            state.ok = False
            state.err = f"MPU init failed: {repr(e)}"
        try:
            bus.close()
        except Exception:
            pass
        return

    # filtros (memoria)
    az_f = 0.0
    gz_f = 0.0
    angle_gyro = 0.0
    angle_fused = 0.0
    t_prev = time.time()

    while not stop_evt.is_set():
        t0 = time.time()
        dt = t0 - t_prev
        if dt <= 0:
            dt = 1.0 / READ_HZ
        t_prev = t0

        if reset_gyro_evt.is_set():
            reset_gyro_evt.clear()
            angle_gyro = 0.0
            angle_fused = 0.0

        try:
            data = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 14)

            # accel Z
            az = _to_int16((data[4] << 8) | data[5]) / ACCEL_SCALE
            # gyro Z
            gz = _to_int16((data[12] << 8) | data[13]) / GYRO_SCALE

            with params_lock:
                tau_acc = float(params.tau_acc)
                tau_gyro = float(params.tau_gyro)
                alpha_comp = float(params.alpha_comp)

            # LPF sobre señales
            a_acc = lpf_alpha(dt, tau_acc)
            a_gyr = lpf_alpha(dt, tau_gyro)

            az_f = az_f + a_acc * (az - az_f)
            gz_f = gz_f + a_gyr * (gz - gz_f)

            # Ángulo desde accel Z (respecto a gravedad)
            # az_f es ~cos(theta) si el eje Z está alineado con g.
            az_c = clamp(az_f, -1.0, 1.0)
            angle_acc = math.degrees(math.acos(az_c))  # 0..180

            # Integración gyro
            angle_gyro += gz_f * dt  # deg/s * s = deg

            # Complementario (ojo: angle_acc está 0..180, gyro es relativo; aquí es “demostración” para Z)
            angle_fused = alpha_comp * (angle_fused + gz_f * dt) + (1.0 - alpha_comp) * angle_acc

            with state_lock:
                state.ts = t0
                state.dt = dt

                state.az_g_raw = float(az)
                state.gz_dps_raw = float(gz)
                state.az_g_f = float(az_f)
                state.gz_dps_f = float(gz_f)

                state.angle_acc_deg = float(angle_acc)
                state.angle_gyro_deg = float(angle_gyro)
                state.angle_fused_deg = float(angle_fused)

                state.ok = True
                state.err = ""

        except Exception as e:
            with state_lock:
                state.ts = t0
                state.dt = dt
                state.ok = False
                state.err = repr(e)
            time.sleep(0.05)

        # pacing
        dt_loop = time.time() - t0
        sleep_s = period - dt_loop
        if sleep_s > 0:
            time.sleep(sleep_s)

    try:
        bus.close()
    except Exception:
        pass


@app.get("/api/z")
def api_z():
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
    return jsonify({"ok": True})


@app.post("/api/reset_gyro")
def api_reset_gyro():
    reset_gyro_evt.set()
    return jsonify({"ok": True})


@app.get("/")
def index():
    html = f"""<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>MPU6050 Z — Ángulo y filtro</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 20px; }}
    .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; max-width: 980px; }}
    .card {{ border: 1px solid #ddd; border-radius: 10px; padding: 14px; }}
    .title {{ font-weight: 700; margin-bottom: 10px; }}
    .mono {{ font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }}
    .ok {{ color: #0a7; font-weight: 700; }}
    .bad {{ color: #c22; font-weight: 700; }}
    .row {{ display:flex; gap:12px; align-items:center; margin: 10px 0; }}
    input[type=range] {{ width: 320px; }}
    .small {{ color:#666; font-size: 12px; }}
    table {{ width:100%; border-collapse: collapse; }}
    td {{ padding: 6px 0; }}
    td.k {{ width: 160px; color:#555; }}
  </style>
</head>
<body>
  <h2>MPU6050 (GY-521) — Solo eje Z: conversión a grados y filtrado</h2>
  <div class="small">
    Backend: {READ_HZ} Hz · Web: {int(WEB_POLL_MS)} ms · I2C addr: 0x{MPU_ADDR:02X}
  </div>

  <div style="margin:10px 0">
    Estado: <span id="status" class="mono">...</span>
    &nbsp;|&nbsp; ts: <span id="ts" class="mono">...</span>
    &nbsp;|&nbsp; dt: <span id="dt" class="mono">...</span>
  </div>

  <div class="grid">
    <div class="card">
      <div class="title">Señales Z</div>
      <table>
        <tr><td class="k">Az raw (g)</td><td class="mono" id="azr">-</td></tr>
        <tr><td class="k">Az LPF (g)</td><td class="mono" id="azf">-</td></tr>
        <tr><td class="k">Gz raw (°/s)</td><td class="mono" id="gzr">-</td></tr>
        <tr><td class="k">Gz LPF (°/s)</td><td class="mono" id="gzf">-</td></tr>
      </table>
    </div>

    <div class="card">
      <div class="title">Ángulos (deg)</div>
      <table>
        <tr><td class="k">Angle acc = acos(Az)</td><td class="mono" id="aacc">-</td></tr>
        <tr><td class="k">Angle gyro (integrado)</td><td class="mono" id="agyr">-</td></tr>
        <tr><td class="k">Angle fused (complement.)</td><td class="mono" id="afus">-</td></tr>
      </table>
      <div class="row">
        <button id="btnReset">Reset gyro</button>
      </div>
      <div class="small">
        Nota: angle_acc aquí es el ángulo respecto al eje Z (0° si Z || g).
      </div>
    </div>

    <div class="card" style="grid-column: 1 / -1;">
      <div class="title">Ajustes</div>

      <div class="row">
        <div style="width:160px">τ accel (s)</div>
        <input id="tauAcc" type="range" min="0" max="1.0" step="0.005" value="0.10">
        <div class="mono" id="tauAccV">0.10</div>
      </div>

      <div class="row">
        <div style="width:160px">τ gyro (s)</div>
        <input id="tauGyr" type="range" min="0" max="0.5" step="0.002" value="0.02">
        <div class="mono" id="tauGyrV">0.02</div>
      </div>

      <div class="row">
        <div style="width:160px">alpha complement.</div>
        <input id="alpha" type="range" min="0" max="1.0" step="0.001" value="0.98">
        <div class="mono" id="alphaV">0.98</div>
      </div>

      <div class="small mono" id="err">-</div>
    </div>
  </div>

<script>
const POLL_MS = {int(WEB_POLL_MS)};
const fmt = (v, n=4) => (typeof v === "number" ? v.toFixed(n) : String(v));

async function postJSON(url, obj){{
  await fetch(url, {{
    method: "POST",
    headers: {{"Content-Type":"application/json"}},
    body: JSON.stringify(obj)
  }});
}}

function bindSlider(id, outId, key){{
  const s = document.getElementById(id);
  const o = document.getElementById(outId);
  const send = async ()=> {{
    const v = Number(s.value);
    o.textContent = fmt(v, 3);
    const payload = {{}};
    payload[key] = v;
    await postJSON("/api/params", payload);
  }};
  s.addEventListener("input", ()=>{{ o.textContent = fmt(Number(s.value), 3); }});
  s.addEventListener("change", send);
}}

bindSlider("tauAcc","tauAccV","tau_acc");
bindSlider("tauGyr","tauGyrV","tau_gyro");
bindSlider("alpha","alphaV","alpha_comp");

document.getElementById("btnReset").addEventListener("click", async ()=>{
  await fetch("/api/reset_gyro", {{method:"POST"}});
});

async function tick(){{
  try {{
    const r = await fetch("/api/z", {{cache:"no-store"}});
    const d = await r.json();
    const s = d.state;

    document.getElementById("azr").textContent = fmt(s.az_g_raw);
    document.getElementById("azf").textContent = fmt(s.az_g_f);
    document.getElementById("gzr").textContent = fmt(s.gz_dps_raw);
    document.getElementById("gzf").textContent = fmt(s.gz_dps_f);

    document.getElementById("aacc").textContent = fmt(s.angle_acc_deg, 3);
    document.getElementById("agyr").textContent = fmt(s.angle_gyro_deg, 3);
    document.getElementById("afus").textContent = fmt(s.angle_fused_deg, 3);

    document.getElementById("ts").textContent = fmt(s.ts, 3);
    document.getElementById("dt").textContent = fmt(s.dt, 4);

    const st = document.getElementById("status");
    if (s.ok) {{
      st.textContent = "OK";
      st.className = "mono ok";
      document.getElementById("err").textContent = "-";
    }} else {{
      st.textContent = "ERROR";
      st.className = "mono bad";
      document.getElementById("err").textContent = s.err || "-";
    }}
  }} catch (e) {{
    const st = document.getElementById("status");
    st.textContent = "HTTP ERROR";
    st.className = "mono bad";
    document.getElementById("err").textContent = String(e);
  }}
}}

tick();
setInterval(tick, POLL_MS);
</script>
</body>
</html>
"""
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
