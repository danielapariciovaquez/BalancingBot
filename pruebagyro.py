#!/usr/bin/env python3
import time
import threading
import signal
import sys
from dataclasses import dataclass, asdict

from flask import Flask, jsonify, Response
from mpu6050 import mpu6050

# ===================== CONFIG =====================
I2C_ADDRESS = 0x68          # 0x68 típico en GY-521 (AD0=0). Si AD0=1 -> 0x69
READ_HZ = 100               # frecuencia de lectura IMU
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 5000
# ==================================================

app = Flask(__name__)

@dataclass
class ImuSample:
    ts: float = 0.0
    ax_g: float = 0.0
    ay_g: float = 0.0
    az_g: float = 0.0
    gx_dps: float = 0.0
    gy_dps: float = 0.0
    gz_dps: float = 0.0
    ok: bool = False
    err: str = ""

sample = ImuSample()
lock = threading.Lock()
stop_evt = threading.Event()


def imu_thread():
    global sample
    sensor = mpu6050(I2C_ADDRESS)

    # Nota: la librería ya configura escalas por defecto (acel en g y gyro en deg/s).
    # Si quieres fijar rangos: sensor.set_accel_range(...), sensor.set_gyro_range(...)
    period = 1.0 / float(READ_HZ)

    last_ok = True
    while not stop_evt.is_set():
        t0 = time.time()
        try:
            a = sensor.get_accel_data()  # {'x':..,'y':..,'z':..} en g
            g = sensor.get_gyro_data()   # {'x':..,'y':..,'z':..} en deg/s

            with lock:
                sample.ts = t0
                sample.ax_g = float(a["x"])
                sample.ay_g = float(a["y"])
                sample.az_g = float(a["z"])
                sample.gx_dps = float(g["x"])
                sample.gy_dps = float(g["y"])
                sample.gz_dps = float(g["z"])
                sample.ok = True
                sample.err = ""
            last_ok = True

        except Exception as e:
            # Mantén el último valor válido, pero marca error/estado
            with lock:
                sample.ts = t0
                sample.ok = False
                sample.err = repr(e)
            # si falla seguido, no spamear CPU
            if last_ok:
                last_ok = False
            time.sleep(0.05)

        dt = time.time() - t0
        sleep_s = period - dt
        if sleep_s > 0:
            time.sleep(sleep_s)


@app.get("/api/imu")
def api_imu():
    with lock:
        return jsonify(asdict(sample))


@app.get("/")
def index():
    # HTML embebido: UI mínima y clara
    html = """<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>IMU MPU6050 - Monitor</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 20px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; max-width: 900px; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 14px; }
    .title { font-weight: 700; margin-bottom: 10px; }
    table { width: 100%; border-collapse: collapse; }
    td { padding: 6px 0; }
    td.k { width: 40px; color: #555; }
    .ok { color: #0a7; font-weight: 700; }
    .bad { color: #c22; font-weight: 700; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .small { color: #666; font-size: 12px; }
  </style>
</head>
<body>
  <h2>MPU6050 (GY-521) — Acelerómetro y Giroscopio</h2>
  <div class="small">Actualización: 5 Hz (web). Lectura IMU: 100 Hz (backend).</div>
  <div style="margin:10px 0">
    Estado: <span id="status" class="mono">...</span> |
    ts: <span id="ts" class="mono">...</span>
  </div>

  <div class="grid">
    <div class="card">
      <div class="title">Aceleración (g)</div>
      <table>
        <tr><td class="k">Ax</td><td class="mono" id="ax">-</td></tr>
        <tr><td class="k">Ay</td><td class="mono" id="ay">-</td></tr>
        <tr><td class="k">Az</td><td class="mono" id="az">-</td></tr>
      </table>
    </div>

    <div class="card">
      <div class="title">Giroscopio (°/s)</div>
      <table>
        <tr><td class="k">Gx</td><td class="mono" id="gx">-</td></tr>
        <tr><td class="k">Gy</td><td class="mono" id="gy">-</td></tr>
        <tr><td class="k">Gz</td><td class="mono" id="gz">-</td></tr>
      </table>
    </div>

    <div class="card" style="grid-column: 1 / -1;">
      <div class="title">Error (si aplica)</div>
      <div class="mono" id="err">-</div>
    </div>
  </div>

<script>
const fmt = (v) => (typeof v === "number" ? v.toFixed(4) : String(v));
async function tick(){
  try{
    const r = await fetch("/api/imu", {cache:"no-store"});
    const d = await r.json();

    document.getElementById("ax").textContent = fmt(d.ax_g);
    document.getElementById("ay").textContent = fmt(d.ay_g);
    document.getElementById("az").textContent = fmt(d.az_g);

    document.getElementById("gx").textContent = fmt(d.gx_dps);
    document.getElementById("gy").textContent = fmt(d.gy_dps);
    document.getElementById("gz").textContent = fmt(d.gz_dps);

    document.getElementById("ts").textContent = fmt(d.ts);

    const st = document.getElementById("status");
    if(d.ok){
      st.textContent = "OK";
      st.className = "mono ok";
      document.getElementById("err").textContent = "-";
    }else{
      st.textContent = "ERROR";
      st.className = "mono bad";
      document.getElementById("err").textContent = d.err || "-";
    }
  }catch(e){
    const st = document.getElementById("status");
    st.textContent = "HTTP ERROR";
    st.className = "mono bad";
    document.getElementById("err").textContent = String(e);
  }
}
tick();
setInterval(tick, 200);
</script>
</body>
</html>
"""
    return Response(html, mimetype="text/html")


def on_exit(signum=None, frame=None):
    stop_evt.set()
    # deja un margen para que pare el hilo
    time.sleep(0.1)
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)

    t = threading.Thread(target=imu_thread, daemon=True)
    t.start()

    # Flask (dev server) suficiente para LAN local
    app.run(host=HTTP_HOST, port=HTTP_PORT, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
