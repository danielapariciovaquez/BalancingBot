#!/usr/bin/env python3
import time
import threading
import signal
import sys
from dataclasses import dataclass, asdict

from smbus2 import SMBus
from flask import Flask, jsonify, Response

# ===================== CONFIG =====================
I2C_BUS = 1
MPU_ADDR = 0x68

READ_HZ = 100          # lectura IMU en backend
WEB_POLL_MS = 200      # refresco del navegador (ms)

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 5000

# Escalas por defecto (MPU6050 reset):
# accel FS=±2g, gyro FS=±250°/s
ACCEL_SCALE = 16384.0  # LSB/g
GYRO_SCALE = 131.0     # LSB/(°/s)

# Registros MPU6050
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
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


def _to_int16(v: int) -> int:
    # v en 0..65535
    if v & 0x8000:
        return v - 0x10000
    return v


def imu_thread():
    global sample
    period = 1.0 / float(READ_HZ)

    try:
        bus = SMBus(I2C_BUS)
    except Exception as e:
        with lock:
            sample.ok = False
            sample.err = f"SMBus open failed: {repr(e)}"
        return

    try:
        # Wake up MPU6050
        bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    except Exception as e:
        with lock:
            sample.ok = False
            sample.err = f"MPU init failed: {repr(e)}"
        try:
            bus.close()
        except Exception:
            pass
        return

    while not stop_evt.is_set():
        t0 = time.time()
        try:
            # Leer 14 bytes: accel(6) temp(2) gyro(6)
            data = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 14)

            ax = _to_int16((data[0] << 8) | data[1]) / ACCEL_SCALE
            ay = _to_int16((data[2] << 8) | data[3]) / ACCEL_SCALE
            az = _to_int16((data[4] << 8) | data[5]) / ACCEL_SCALE

            gx = _to_int16((data[8] << 8) | data[9]) / GYRO_SCALE
            gy = _to_int16((data[10] << 8) | data[11]) / GYRO_SCALE
            gz = _to_int16((data[12] << 8) | data[13]) / GYRO_SCALE

            with lock:
                sample.ts = t0
                sample.ax_g = float(ax)
                sample.ay_g = float(ay)
                sample.az_g = float(az)
                sample.gx_dps = float(gx)
                sample.gy_dps = float(gy)
                sample.gz_dps = float(gz)
                sample.ok = True
                sample.err = ""

        except Exception as e:
            with lock:
                sample.ts = t0
                sample.ok = False
                sample.err = repr(e)
            time.sleep(0.05)  # evita bucle loco si I2C cae

        dt = time.time() - t0
        sleep_s = period - dt
        if sleep_s > 0:
            time.sleep(sleep_s)

    try:
        bus.close()
    except Exception:
        pass


@app.get("/api/imu")
def api_imu():
    with lock:
        return jsonify(asdict(sample))


@app.get("/")
def index():
    html = f"""<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>MPU6050 - IMU Monitor</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 20px; }}
    .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; max-width: 900px; }}
    .card {{ border: 1px solid #ddd; border-radius: 10px; padding: 14px; }}
    .title {{ font-weight: 700; margin-bottom: 10px; }}
    table {{ width: 100%; border-collapse: collapse; }}
    td {{ padding: 6px 0; }}
    td.k {{ width: 44px; color: #555; }}
    .ok {{ color: #0a7; font-weight: 700; }}
    .bad {{ color: #c22; font-weight: 700; }}
    .mono {{ font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }}
    .small {{ color: #666; font-size: 12px; }}
  </style>
</head>
<body>
  <h2>MPU6050 (GY-521) — Acelerómetro y Giroscopio</h2>
  <div class="small">
    Backend: {READ_HZ} Hz · Web: {int(WEB_POLL_MS)} ms · I2C addr: 0x{MPU_ADDR:02X}
  </div>

  <div style="margin:10px 0">
    Estado: <span id="status" class="mono">...</span>
    &nbsp;|&nbsp; ts: <span id="ts" class="mono">...</span>
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

async function tick() {{
  try {{
    const r = await fetch("/api/imu", {{cache:"no-store"}});
    const d = await r.json();

    document.getElementById("ax").textContent = fmt(d.ax_g);
    document.getElementById("ay").textContent = fmt(d.ay_g);
    document.getElementById("az").textContent = fmt(d.az_g);

    document.getElementById("gx").textContent = fmt(d.gx_dps);
    document.getElementById("gy").textContent = fmt(d.gy_dps);
    document.getElementById("gz").textContent = fmt(d.gz_dps);

    document.getElementById("ts").textContent = fmt(d.ts);

    const st = document.getElementById("status");
    if (d.ok) {{
      st.textContent = "OK";
      st.className = "mono ok";
      document.getElementById("err").textContent = "-";
    }} else {{
      st.textContent = "ERROR";
      st.className = "mono bad";
      document.getElementById("err").textContent = d.err || "-";
    }}
  }} catch (e) {{
    const st = document.getElementById("status");
    st.textContent = "HTTP ERROR";
    st.className = "mono bad";
    document.getElementById("err").textContent = String(e);
  }}
}}

tick();
setInterval(tick, {int(WEB_POLL_MS)});
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

    # Flask dev server suficiente en LAN local
    app.run(host=HTTP_HOST, port=HTTP_PORT, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
