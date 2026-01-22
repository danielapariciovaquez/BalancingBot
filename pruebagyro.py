import time
import threading
from flask import Flask, jsonify, render_template_string

from mpu6050 import mpu6050  # librería que ya estás usando


# ---------------------------
# Configuración
# ---------------------------
I2C_ADDR = 0x68
SAMPLE_HZ = 50.0  # frecuencia de muestreo (50 Hz es razonable para visualización web)
DT = 1.0 / SAMPLE_HZ


# ---------------------------
# Estado compartido (thread-safe)
# ---------------------------
latest = {
    "t": 0.0,
    "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
    "temp": 0.0,
}
lock = threading.Lock()


# ---------------------------
# Hilo de adquisición
# ---------------------------
def acquisition_loop():
    sensor = mpu6050(I2C_ADDR)  # no fuerzo bus para mantener el comportamiento que te funciona

    t0 = time.monotonic()
    next_t = time.monotonic()

    while True:
        now = time.monotonic()
        if now < next_t:
            time.sleep(next_t - now)
            continue
        next_t += DT

        accel = sensor.get_accel_data()
        gyro = sensor.get_gyro_data()
        temp = sensor.get_temp()

        with lock:
            latest["t"] = now - t0
            latest["accel"] = {
                "x": float(accel.get("x", 0.0)),
                "y": float(accel.get("y", 0.0)),
                "z": float(accel.get("z", 0.0)),
            }
            latest["gyro"] = {
                "x": float(gyro.get("x", 0.0)),
                "y": float(gyro.get("y", 0.0)),
                "z": float(gyro.get("z", 0.0)),
            }
            latest["temp"] = float(temp)


# ---------------------------
# Servidor web
# ---------------------------
app = Flask(__name__)

HTML = """
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MPU6050 - Live</title>
  <style>
    body { font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; margin: 20px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 14px; }
    .row { display: grid; grid-template-columns: 120px 1fr; gap: 10px; margin: 6px 0; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; }
    canvas { width: 100% !important; height: 300px !important; }
  </style>
</head>
<body>
  <h2>MPU6050 - Datos en tiempo real</h2>

  <div class="grid">
    <div class="card">
      <h3>Valores actuales</h3>
      <div class="row"><div>t (s)</div><div class="mono" id="t">-</div></div>

      <div class="row"><div>Accel X</div><div class="mono" id="ax">-</div></div>
      <div class="row"><div>Accel Y</div><div class="mono" id="ay">-</div></div>
      <div class="row"><div>Accel Z</div><div class="mono" id="az">-</div></div>

      <div class="row"><div>Gyro X</div><div class="mono" id="gx">-</div></div>
      <div class="row"><div>Gyro Y</div><div class="mono" id="gy">-</div></div>
      <div class="row"><div>Gyro Z</div><div class="mono" id="gz">-</div></div>

      <div class="row"><div>Temp (°C)</div><div class="mono" id="temp">-</div></div>
      <p class="mono">Endpoint JSON: <a href="/data">/data</a></p>
    </div>

    <div class="card">
      <h3>Acelerómetro (m/s²)</h3>
      <canvas id="accelChart"></canvas>
    </div>

    <div class="card">
      <h3>Giróscopo (deg/s)</h3>
      <canvas id="gyroChart"></canvas>
    </div>

    <div class="card">
      <h3>Temperatura (°C)</h3>
      <canvas id="tempChart"></canvas>
    </div>
  </div>

  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <script>
    const MAX_POINTS = 200; // ~4 s si actualizamos a 50 Hz

    function makeLineChart(ctx, labels, datasets) {
      return new Chart(ctx, {
        type: 'line',
        data: { labels, datasets },
        options: {
          animation: false,
          responsive: true,
          scales: { x: { display: false } }
        }
      });
    }

    const labels = [];
    const accelData = { x: [], y: [], z: [] };
    const gyroData  = { x: [], y: [], z: [] };
    const tempData  = [];

    const accelChart = makeLineChart(
      document.getElementById('accelChart'),
      labels,
      [
        { label: 'Ax', data: accelData.x },
        { label: 'Ay', data: accelData.y },
        { label: 'Az', data: accelData.z },
      ]
    );

    const gyroChart = makeLineChart(
      document.getElementById('gyroChart'),
      labels,
      [
        { label: 'Gx', data: gyroData.x },
        { label: 'Gy', data: gyroData.y },
        { label: 'Gz', data: gyroData.z },
      ]
    );

    const tempChart = makeLineChart(
      document.getElementById('tempChart'),
      labels,
      [
        { label: 'Temp', data: tempData },
      ]
    );

    function pushPoint(t, ax, ay, az, gx, gy, gz, temp) {
      labels.push(t);

      accelData.x.push(ax); accelData.y.push(ay); accelData.z.push(az);
      gyroData.x.push(gx);  gyroData.y.push(gy);  gyroData.z.push(gz);
      tempData.push(temp);

      if (labels.length > MAX_POINTS) {
        labels.shift();
        accelData.x.shift(); accelData.y.shift(); accelData.z.shift();
        gyroData.x.shift();  gyroData.y.shift();  gyroData.z.shift();
        tempData.shift();
      }
    }

    async function update() {
      try {
        const r = await fetch('/data', { cache: 'no-store' });
        const d = await r.json();

        document.getElementById('t').textContent = d.t.toFixed(3);
        document.getElementById('ax').textContent = d.accel.x.toFixed(4);
        document.getElementById('ay').textContent = d.accel.y.toFixed(4);
        document.getElementById('az').textContent = d.accel.z.toFixed(4);
        document.getElementById('gx').textContent = d.gyro.x.toFixed(4);
        document.getElementById('gy').textContent = d.gyro.y.toFixed(4);
        document.getElementById('gz').textContent = d.gyro.z.toFixed(4);
        document.getElementById('temp').textContent = d.temp.toFixed(2);

        pushPoint(d.t, d.accel.x, d.accel.y, d.accel.z, d.gyro.x, d.gyro.y, d.gyro.z, d.temp);
        accelChart.update();
        gyroChart.update();
        tempChart.update();
      } catch (e) {
        // si hay fallo puntual, simplemente no actualizamos ese ciclo
      } finally {
        setTimeout(update, 100); // 10 Hz de refresco web (suficiente y ligero)
      }
    }

    update();
  </script>
</body>
</html>
"""

@app.get("/")
def index():
    return render_template_string(HTML)

@app.get("/data")
def data():
    with lock:
        return jsonify(latest)


if __name__ == "__main__":
    th = threading.Thread(target=acquisition_loop, daemon=True)
    th.start()

    # host=0.0.0.0 para verlo desde otro equipo de la red
    app.run(host="0.0.0.0", port=5000, debug=False)
