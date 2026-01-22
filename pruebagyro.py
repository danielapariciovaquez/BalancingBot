import time
import math
import threading
from flask import Flask, jsonify, render_template_string

from mpu6050 import mpu6050  # usa tu librería actual (vía shim smbus->smbus2)

I2C_ADDR = 0x68
SAMPLE_HZ = 100.0
DT = 1.0 / SAMPLE_HZ

latest = {
    "t": 0.0,
    "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
    "temp": 0.0,
    "tilt": {"roll": 0.0, "pitch": 0.0},  # radianes
}
lock = threading.Lock()


def compute_roll_pitch_from_accel(ax: float, ay: float, az: float) -> tuple[float, float]:
    """
    Estimación de orientación con acelerómetro (inclinómetro):
      roll  = atan2(ay, az)
      pitch = atan2(-ax, sqrt(ay^2 + az^2))
    Devuelve (roll, pitch) en radianes.
    """
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return roll, pitch


def acquisition_loop():
    sensor = mpu6050(I2C_ADDR)  # no fuerzo bus para mantener tu comportamiento funcional

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

        ax = float(accel.get("x", 0.0))
        ay = float(accel.get("y", 0.0))
        az = float(accel.get("z", 0.0))

        roll, pitch = compute_roll_pitch_from_accel(ax, ay, az)

        with lock:
            latest["t"] = now - t0
            latest["accel"] = {"x": ax, "y": ay, "z": az}
            latest["gyro"] = {
                "x": float(gyro.get("x", 0.0)),
                "y": float(gyro.get("y", 0.0)),
                "z": float(gyro.get("z", 0.0)),
            }
            latest["temp"] = float(temp)
            latest["tilt"] = {"roll": float(roll), "pitch": float(pitch)}


app = Flask(__name__)

HTML = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MPU6050 - Cubo</title>
  <style>
    body { margin: 0; font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; background: #0b0f14; color: #e6edf3; }
    .topbar {
      position: fixed; left: 0; right: 0; top: 0;
      display: flex; gap: 16px; align-items: center;
      padding: 10px 14px; background: rgba(11,15,20,0.85); backdrop-filter: blur(6px);
      border-bottom: 1px solid rgba(230,237,243,0.12);
      z-index: 10;
    }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; font-size: 12px; color: rgba(230,237,243,0.85); }
    .kv { display: grid; grid-template-columns: 70px 1fr; gap: 6px 10px; }
    #viewport { position: fixed; left: 0; right: 0; top: 52px; bottom: 0; }
    a { color: #7aa2ff; text-decoration: none; }
  </style>
</head>
<body>
  <div class="topbar">
    <div>
      <div style="font-weight: 600;">MPU6050 - Cubo (roll/pitch por acelerómetro)</div>
      <div class="mono">JSON: <a href="/data">/data</a></div>
    </div>
    <div class="kv mono">
      <div>roll</div><div id="roll">-</div>
      <div>pitch</div><div id="pitch">-</div>
      <div>ax ay az</div><div id="acc">-</div>
    </div>
  </div>

  <div id="viewport"></div>

  <script src="https://unpkg.com/three@0.160.0/build/three.min.js"></script>
  <script>
    const container = document.getElementById('viewport');

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0f14);

    const camera = new THREE.PerspectiveCamera(60, 1, 0.1, 100);
    camera.position.set(0, 0.6, 2.2);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    // Luz
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambient);
    const dir = new THREE.DirectionalLight(0xffffff, 1.0);
    dir.position.set(2, 3, 2);
    scene.add(dir);

    // Cubo
    const geo = new THREE.BoxGeometry(1, 1, 1);
    const mat = new THREE.MeshStandardMaterial({ color: 0x4f86ff, roughness: 0.35, metalness: 0.15 });
    const cube = new THREE.Mesh(geo, mat);
    scene.add(cube);

    // Ejes
    const axes = new THREE.AxesHelper(1.4);
    scene.add(axes);

    // Rejilla
    const grid = new THREE.GridHelper(6, 12, 0x334155, 0x1f2937);
    grid.position.y = -0.8;
    scene.add(grid);

    function resize() {
      const w = container.clientWidth;
      const h = container.clientHeight;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    }
    window.addEventListener('resize', resize);
    resize();

    // Suavizado (filtro 1er orden) para que el cubo no “tiemble”
    let roll_f = 0.0;
    let pitch_f = 0.0;
    const ALPHA = 0.15; // 0..1 (más alto = menos filtrado)

    function applyTilt(roll, pitch) {
      // Convención:
      // - roll  alrededor de X
      // - pitch alrededor de Z o Y según el marco; aquí usamos Y para “inclinar hacia delante/atrás”
      // Ajusta signos si tu eje está invertido físicamente.
      roll_f = roll_f + ALPHA * (roll - roll_f);
      pitch_f = pitch_f + ALPHA * (pitch - pitch_f);

      // Three.js: rotación en radianes
      cube.rotation.x = roll_f;
      cube.rotation.y = pitch_f;
    }

    async function poll() {
      try {
        const r = await fetch('/data', { cache: 'no-store' });
        const d = await r.json();

        const roll = d.tilt.roll;
        const pitch = d.tilt.pitch;

        document.getElementById('roll').textContent = roll.toFixed(4) + " rad (" + (roll*180/Math.PI).toFixed(1) + "°)";
        document.getElementById('pitch').textContent = pitch.toFixed(4) + " rad (" + (pitch*180/Math.PI).toFixed(1) + "°)";
        document.getElementById('acc').textContent =
          d.accel.x.toFixed(3) + "  " + d.accel.y.toFixed(3) + "  " + d.accel.z.toFixed(3);

        applyTilt(roll, pitch);
      } catch (e) {
        // fallo puntual: ignorar
      } finally {
        setTimeout(poll, 30); // ~33 Hz de refresco web
      }
    }
    poll();

    function animate() {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    }
    animate();
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
    app.run(host="0.0.0.0", port=5000, debug=False)
