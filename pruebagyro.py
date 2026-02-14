#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
import signal
import sys

from smbus2 import SMBus
from flask import Flask, render_template_string
from flask_socketio import SocketIO

# =========================
# Config I2C / MPU6050
# =========================
I2C_BUS = 1
MPU_ADDR = 0x68  # GY-521 típico. Si AD0=1 -> 0x69

# Registros MPU6050
REG_PWR_MGMT_1   = 0x6B
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B

# Escalas elegidas
# Accel ±2g -> 16384 LSB/g
ACC_LSB_PER_G = 16384.0
# Gyro ±250°/s -> 131 LSB/(°/s)
GYRO_LSB_PER_DPS = 131.0

# Frecuencia de muestreo / envío
LOOP_HZ = 100.0
EMIT_HZ = 25.0

# Calibración de gyro al arranque
GYRO_CALIB_SAMPLES = 800

# =========================
# Kalman 1D (pitch)
# =========================
class Kalman1D:
    """
    Estado:
      angle (deg)
      bias  (deg/s)
    Entrada:
      new_rate (deg/s)
      dt (s)
      new_angle (deg) medido por acelerómetro
    """
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = float(Q_angle)
        self.Q_bias = float(Q_bias)
        self.R_measure = float(R_measure)

        self.angle = 0.0
        self.bias = 0.0
        self.P00 = 0.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 0.0

    def set_params(self, Q_angle=None, Q_bias=None, R_measure=None):
        if Q_angle is not None:
            self.Q_angle = float(Q_angle)
        if Q_bias is not None:
            self.Q_bias = float(Q_bias)
        if R_measure is not None:
            self.R_measure = float(R_measure)

    def update(self, new_angle, new_rate, dt):
        # Predicción
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Covarianza
        self.P00 += dt * (dt*self.P11 - self.P01 - self.P10 + self.Q_angle)
        self.P01 -= dt * self.P11
        self.P10 -= dt * self.P11
        self.P11 += self.Q_bias * dt

        # Innovación
        S = self.P00 + self.R_measure
        if S == 0.0:
            return self.angle

        K0 = self.P00 / S
        K1 = self.P10 / S

        y = new_angle - self.angle  # residual
        self.angle += K0 * y
        self.bias  += K1 * y

        # Actualiza P
        P00_temp = self.P00
        P01_temp = self.P01

        self.P00 -= K0 * P00_temp
        self.P01 -= K0 * P01_temp
        self.P10 -= K1 * P00_temp
        self.P11 -= K1 * P01_temp

        return self.angle

# =========================
# Driver MPU6050 mínimo
# =========================
def _to_int16(h, l):
    v = (h << 8) | l
    return v - 65536 if v & 0x8000 else v

class MPU6050:
    def __init__(self, bus_id=1, addr=0x68):
        self.bus = SMBus(bus_id)
        self.addr = addr
        self._init_device()

    def _write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read_block(self, reg, n):
        return self.bus.read_i2c_block_data(self.addr, reg, n)

    def _init_device(self):
        # Wake up
        self._write(REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

        # Sample rate: Gyro output / (1 + SMPLRT_DIV)
        # Gyro output rate = 8 kHz si DLPF desactivado, 1 kHz si DLPF activo.
        # Ponemos DLPF activo y sample 1 kHz/(1+9)=100 Hz aprox.
        self._write(REG_CONFIG, 0x03)       # DLPF cfg=3 (aprox 44 Hz accel / 42 Hz gyro)
        self._write(REG_SMPLRT_DIV, 9)      # ~100 Hz (si base 1 kHz)
        self._write(REG_GYRO_CONFIG, 0x00)  # ±250 dps
        self._write(REG_ACCEL_CONFIG, 0x00) # ±2g

    def read_accel_gyro(self):
        # Lee 14 bytes: accel(6) + temp(2) + gyro(6)
        b = self._read_block(REG_ACCEL_XOUT_H, 14)
        ax = _to_int16(b[0], b[1])
        ay = _to_int16(b[2], b[3])
        az = _to_int16(b[4], b[5])
        gx = _to_int16(b[8], b[9])
        gy = _to_int16(b[10], b[11])
        gz = _to_int16(b[12], b[13])
        return ax, ay, az, gx, gy, gz

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

# =========================
# Web UI
# =========================
HTML = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>BalancingBot IMU (Kalman)</title>
  <style>
    body { font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial; margin: 18px; }
    .row { display: flex; gap: 18px; flex-wrap: wrap; align-items: flex-start; }
    canvas { border: 1px solid #ccc; border-radius: 10px; background: #fff; }
    .card { border: 1px solid #ddd; border-radius: 12px; padding: 12px 14px; min-width: 320px; }
    .k { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; }
    .ctrl { display: grid; grid-template-columns: 120px 1fr 70px; gap: 10px; align-items: center; margin: 10px 0; }
    input[type="range"] { width: 100%; }
    .small { color: #666; font-size: 12px; margin-top: 6px; }
    .badge { display:inline-block; padding: 2px 8px; border-radius: 999px; background:#f2f2f2; }
  </style>
</head>
<body>
  <h2>IMU (MPU6050) + Kalman 1D + Visualización</h2>

  <div class="row">
    <canvas id="cv" width="520" height="420"></canvas>

    <div class="card">
      <div>Ángulo (Kalman): <span class="k" id="ang">0.00</span> deg</div>
      <div>Rate gyro: <span class="k" id="rate">0.00</span> deg/s</div>
      <div>dt: <span class="k" id="dt">0.0000</span> s</div>
      <div class="small">Estado: <span class="badge" id="st">conectando…</span></div>

      <hr/>

      <div class="ctrl">
        <label>Q_angle</label>
        <input id="Q_angle" type="range" min="0.0001" max="0.02" step="0.0001" value="0.001">
        <span class="k" id="Q_angle_v">0.0010</span>
      </div>

      <div class="ctrl">
        <label>Q_bias</label>
        <input id="Q_bias" type="range" min="0.0001" max="0.05" step="0.0001" value="0.003">
        <span class="k" id="Q_bias_v">0.0030</span>
      </div>

      <div class="ctrl">
        <label>R_measure</label>
        <input id="R_measure" type="range" min="0.001" max="0.2" step="0.001" value="0.03">
        <span class="k" id="R_measure_v">0.030</span>
      </div>

      <div class="ctrl" style="grid-template-columns: 160px 1fr;">
        <label>Invertir gyro</label>
        <input id="invert_gyro" type="checkbox">
      </div>

      <div class="small">
        Si tu IMU está montada con otro eje para pitch, cambia el mapeo en el Python (ver comentarios).
      </div>
    </div>
  </div>

  <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
  <script>
    const socket = io();

    const cv = document.getElementById("cv");
    const ctx = cv.getContext("2d");

    function drawRobot(angleDeg) {
      ctx.clearRect(0,0,cv.width,cv.height);

      // Suelo
      ctx.beginPath();
      ctx.moveTo(40, 340);
      ctx.lineTo(480, 340);
      ctx.stroke();

      // Centro del robot
      const cx = 260, cy = 300;

      // Convierte a radianes
      const a = angleDeg * Math.PI / 180.0;

      // Dibuja robot como chasis + ruedas, rotando alrededor de (cx,cy)
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(-a); // convención visual: positivo inclina a la derecha (ajústalo si quieres)
      ctx.translate(-cx, -cy);

      // Ruedas
      ctx.beginPath();
      ctx.arc(cx - 80, cy + 40, 26, 0, 2*Math.PI);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(cx + 80, cy + 40, 26, 0, 2*Math.PI);
      ctx.stroke();

      // Chasis
      ctx.beginPath();
      ctx.roundRect(cx - 120, cy - 40, 240, 110, 16);
      ctx.stroke();

      // “Mástil” superior (para que se note el balanceo)
      ctx.beginPath();
      ctx.roundRect(cx - 18, cy - 170, 36, 140, 12);
      ctx.stroke();

      // Cabeza/masa
      ctx.beginPath();
      ctx.arc(cx, cy - 185, 26, 0, 2*Math.PI);
      ctx.stroke();

      // Marca vertical del robot
      ctx.beginPath();
      ctx.moveTo(cx, cy - 185);
      ctx.lineTo(cx, cy + 10);
      ctx.stroke();

      ctx.restore();

      // Indicador ángulo
      ctx.fillText("angle = " + angleDeg.toFixed(2) + " deg", 40, 30);
    }

    // Polyfill roundRect (Chrome moderno ya lo trae, pero por si acaso)
    if (!CanvasRenderingContext2D.prototype.roundRect) {
      CanvasRenderingContext2D.prototype.roundRect = function(x,y,w,h,r){
        r = Math.min(r, w/2, h/2);
        this.beginPath();
        this.moveTo(x+r, y);
        this.arcTo(x+w, y, x+w, y+h, r);
        this.arcTo(x+w, y+h, x, y+h, r);
        this.arcTo(x, y+h, x, y, r);
        this.arcTo(x, y, x+w, y, r);
        this.closePath();
        return this;
      }
    }

    function bindSlider(id) {
      const el = document.getElementById(id);
      const out = document.getElementById(id + "_v");
      function emit() {
        const v = parseFloat(el.value);
        out.textContent = (id === "R_measure") ? v.toFixed(3) : v.toFixed(4);
        socket.emit("kalman_params", {
          Q_angle: parseFloat(document.getElementById("Q_angle").value),
          Q_bias: parseFloat(document.getElementById("Q_bias").value),
          R_measure: parseFloat(document.getElementById("R_measure").value),
          invert_gyro: document.getElementById("invert_gyro").checked
        });
      }
      el.addEventListener("input", emit);
      emit();
    }

    bindSlider("Q_angle");
    bindSlider("Q_bias");
    bindSlider("R_measure");

    document.getElementById("invert_gyro").addEventListener("change", () => {
      socket.emit("kalman_params", {
        Q_angle: parseFloat(document.getElementById("Q_angle").value),
        Q_bias: parseFloat(document.getElementById("Q_bias").value),
        R_measure: parseFloat(document.getElementById("R_measure").value),
        invert_gyro: document.getElementById("invert_gyro").checked
      });
    });

    socket.on("connect", () => {
      document.getElementById("st").textContent = "conectado";
    });

    socket.on("disconnect", () => {
      document.getElementById("st").textContent = "desconectado";
    });

    socket.on("imu", (msg) => {
      document.getElementById("ang").textContent = msg.angle_deg.toFixed(2);
      document.getElementById("rate").textContent = msg.gyro_dps.toFixed(2);
      document.getElementById("dt").textContent = msg.dt_s.toFixed(4);
      drawRobot(msg.angle_deg);
    });

    // primera pintura
    drawRobot(0);
  </script>
</body>
</html>
"""

# =========================
# App + estado compartido
# =========================
app = Flask(__name__)
socketio = SocketIO(app, async_mode="eventlet", cors_allowed_origins="*")

kalman = Kalman1D()
invert_gyro = False

state_lock = threading.Lock()
latest_angle = 0.0
latest_rate = 0.0
latest_dt = 0.0

stop_event = threading.Event()

@app.route("/")
def index():
    return render_template_string(HTML)

@socketio.on("kalman_params")
def on_kalman_params(msg):
    global invert_gyro
    try:
        q_angle = float(msg.get("Q_angle", kalman.Q_angle))
        q_bias = float(msg.get("Q_bias", kalman.Q_bias))
        r_meas = float(msg.get("R_measure", kalman.R_measure))
        inv = bool(msg.get("invert_gyro", invert_gyro))
        kalman.set_params(Q_angle=q_angle, Q_bias=q_bias, R_measure=r_meas)
        invert_gyro = inv
    except Exception:
        # No reventar la app por un mensaje malformado
        pass

def imu_thread():
    global latest_angle, latest_rate, latest_dt

    mpu = MPU6050(I2C_BUS, MPU_ADDR)

    # --- Calibración gyro X (robot quieto y vertical) ---
    gx_sum = 0.0
    for _ in range(GYRO_CALIB_SAMPLES):
        _, _, _, gx, _, _ = mpu.read_accel_gyro()
        gx_sum += (gx / GYRO_LSB_PER_DPS)
        time.sleep(0.001)
    gyro_x_bias = gx_sum / float(GYRO_CALIB_SAMPLES)

    # Inicializa ángulo con acelerómetro
    ax, ay, az, gx, gy, gz = mpu.read_accel_gyro()
    ay_g = ay / ACC_LSB_PER_G
    az_g = az / ACC_LSB_PER_G
    # Pitch desde accY/accZ (convención típica)
    acc_angle_deg = math.degrees(math.atan2(ay_g, az_g))
    kalman.angle = acc_angle_deg

    last_t = time.perf_counter()
    emit_period = 1.0 / EMIT_HZ
    next_emit = last_t

    while not stop_event.is_set():
        t = time.perf_counter()
        dt = t - last_t
        if dt <= 0.0:
            dt = 1.0 / LOOP_HZ
        last_t = t

        ax, ay, az, gx, gy, gz = mpu.read_accel_gyro()

        # Convierte a unidades físicas
        ay_g = ay / ACC_LSB_PER_G
        az_g = az / ACC_LSB_PER_G

        # Ángulo por acelerómetro (pitch)
        acc_angle_deg = math.degrees(math.atan2(ay_g, az_g))

        # Velocidad angular para pitch (gyro X)
        gyro_x_dps = (gx / GYRO_LSB_PER_DPS) - gyro_x_bias
        if invert_gyro:
            gyro_x_dps = -gyro_x_dps

        # Kalman
        angle_deg = kalman.update(acc_angle_deg, gyro_x_dps, dt)

        with state_lock:
            latest_angle = angle_deg
            latest_rate = gyro_x_dps
            latest_dt = dt

        # Emisión a web a EMIT_HZ
        if t >= next_emit:
            socketio.emit("imu", {
                "angle_deg": float(angle_deg),
                "gyro_dps": float(gyro_x_dps),
                "dt_s": float(dt),
            })
            next_emit = t + emit_period

        # Control de loop
        sleep_s = max(0.0, (1.0 / LOOP_HZ) - (time.perf_counter() - t))
        if sleep_s > 0:
            time.sleep(sleep_s)

    mpu.close()

def shutdown_handler(signum, frame):
    stop_event.set()

def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    th = threading.Thread(target=imu_thread, daemon=True)
    th.start()

    # Servidor web
    # Acceso desde LAN: http://IP_RASPBERRY:5000
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)

    stop_event.set()
    th.join(timeout=1.0)

if __name__ == "__main__":
    main()
