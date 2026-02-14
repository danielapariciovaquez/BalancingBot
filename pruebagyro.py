#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MPU6050 (GY-521) por I2C + Kalman 1D (pitch) + Web UI (Canvas) con dibujo mejorado
- Backend: Flask + Flask-SocketIO (eventlet)
- Frontend: Socket.IO servido desde el propio servidor (/socket.io/socket.io.js) => compatibilidad asegurada
- Dibujo: robot de autobalanceo 2 ruedas (péndulo invertido) rotando alrededor del eje de ruedas
"""

# ---------- IMPORTANTE: monkey_patch ANTES de Flask/SocketIO ----------
import eventlet
eventlet.monkey_patch()

import math
import time
import threading
import signal

from smbus2 import SMBus
from flask import Flask, render_template_string
from flask_socketio import SocketIO

# ============================================================
# CONFIG I2C / MPU6050
# ============================================================
I2C_BUS = 1
MPU_ADDR = 0x68  # GY-521 típico (AD0=0). Si AD0=1 -> 0x69

# Registros MPU6050
REG_PWR_MGMT_1   = 0x6B
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B

# Escalas
ACC_LSB_PER_G = 16384.0      # ±2g
GYRO_LSB_PER_DPS = 131.0     # ±250°/s

# Loop
LOOP_HZ = 200.0              # lectura IMU + Kalman
EMIT_HZ = 50.0               # envío a UI

# Calibración
GYRO_CALIB_SAMPLES = 1000    # robot quieto y vertical durante el arranque

# ============================================================
# Kalman 1D (ángulo + bias gyro)
# ============================================================
class Kalman1D:
    """
    Estado:
      angle (deg)
      bias  (deg/s)
    Entrada:
      new_rate (deg/s)
      dt (s)
      new_angle (deg) (medida por acelerómetro)
    """
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = float(Q_angle)
        self.Q_bias = float(Q_bias)
        self.R_measure = float(R_measure)

        self.angle = 0.0
        self.bias = 0.0

        # Matriz de covarianza P (2x2)
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

        # Predicción de covarianza
        self.P00 += dt * (dt*self.P11 - self.P01 - self.P10 + self.Q_angle)
        self.P01 -= dt * self.P11
        self.P10 -= dt * self.P11
        self.P11 += self.Q_bias * dt

        # Innovación
        S = self.P00 + self.R_measure
        if S <= 0.0:
            return self.angle

        K0 = self.P00 / S
        K1 = self.P10 / S

        y = new_angle - self.angle
        self.angle += K0 * y
        self.bias  += K1 * y

        # Actualización de P
        P00_temp = self.P00
        P01_temp = self.P01

        self.P00 -= K0 * P00_temp
        self.P01 -= K0 * P01_temp
        self.P10 -= K1 * P00_temp
        self.P11 -= K1 * P01_temp

        return self.angle

# ============================================================
# MPU6050 mínimo
# ============================================================
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

        # DLPF + sample rate
        # CONFIG: DLPF_CFG=3 => ~44Hz accel / 42Hz gyro (aprox)
        self._write(REG_CONFIG, 0x03)

        # SMPLRT_DIV: si base 1kHz (DLPF activo), 1kHz/(1+4)=200Hz
        self._write(REG_SMPLRT_DIV, 4)

        # Gyro ±250 dps
        self._write(REG_GYRO_CONFIG, 0x00)

        # Accel ±2g
        self._write(REG_ACCEL_CONFIG, 0x00)

    def read_accel_gyro(self):
        # 14 bytes: accel(6) + temp(2) + gyro(6)
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

# ============================================================
# Web UI (HTML embebido)
# ============================================================
HTML = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>BalancingBot IMU + Kalman</title>
  <style>
    body { font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial; margin: 18px; }
    .row { display: flex; gap: 18px; flex-wrap: wrap; align-items: flex-start; }
    canvas { border: 1px solid #cfcfcf; border-radius: 12px; background: #fff; }
    .card { border: 1px solid #ddd; border-radius: 12px; padding: 12px 14px; min-width: 340px; }
    .k { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; }
    .ctrl { display: grid; grid-template-columns: 120px 1fr 78px; gap: 10px; align-items: center; margin: 10px 0; }
    input[type="range"] { width: 100%; }
    .small { color: #666; font-size: 12px; margin-top: 6px; line-height: 1.25; }
    .badge { display:inline-block; padding: 2px 8px; border-radius: 999px; background:#f2f2f2; }
    hr { border: 0; border-top: 1px solid #eee; margin: 12px 0; }
  </style>
</head>
<body>
  <h2>MPU6050 (GY-521) + Kalman 1D + Visualización</h2>

  <div class="row">
    <canvas id="cv" width="700" height="480"></canvas>

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
        Pitch típico: accY/accZ y gyroX. Si tu IMU está orientada distinto, se cambia en el Python
        (buscar: "MAPEO EJES").
      </div>
    </div>
  </div>

  <!-- IMPORTANTE: JS de Socket.IO servido por Flask-SocketIO => versión compatible -->
  <script src="/socket.io/socket.io.js"></script>
  <script>
    const socket = io({ transports: ["websocket", "polling"] });

    const cv = document.getElementById("cv");
    const ctx = cv.getContext("2d");

    // Polyfill roundRect (por compat)
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

    function drawGrid(){
      // rejilla tenue
      ctx.save();
      ctx.globalAlpha = 0.12;
      for(let x=0;x<=cv.width;x+=40){
        ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,cv.height); ctx.stroke();
      }
      for(let y=0;y<=cv.height;y+=40){
        ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(cv.width,y); ctx.stroke();
      }
      ctx.restore();
    }

    function drawRobot(angleDeg){
      ctx.clearRect(0,0,cv.width,cv.height);
      drawGrid();

      // referencia suelo
      const groundY = 380;
      ctx.beginPath(); ctx.moveTo(40, groundY); ctx.lineTo(cv.width-40, groundY); ctx.stroke();

      // "pivote" físico del robot (eje ruedas) en canvas
      const axleX = cv.width*0.5;
      const axleY = groundY - 22;

      // geometría del robot (en su frame local)
      const wheelR = 28;
      const wheelBase = 170;        // distancia entre centros de ruedas
      const wheelX_L = -wheelBase/2;
      const wheelX_R = +wheelBase/2;
      const wheelY   = 0;

      const chassisW = 220;
      const chassisH = 90;
      const chassisX = -chassisW/2;
      const chassisY = -chassisH - 16;

      const mastW = 26;
      const mastH = 160;
      const mastX = -mastW/2;
      const mastY = chassisY - mastH + 10;

      const headR = 22;
      const headX = 0;
      const headY = mastY - 18;

      // Convención: angleDeg>0 => inclina hacia delante (elige visualmente)
      const a = angleDeg * Math.PI/180.0;

      // guía vertical (setpoint)
      ctx.save();
      ctx.globalAlpha = 0.35;
      ctx.setLineDash([6,6]);
      ctx.beginPath();
      ctx.moveTo(axleX, axleY-260);
      ctx.lineTo(axleX, axleY+40);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.restore();

      // Sombra en suelo (no rota)
      ctx.save();
      ctx.globalAlpha = 0.18;
      ctx.beginPath();
      ctx.ellipse(axleX, groundY+6, 150, 16, 0, 0, 2*Math.PI);
      ctx.fill();
      ctx.restore();

      // ROTACIÓN alrededor del eje de ruedas
      ctx.save();
      ctx.translate(axleX, axleY);
      ctx.rotate(-a);     // signo visual (si lo quieres al revés, cambia aquí)
      ctx.translate(-axleX, -axleY);

      // --- ruedas ---
      function wheel(cx, cy){
        // neumático
        ctx.beginPath();
        ctx.arc(cx, cy, wheelR, 0, 2*Math.PI);
        ctx.stroke();

        // llanta interior
        ctx.beginPath();
        ctx.arc(cx, cy, wheelR*0.62, 0, 2*Math.PI);
        ctx.stroke();

        // radios
        for(let k=0;k<6;k++){
          const th = k*(Math.PI/3);
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(cx + Math.cos(th)*wheelR*0.62, cy + Math.sin(th)*wheelR*0.62);
          ctx.stroke();
        }
      }

      // rueda izq
      wheel(axleX + wheelX_L, axleY + wheelY);
      // rueda der
      wheel(axleX + wheelX_R, axleY + wheelY);

      // eje (axle)
      ctx.beginPath();
      ctx.moveTo(axleX + wheelX_L + wheelR*0.2, axleY);
      ctx.lineTo(axleX + wheelX_R - wheelR*0.2, axleY);
      ctx.stroke();

      // --- chasis (caja) ---
      ctx.beginPath();
      ctx.roundRect(axleX + chassisX, axleY + chassisY, chassisW, chassisH, 14);
      ctx.stroke();

      // “batería”/masa interior (bloque)
      ctx.beginPath();
      ctx.roundRect(axleX - 60, axleY + chassisY + 18, 120, 44, 10);
      ctx.stroke();

      // “drivers” laterales (2 bloques)
      ctx.beginPath();
      ctx.roundRect(axleX + chassisX + 14, axleY + chassisY + 18, 44, 44, 8);
      ctx.stroke();
      ctx.beginPath();
      ctx.roundRect(axleX + chassisX + chassisW - 58, axleY + chassisY + 18, 44, 44, 8);
      ctx.stroke();

      // --- mástil ---
      ctx.beginPath();
      ctx.roundRect(axleX + mastX, axleY + mastY, mastW, mastH, 10);
      ctx.stroke();

      // manillar
      ctx.beginPath();
      ctx.moveTo(axleX - 55, axleY + mastY + 18);
      ctx.lineTo(axleX + 55, axleY + mastY + 18);
      ctx.stroke();
      ctx.beginPath();
      ctx.roundRect(axleX - 62, axleY + mastY + 10, 18, 16, 6);
      ctx.stroke();
      ctx.beginPath();
      ctx.roundRect(axleX + 44, axleY + mastY + 10, 18, 16, 6);
      ctx.stroke();

      // cabeza/masa superior
      ctx.beginPath();
      ctx.arc(axleX + headX, axleY + headY, headR, 0, 2*Math.PI);
      ctx.stroke();

      // vector gravedad en el frame rotado del robot (para visualizar error)
      ctx.save();
      ctx.globalAlpha = 0.65;
      ctx.beginPath();
      ctx.moveTo(axleX, axleY - 220);
      ctx.lineTo(axleX, axleY - 300);
      ctx.stroke();
      // flecha
      ctx.beginPath();
      ctx.moveTo(axleX - 6, axleY - 294);
      ctx.lineTo(axleX, axleY - 300);
      ctx.lineTo(axleX + 6, axleY - 294);
      ctx.stroke();
      ctx.restore();

      ctx.restore(); // fin rotación

      // texto
      ctx.fillText("angle = " + angleDeg.toFixed(2) + " deg", 40, 28);
      ctx.fillText("setpoint = 0.00 deg", 40, 48);
    }

    function bindSlider(id, decimals){
      const el = document.getElementById(id);
      const out = document.getElementById(id + "_v");
      function emit(){
        const v = parseFloat(el.value);
        out.textContent = v.toFixed(decimals);
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

    bindSlider("Q_angle", 4);
    bindSlider("Q_bias", 4);
    bindSlider("R_measure", 3);

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

    drawRobot(0);
  </script>
</body>
</html>
"""

# ============================================================
# Flask + SocketIO
# ============================================================
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
        q_bias  = float(msg.get("Q_bias", kalman.Q_bias))
        r_meas  = float(msg.get("R_measure", kalman.R_measure))
        inv     = bool(msg.get("invert_gyro", invert_gyro))
        kalman.set_params(Q_angle=q_angle, Q_bias=q_bias, R_measure=r_meas)
        invert_gyro = inv
    except Exception:
        # No reventar por payload malformado
        pass

# ============================================================
# Hilo IMU
# ============================================================
def imu_thread():
    global latest_angle, latest_rate, latest_dt

    mpu = MPU6050(I2C_BUS, MPU_ADDR)

    # ---------- Calibración gyro (robot quieto) ----------
    gx_sum = 0.0
    for _ in range(GYRO_CALIB_SAMPLES):
        _, _, _, gx, _, _ = mpu.read_accel_gyro()
        gx_sum += (gx / GYRO_LSB_PER_DPS)
        time.sleep(0.001)
    gyro_x_bias = gx_sum / float(GYRO_CALIB_SAMPLES)

    # ---------- Inicializa ángulo con acelerómetro ----------
    ax, ay, az, gx, gy, gz = mpu.read_accel_gyro()
    ax_g = ax / ACC_LSB_PER_G
    ay_g = ay / ACC_LSB_PER_G
    az_g = az / ACC_LSB_PER_G

    # ========= MAPEO EJES (pitch) =========
    # Por defecto: pitch = atan2(ay, az) ; rate = gyroX
    acc_angle_deg = math.degrees(math.atan2(ay_g, az_g))
    kalman.angle = acc_angle_deg

    last_t = time.perf_counter()

    emit_period = 1.0 / EMIT_HZ
    next_emit = last_t

    while not stop_event.is_set():
        t0 = time.perf_counter()
        dt = t0 - last_t
        if dt <= 0.0:
            dt = 1.0 / LOOP_HZ
        last_t = t0

        ax, ay, az, gx, gy, gz = mpu.read_accel_gyro()

        ax_g = ax / ACC_LSB_PER_G
        ay_g = ay / ACC_LSB_PER_G
        az_g = az / ACC_LSB_PER_G

        # ========= MAPEO EJES (pitch) =========
        acc_angle_deg = math.degrees(math.atan2(ay_g, az_g))

        gyro_x_dps = (gx / GYRO_LSB_PER_DPS) - gyro_x_bias
        if invert_gyro:
            gyro_x_dps = -gyro_x_dps

        angle_deg = kalman.update(acc_angle_deg, gyro_x_dps, dt)

        with state_lock:
            latest_angle = float(angle_deg)
            latest_rate = float(gyro_x_dps)
            latest_dt = float(dt)

        if t0 >= next_emit:
            socketio.emit("imu", {
                "angle_deg": latest_angle,
                "gyro_dps": latest_rate,
                "dt_s": latest_dt,
            })
            next_emit = t0 + emit_period

        # control de tasa
        elapsed = time.perf_counter() - t0
        sleep_s = (1.0 / LOOP_HZ) - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

    mpu.close()

# ============================================================
# Main / señales
# ============================================================
def shutdown_handler(signum, frame):
    stop_event.set()

def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    th = threading.Thread(target=imu_thread, daemon=True)
    th.start()

    # Servidor web (sin reloader para evitar doble ejecución)
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, use_reloader=False)

    stop_event.set()
    th.join(timeout=1.0)

if __name__ == "__main__":
    main()
