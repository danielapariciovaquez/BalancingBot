#!/usr/bin/env python3
import time
import sys
import math
import serial
import pygame
import threading
from dataclasses import dataclass

# ===================== RS485 / MOTORES =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05

ADDR_LEFT  = 0x01
ADDR_RIGHT = 0x02

MAX_RPM = 300
ACC = 255

INTER_FRAME_DELAY_S = 0.004
ENABLE_RETRIES = 2
ENABLE_RETRY_DELAY_S = 0.02

INVERT_LEFT  = False
INVERT_RIGHT = True
# ===========================================================

# ===================== LOOP =====================
UPDATE_HZ = 150
DT_MAX = 0.05
ANGLE_CUTOFF_DEG = 35.0

# Slew-rate para evitar latigazos (rpm/s)
MAX_RPM_STEP_PER_S = 700.0

# Limita el mando del balanceo (salida PID), muy importante para estabilidad
BAL_MAX_RPM = 120
# =========================================================

# ===================== JOYSTICK =====================
DEADZONE = 0.08
AXIS_THROTTLE = 1
AXIS_TURN     = 3

SETPOINT_DEG = 0.0
MAX_SETPOINT_OFFSET_DEG = 10.0  # “lean-to-go”
TURN_SCALE = 0.25              # limita mucho el giro (antes era muy sensible)
# =========================================================

# ===================== PID (por web) =====================
PID_INIT_KP = 18.0
PID_INIT_KI = 0.0
PID_INIT_KD = 0.9
I_LIM = 200.0
# =========================================================

# ===================== MPU6050 (GY-521) =====================
I2C_BUS = 1
MPU_ADDR = 0x68

REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_YOUT_H  = 0x45

ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

CAL_SAMPLES_GYRO  = 800
CAL_SAMPLES_ACCEL = 200

# Convención: al inclinar hacia adelante, el ángulo decrementa
INVERT_GYRO_RATE = False
# ===========================================================

# ===================== WEB UI =====================
WEB_HOST = "0.0.0.0"
WEB_PORT = 8000

KP_RANGE = (0.0, 80.0)
KI_RANGE = (0.0, 20.0)
KD_RANGE = (0.0, 20.0)

BAL_MAX_RPM_RANGE = (0.0, 300.0)
SLEW_RANGE = (0.0, 4000.0)
SETPOINT_OFF_RANGE = (0.0, 25.0)
TURN_SCALE_RANGE = (0.0, 1.0)

# ----- NUEVO: filtros IMU ajustables -----
# LPF 1er orden: tau en segundos. 0 => sin filtro
ACC_TAU_RANGE = (0.0, 0.30)   # filtro del ángulo de accel (antes de Kalman)
GYRO_TAU_RANGE = (0.0, 0.30)  # filtro del gyro_rate (antes de Kalman)

# Parámetros Kalman ajustables (típicos)
# Q_angle, Q_bias, R_measure
K_QA_RANGE = (1e-6, 5e-2)
K_QB_RANGE = (1e-6, 5e-1)
K_RM_RANGE = (1e-4, 2e-1)
# ===========================================================

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus  # type: ignore
    except ImportError:
        SMBus = None

try:
    from flask import Flask, request, jsonify
except ImportError:
    Flask = None


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def frame(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_enable(addr: int, en: bool) -> bytes:
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_speed(addr: int, rpm: int, acc: int) -> bytes:
    acc_u8 = int(clamp(acc, 0, 255))
    if rpm >= 0:
        direction_bit = 0
        speed = rpm
    else:
        direction_bit = 1
        speed = -rpm

    speed = int(clamp(speed, 0, MAX_RPM))
    b4 = ((direction_bit & 0x01) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    payload = bytes([b4, b5, acc_u8])
    return frame(addr, 0xF6, payload)

@dataclass
class MotorCmd:
    left_rpm: int
    right_rpm: int

def open_serial() -> serial.Serial:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT_S,
        write_timeout=TIMEOUT_S,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def send(ser: serial.Serial, data: bytes) -> None:
    ser.write(data)
    ser.flush()

def send_paced(ser: serial.Serial, data: bytes, delay_s: float = INTER_FRAME_DELAY_S) -> None:
    send(ser, data)
    if delay_s > 0:
        time.sleep(delay_s)

def send_enable_robust(ser: serial.Serial, addr: int, en: bool) -> None:
    pkt = cmd_enable(addr, en)
    for i in range(ENABLE_RETRIES):
        send_paced(ser, pkt, INTER_FRAME_DELAY_S)
        if i != ENABLE_RETRIES - 1:
            time.sleep(ENABLE_RETRY_DELAY_S)

def i2c_require():
    if SMBus is None:
        raise RuntimeError("Falta smbus2/smbus. Instala: pip3 install smbus2")

def web_require():
    if Flask is None:
        raise RuntimeError("Falta Flask. Instala: pip3 install flask")

def read_i16_be(bus: SMBus, addr: int, reg_hi: int) -> int:
    hi = bus.read_byte_data(addr, reg_hi)
    lo = bus.read_byte_data(addr, reg_hi + 1)
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v

def mpu_wake(bus: SMBus) -> None:
    bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.05)

def read_accel_gyro(bus: SMBus):
    ax = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H)
    az = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H + 4)
    gy = read_i16_be(bus, MPU_ADDR, REG_GYRO_YOUT_H)
    return ax, az, gy

def accel_angle_deg_from_ax_az(ax_raw: int, az_raw: int) -> float:
    ax_g = ax_raw / ACC_LSB_PER_G
    az_g = az_raw / ACC_LSB_PER_G
    ang = math.degrees(math.atan2(ax_g, az_g))
    return -ang  # forward tilt => decrement

def calibrate_gyro_y_bias(bus: SMBus) -> float:
    s = 0.0
    for _ in range(CAL_SAMPLES_GYRO):
        _, _, gy_raw = read_accel_gyro(bus)
        s += (gy_raw / GYRO_LSB_PER_DPS)
        time.sleep(0.001)
    return s / float(CAL_SAMPLES_GYRO)

def calibrate_accel_angle_zero(bus: SMBus) -> float:
    s = 0.0
    for _ in range(CAL_SAMPLES_ACCEL):
        ax, az, _ = read_accel_gyro(bus)
        s += accel_angle_deg_from_ax_az(ax, az)
        time.sleep(0.001)
    return s / float(CAL_SAMPLES_ACCEL)

def motors_from_balance(base_rpm: float, turn_rpm: float) -> MotorCmd:
    left = base_rpm - turn_rpm
    right = base_rpm + turn_rpm
    left_rpm = int(round(clamp(left, -MAX_RPM, MAX_RPM)))
    right_rpm = int(round(clamp(right, -MAX_RPM, MAX_RPM)))
    if INVERT_LEFT:
        left_rpm = -left_rpm
    if INVERT_RIGHT:
        right_rpm = -right_rpm
    return MotorCmd(left_rpm=left_rpm, right_rpm=right_rpm)


# ===================== Kalman 1D =====================
class Kalman1D:
    """
    Kalman 1D típico para ángulo con bias de gyro:
      state: [angle, bias]
      input: gyro_rate (deg/s)
      measurement: accel_angle (deg)
    """
    def __init__(self, q_angle=0.001, q_bias=0.003, r_measure=0.03):
        self.q_angle = float(q_angle)
        self.q_bias = float(q_bias)
        self.r_measure = float(r_measure)

        self.angle = 0.0
        self.bias = 0.0
        self.P00 = 1.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 1.0

    def set_angle(self, angle_deg: float):
        self.angle = float(angle_deg)

    def set_params(self, q_angle: float, q_bias: float, r_measure: float):
        self.q_angle = float(q_angle)
        self.q_bias = float(q_bias)
        self.r_measure = float(r_measure)

    def update(self, meas_angle_deg: float, gyro_rate_dps: float, dt: float) -> float:
        # Predict
        rate = gyro_rate_dps - self.bias
        self.angle += dt * rate

        # Covariance predict
        P00 = self.P00 + dt * (dt*self.P11 - self.P01 - self.P10 + self.q_angle)
        P01 = self.P01 - dt * self.P11
        P10 = self.P10 - dt * self.P11
        P11 = self.P11 + self.q_bias * dt
        self.P00, self.P01, self.P10, self.P11 = P00, P01, P10, P11

        # Update
        y = meas_angle_deg - self.angle
        S = self.P00 + self.r_measure
        K0 = self.P00 / S
        K1 = self.P10 / S

        self.angle += K0 * y
        self.bias  += K1 * y

        P00 = self.P00 - K0 * self.P00
        P01 = self.P01 - K0 * self.P01
        P10 = self.P10 - K1 * self.P00
        P11 = self.P11 - K1 * self.P01
        self.P00, self.P01, self.P10, self.P11 = P00, P01, P10, P11
        return self.angle


# ===================== Shared params (web <-> loop) =====================
class Params:
    def __init__(self):
        self.kp = PID_INIT_KP
        self.ki = PID_INIT_KI
        self.kd = PID_INIT_KD

        self.i_lim = I_LIM
        self.bal_max_rpm = BAL_MAX_RPM
        self.max_rpm_step = MAX_RPM_STEP_PER_S
        self.max_setpoint_offset = MAX_SETPOINT_OFFSET_DEG
        self.turn_scale = TURN_SCALE

        # NUEVO: filtros IMU
        self.acc_tau = 0.03   # s (LPF ángulo accel)
        self.gyro_tau = 0.02  # s (LPF gyro_rate)

        # NUEVO: params Kalman
        self.k_q_angle = 0.001
        self.k_q_bias = 0.003
        self.k_r_meas = 0.03

        self.invert_gyro = INVERT_GYRO_RATE

param_lock = threading.Lock()
params = Params()

def get_params():
    with param_lock:
        return {
            "kp": params.kp,
            "ki": params.ki,
            "kd": params.kd,
            "i_lim": params.i_lim,
            "bal_max_rpm": params.bal_max_rpm,
            "max_rpm_step": params.max_rpm_step,
            "max_setpoint_offset": params.max_setpoint_offset,
            "turn_scale": params.turn_scale,
            "acc_tau": params.acc_tau,
            "gyro_tau": params.gyro_tau,
            "k_q_angle": params.k_q_angle,
            "k_q_bias": params.k_q_bias,
            "k_r_meas": params.k_r_meas,
            "invert_gyro": params.invert_gyro,
        }

def set_params(d: dict):
    with param_lock:
        if "kp" in d: params.kp = float(d["kp"])
        if "ki" in d: params.ki = float(d["ki"])
        if "kd" in d: params.kd = float(d["kd"])
        if "i_lim" in d: params.i_lim = float(d["i_lim"])
        if "bal_max_rpm" in d: params.bal_max_rpm = float(d["bal_max_rpm"])
        if "max_rpm_step" in d: params.max_rpm_step = float(d["max_rpm_step"])
        if "max_setpoint_offset" in d: params.max_setpoint_offset = float(d["max_setpoint_offset"])
        if "turn_scale" in d: params.turn_scale = float(d["turn_scale"])
        if "acc_tau" in d: params.acc_tau = float(d["acc_tau"])
        if "gyro_tau" in d: params.gyro_tau = float(d["gyro_tau"])
        if "k_q_angle" in d: params.k_q_angle = float(d["k_q_angle"])
        if "k_q_bias" in d: params.k_q_bias = float(d["k_q_bias"])
        if "k_r_meas" in d: params.k_r_meas = float(d["k_r_meas"])
        if "invert_gyro" in d: params.invert_gyro = bool(d["invert_gyro"])


# ===================== Web server =====================
def start_web_server():
    web_require()
    app = Flask(__name__)

    HTML = f"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Balance PID + Filtros IMU</title>
  <style>
    body {{ font-family: sans-serif; margin: 20px; max-width: 920px; }}
    .row {{ margin: 16px 0; }}
    .label {{ display:flex; justify-content:space-between; margin-bottom: 6px; }}
    input[type=range] {{ width: 100%; }}
    .box {{ padding: 12px; border: 1px solid #ddd; border-radius: 12px; }}
    .small {{ color:#555; font-size: 0.95em; line-height: 1.35; }}
    code {{ background:#f6f6f6; padding:2px 6px; border-radius:6px; }}
    h2 {{ margin-bottom: 8px; }}
    h3 {{ margin: 16px 0 6px; }}
  </style>
</head>
<body>
  <h2>Self-Balancing: PID + Filtros IMU (Kalman)</h2>
  <div class="box">
    <h3>PID</h3>
    <div class="row">
      <div class="label"><b>Kp</b><span id="kpv"></span></div>
      <input id="kp" type="range" min="{KP_RANGE[0]}" max="{KP_RANGE[1]}" step="0.1">
    </div>
    <div class="row">
      <div class="label"><b>Ki</b><span id="kiv"></span></div>
      <input id="ki" type="range" min="{KI_RANGE[0]}" max="{KI_RANGE[1]}" step="0.01">
    </div>
    <div class="row">
      <div class="label"><b>Kd</b><span id="kdv"></span></div>
      <input id="kd" type="range" min="{KD_RANGE[0]}" max="{KD_RANGE[1]}" step="0.1">
    </div>

    <h3>Estabilidad</h3>
    <div class="row">
      <div class="label"><b>BAL_MAX_RPM</b><span id="balv"></span></div>
      <input id="bal" type="range" min="{BAL_MAX_RPM_RANGE[0]}" max="{BAL_MAX_RPM_RANGE[1]}" step="1">
    </div>
    <div class="row">
      <div class="label"><b>Slew rpm/s</b><span id="slewv"></span></div>
      <input id="slew" type="range" min="{SLEW_RANGE[0]}" max="{SLEW_RANGE[1]}" step="10">
    </div>
    <div class="row">
      <div class="label"><b>MAX_SETPOINT_OFFSET (deg)</b><span id="spoffv"></span></div>
      <input id="spoff" type="range" min="{SETPOINT_OFF_RANGE[0]}" max="{SETPOINT_OFF_RANGE[1]}" step="0.5">
    </div>
    <div class="row">
      <div class="label"><b>TURN_SCALE</b><span id="turnv"></span></div>
      <input id="turn" type="range" min="{TURN_SCALE_RANGE[0]}" max="{TURN_SCALE_RANGE[1]}" step="0.01">
    </div>

    <h3>Filtros IMU (pre-Kalman)</h3>
    <div class="row">
      <div class="label"><b>ACC_TAU (s)</b><span id="acctauv"></span></div>
      <input id="acctau" type="range" min="{ACC_TAU_RANGE[0]}" max="{ACC_TAU_RANGE[1]}" step="0.005">
    </div>
    <div class="row">
      <div class="label"><b>GYRO_TAU (s)</b><span id="gyrotauv"></span></div>
      <input id="gyrotau" type="range" min="{GYRO_TAU_RANGE[0]}" max="{GYRO_TAU_RANGE[1]}" step="0.005">
    </div>

    <h3>Kalman</h3>
    <div class="row">
      <div class="label"><b>Q_angle</b><span id="qav"></span></div>
      <input id="qa" type="range" min="{K_QA_RANGE[0]}" max="{K_QA_RANGE[1]}" step="0.000001">
    </div>
    <div class="row">
      <div class="label"><b>Q_bias</b><span id="qbv"></span></div>
      <input id="qb" type="range" min="{K_QB_RANGE[0]}" max="{K_QB_RANGE[1]}" step="0.000001">
    </div>
    <div class="row">
      <div class="label"><b>R_measure</b><span id="rmv"></span></div>
      <input id="rm" type="range" min="{K_RM_RANGE[0]}" max="{K_RM_RANGE[1]}" step="0.0001">
    </div>

    <div class="row">
      <label><input id="invgyro" type="checkbox"> Invertir signo gyro_rate</label>
    </div>

    <div class="small">
      **Cambio pedido**: se ha quitado el filtro de derivada <code>D_TAU</code>. La derivada usa <code>derr</code> directa.
      <br/>Aquí ajustas el filtrado IMU: <code>ACC_TAU</code> y <code>GYRO_TAU</code> (LPF 1er orden) y los parámetros del Kalman.
    </div>
  </div>

<script>
async function getP() {{
  const r = await fetch('/params');
  return await r.json();
}}
async function setP(p) {{
  await fetch('/params', {{
    method: 'POST',
    headers: {{'Content-Type':'application/json'}},
    body: JSON.stringify(p)
  }});
}}
function bindSlider(id, labId, fmt) {{
  const s = document.getElementById(id);
  const l = document.getElementById(labId);
  const upd = () => l.textContent = fmt(parseFloat(s.value));
  s.addEventListener('input', upd);
  return {{slider:s, updateLabel:upd}};
}}

(async () => {{
  const p = await getP();

  const kp = bindSlider('kp','kpv',v=>v.toFixed(1));
  const ki = bindSlider('ki','kiv',v=>v.toFixed(2));
  const kd = bindSlider('kd','kdv',v=>v.toFixed(2));

  const bal = bindSlider('bal','balv',v=>v.toFixed(0));
  const slew = bindSlider('slew','slewv',v=>v.toFixed(0));
  const spoff = bindSlider('spoff','spoffv',v=>v.toFixed(1));
  const turn = bindSlider('turn','turnv',v=>v.toFixed(2));

  const acctau = bindSlider('acctau','acctauv',v=>v.toFixed(3));
  const gyrotau = bindSlider('gyrotau','gyrotauv',v=>v.toFixed(3));

  const qa = bindSlider('qa','qav',v=>v.toExponential(3));
  const qb = bindSlider('qb','qbv',v=>v.toExponential(3));
  const rm = bindSlider('rm','rmv',v=>v.toExponential(3));

  const invgyro = document.getElementById('invgyro');

  kp.slider.value = p.kp; kp.updateLabel();
  ki.slider.value = p.ki; ki.updateLabel();
  kd.slider.value = p.kd; kd.updateLabel();

  bal.slider.value = p.bal_max_rpm; bal.updateLabel();
  slew.slider.value = p.max_rpm_step; slew.updateLabel();
  spoff.slider.value = p.max_setpoint_offset; spoff.updateLabel();
  turn.slider.value = p.turn_scale; turn.updateLabel();

  acctau.slider.value = p.acc_tau; acctau.updateLabel();
  gyrotau.slider.value = p.gyro_tau; gyrotau.updateLabel();

  qa.slider.value = p.k_q_angle; qa.updateLabel();
  qb.slider.value = p.k_q_bias; qb.updateLabel();
  rm.slider.value = p.k_r_meas; rm.updateLabel();

  invgyro.checked = !!p.invert_gyro;

  let t = null;
  const push = () => {{
    clearTimeout(t);
    t = setTimeout(() => {{
      setP({{
        kp: parseFloat(kp.slider.value),
        ki: parseFloat(ki.slider.value),
        kd: parseFloat(kd.slider.value),
        bal_max_rpm: parseFloat(bal.slider.value),
        max_rpm_step: parseFloat(slew.slider.value),
        max_setpoint_offset: parseFloat(spoff.slider.value),
        turn_scale: parseFloat(turn.slider.value),
        acc_tau: parseFloat(acctau.slider.value),
        gyro_tau: parseFloat(gyrotau.slider.value),
        k_q_angle: parseFloat(qa.slider.value),
        k_q_bias: parseFloat(qb.slider.value),
        k_r_meas: parseFloat(rm.slider.value),
        invert_gyro: invgyro.checked
      }});
    }}, 120);
  }};

  for (const s of [kp.slider,ki.slider,kd.slider,bal.slider,slew.slider,spoff.slider,turn.slider,acctau.slider,gyrotau.slider,qa.slider,qb.slider,rm.slider]) {{
    s.addEventListener('input', push);
  }}
  invgyro.addEventListener('change', push);
}})();
</script>
</body>
</html>
"""

    @app.get("/")
    def index():
        return HTML

    @app.get("/params")
    def params_get():
        return jsonify(get_params())

    @app.post("/params")
    def params_post():
        d = request.get_json(force=True, silent=True) or {}

        def fclamp(name, lo, hi):
            if name in d:
                try:
                    d[name] = float(d[name])
                except Exception:
                    d.pop(name, None)
                    return
                d[name] = clamp(d[name], lo, hi)

        fclamp("kp", *KP_RANGE)
        fclamp("ki", *KI_RANGE)
        fclamp("kd", *KD_RANGE)
        fclamp("bal_max_rpm", *BAL_MAX_RPM_RANGE)
        fclamp("max_rpm_step", *SLEW_RANGE)
        fclamp("max_setpoint_offset", *SETPOINT_OFF_RANGE)
        fclamp("turn_scale", *TURN_SCALE_RANGE)
        fclamp("acc_tau", *ACC_TAU_RANGE)
        fclamp("gyro_tau", *GYRO_TAU_RANGE)
        fclamp("k_q_angle", *K_QA_RANGE)
        fclamp("k_q_bias", *K_QB_RANGE)
        fclamp("k_r_meas", *K_RM_RANGE)

        if "invert_gyro" in d:
            d["invert_gyro"] = bool(d["invert_gyro"])

        set_params(d)
        return jsonify({"ok": True, **get_params()})

    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)


# ===================== Helper LPF =====================
def lpf_update(x_f: float, x: float, tau: float, dt: float) -> float:
    if tau <= 0.0 or dt <= 0.0:
        return x
    a = dt / (tau + dt)
    return x_f + a * (x - x_f)


# ===================== main =====================
def main() -> int:
    i2c_require()
    if Flask is None:
        print("ERROR: Flask no instalado. Instala: pip3 install flask", file=sys.stderr)
        return 1

    th = threading.Thread(target=start_web_server, daemon=True)
    th.start()
    print(f"Web: http://<IP_RPI>:{WEB_PORT}/")

    bus = SMBus(I2C_BUS)
    mpu_wake(bus)

    print("Calibrando IMU (quieto y en posición de equilibrio)...")
    gyro_y_bias_dps = calibrate_gyro_y_bias(bus)
    accel_zero_deg = calibrate_accel_angle_zero(bus)

    kf = Kalman1D(params.k_q_angle, params.k_q_bias, params.k_r_meas)
    kf.set_angle(0.0)

    # filtros pre-Kalman
    accel_angle_f = 0.0
    gyro_rate_f = 0.0

    print(f"OK. BiasGy={gyro_y_bias_dps:.6f} °/s | AccZero={accel_zero_deg:.6f} ° | Ang0=0.000°")

    ser = open_serial()
    send_enable_robust(ser, ADDR_LEFT, True)
    send_enable_robust(ser, ADDR_RIGHT, True)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() < 1:
        print("ERROR: no hay joystick detectado por pygame.", file=sys.stderr)
        return 2
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Joystick: {joy.get_name()} | axes={joy.get_numaxes()} buttons={joy.get_numbuttons()}")

    integ = 0.0
    prev_err = 0.0
    base_rpm_cmd = 0.0

    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()
    t_prev = t_next

    try:
        while True:
            pygame.event.pump()

            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            if dt < 0.0:
                dt = 0.0
            elif dt > DT_MAX:
                dt = DT_MAX

            p = get_params()
            # aplica parámetros Kalman en caliente
            kf.set_params(p["k_q_angle"], p["k_q_bias"], p["k_r_meas"])

            # IMU
            ax, az, gy_raw = read_accel_gyro(bus)
            accel_angle = accel_angle_deg_from_ax_az(ax, az) - accel_zero_deg
            gyro_rate = (gy_raw / GYRO_LSB_PER_DPS) - gyro_y_bias_dps
            if p["invert_gyro"]:
                gyro_rate = -gyro_rate

            # NUEVO: filtros de señal ajustables (pre-Kalman)
            accel_angle_f = lpf_update(accel_angle_f, accel_angle, p["acc_tau"], dt)
            gyro_rate_f = lpf_update(gyro_rate_f, gyro_rate, p["gyro_tau"], dt)

            angle = kf.update(accel_angle_f, gyro_rate_f, dt)

            # Safety
            if abs(angle) > ANGLE_CUTOFF_DEG:
                send_paced(ser, cmd_speed(ADDR_LEFT, 0, 0))
                send_paced(ser, cmd_speed(ADDR_RIGHT, 0, 0))
                integ = 0.0
                prev_err = 0.0
                base_rpm_cmd = 0.0
                sys.stdout.write(f"\rCAIDO: Ang={angle:+07.2f} deg -> motores 0 rpm                           ")
                sys.stdout.flush()

                t_next += period
                sleep_s = t_next - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    t_next = time.monotonic()
                continue

            # Joystick -> setpoint y giro (limitado)
            thr = -joy.get_axis(AXIS_THROTTLE)
            trn = joy.get_axis(AXIS_TURN)
            thr = 0.0 if abs(thr) < DEADZONE else thr
            trn = 0.0 if abs(trn) < DEADZONE else trn
            thr = clamp(thr, -1.0, 1.0)
            trn = clamp(trn, -1.0, 1.0)

            setpoint = SETPOINT_DEG + thr * p["max_setpoint_offset"]
            turn_rpm = trn * MAX_RPM * p["turn_scale"]

            # PID (SIN filtro D_TAU: derr directa)
            err = setpoint - angle
            integ += err * dt
            integ = clamp(integ, -p["i_lim"], +p["i_lim"])

            derr = (err - prev_err) / dt if dt > 0 else 0.0
            prev_err = err

            base_rpm = (p["kp"] * err) + (p["ki"] * integ) + (p["kd"] * derr)
            base_rpm = clamp(base_rpm, -p["bal_max_rpm"], +p["bal_max_rpm"])

            # Slew-rate
            max_step = p["max_rpm_step"] * dt
            delta = clamp(base_rpm - base_rpm_cmd, -max_step, +max_step)
            base_rpm_cmd += delta
            base_rpm_cmd = clamp(base_rpm_cmd, -p["bal_max_rpm"], +p["bal_max_rpm"])

            mc = motors_from_balance(base_rpm_cmd, turn_rpm)

            send_paced(ser, cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
            send_paced(ser, cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

            sys.stdout.write(
                f"\rAng={angle:+07.2f} | AccF={accel_angle_f:+07.2f} | GyF={gyro_rate_f:+07.2f} | "
                f"SP={setpoint:+06.2f} | base={base_rpm_cmd:+06.1f} | turn={turn_rpm:+06.1f} | "
                f"PID({p['kp']:4.1f},{p['ki']:4.2f},{p['kd']:4.2f}) | "
                f"ACC_TAU={p['acc_tau']:.3f} GY_TAU={p['gyro_tau']:.3f} | "
                f"K(qA={p['k_q_angle']:.4g},qB={p['k_q_bias']:.4g},rM={p['k_r_meas']:.4g}) | "
                f"L={mc.left_rpm:+04d} R={mc.right_rpm:+04d}     "
            )
            sys.stdout.flush()

            t_next += period
            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                t_next = time.monotonic()

    except KeyboardInterrupt:
        print("\nCTRL+C -> Parando motores...")

    finally:
        try:
            send_paced(ser, cmd_speed(ADDR_LEFT, 0, 0))
            send_paced(ser, cmd_speed(ADDR_RIGHT, 0, 0))
            time.sleep(0.05)
            send_enable_robust(ser, ADDR_LEFT, False)
            send_enable_robust(ser, ADDR_RIGHT, False)
        except Exception as e:
            print(f"WARNING: error al parar/disable: {e}", file=sys.stderr)

        try:
            ser.close()
        except Exception:
            pass
        try:
            bus.close()
        except Exception:
            pass
        pygame.quit()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
