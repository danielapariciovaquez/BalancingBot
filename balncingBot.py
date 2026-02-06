#!/usr/bin/env python3
import time
import sys
import math
import serial
import pygame
import threading
from dataclasses import dataclass

# ===================== CONFIG (RS485 + JOYSTICK) =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05

ADDR_LEFT  = 0x01
ADDR_RIGHT = 0x02

MAX_RPM = 30
ACC = 255

DEADZONE = 0.08
UPDATE_HZ = 100

AXIS_THROTTLE = 1
AXIS_TURN     = 3

INVERT_LEFT  = False
INVERT_RIGHT = True

# RS485: separa tramas para que enable/disable y F6 sean más fiables
INTER_FRAME_DELAY_S = 0.004
ENABLE_RETRIES = 2
ENABLE_RETRY_DELAY_S = 0.02
# ====================================================================

# ===================== CONFIG (GY-521 / MPU6050) =====================
I2C_BUS = 1
MPU_ADDR = 0x68  # AD0=0 típico; si AD0=1 -> 0x69

REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_YOUT_H  = 0x45

ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

CAL_SAMPLES_GYRO = 800
CAL_SAMPLES_ACCEL = 200

ANGLE_CUTOFF_DEG = 35.0
DT_MAX = 0.05
# ====================================================================

# ===================== BALANCE CONTROL (PID + estabilidad) =====================
SETPOINT_DEG = 0.0
MAX_SETPOINT_OFFSET_DEG = 10.0

# PID por defecto (se ajusta por web)
Kp = 18.0
Ki = 0.0
Kd = 0.9

I_LIM = 200.0

# Limita la salida del balanceo (muy importante para estabilidad)
BAL_MAX_RPM = 120

# Filtro derivativo: reduce vibración / ruido (derivada del error)
D_TAU = 0.05  # s

# Slew-rate: limita cambios bruscos de consigna (rpm/s)
MAX_RPM_STEP_PER_S = 700.0

# Invertir signo del gyro si fuese necesario para coherencia con el ángulo
INVERT_GYRO = False
# ====================================================================

# ===================== WEB UI =====================
WEB_HOST = "0.0.0.0"
WEB_PORT = 8000
# Rangos sliders (en caliente)
KP_RANGE = (0.0, 80.0)
KI_RANGE = (0.0, 20.0)
KD_RANGE = (0.0, 20.0)

BAL_MAX_RPM_RANGE = (0.0, 300.0)
MAX_RPM_STEP_RANGE = (0.0, 4000.0)     # rpm/s
D_TAU_RANGE = (0.0, 0.30)              # s
SETPOINT_OFF_RANGE = (0.0, 25.0)       # deg
# ====================================================================

# --- I2C backend (smbus2 preferente, si no smbus) ---
try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus  # type: ignore
    except ImportError:
        SMBus = None

# --- Web backend ---
try:
    from flask import Flask, request, jsonify
except ImportError:
    Flask = None


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def dz(x: float, dead: float) -> float:
    return 0.0 if abs(x) < dead else x

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
        raise RuntimeError(
            "No se encontró smbus2/smbus. Instala:\n"
            "  pip3 install smbus2\n"
            "y habilita i2c (raspi-config) + permisos /dev/i2c-1."
        )

def web_require():
    if Flask is None:
        raise RuntimeError("No se encontró Flask. Instala: pip3 install flask")

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
    """
    Pitch derivado de acelerómetro usando AX y AZ.
    Requisito: al inclinar hacia adelante, el ángulo DECREMENTA.
    """
    ax_g = ax_raw / ACC_LSB_PER_G
    az_g = az_raw / ACC_LSB_PER_G
    ang = math.degrees(math.atan2(ax_g, az_g))
    return -ang

class Kalman1D:
    """
    Kalman 1D típico para ángulo con bias de gyro.
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

    def update(self, meas_angle_deg: float, gyro_rate_dps: float, dt: float) -> float:
        rate = gyro_rate_dps - self.bias
        self.angle += dt * rate

        P00 = self.P00 + dt * (dt*self.P11 - self.P01 - self.P10 + self.q_angle)
        P01 = self.P01 - dt * self.P11
        P10 = self.P10 - dt * self.P11
        P11 = self.P11 + self.q_bias * dt
        self.P00, self.P01, self.P10, self.P11 = P00, P01, P10, P11

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

# ===================== SHARED PARAMS (web <-> control loop) =====================
param_lock = threading.Lock()

class Params:
    def __init__(self):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.i_lim = I_LIM
        self.bal_max_rpm = BAL_MAX_RPM
        self.max_rpm_step = MAX_RPM_STEP_PER_S
        self.d_tau = D_TAU
        self.max_setpoint_offset = MAX_SETPOINT_OFFSET_DEG
        self.invert_gyro = INVERT_GYRO

params = Params()

def get_params_dict():
    with param_lock:
        return {
            "kp": params.kp,
            "ki": params.ki,
            "kd": params.kd,
            "i_lim": params.i_lim,
            "bal_max_rpm": params.bal_max_rpm,
            "max_rpm_step": params.max_rpm_step,
            "d_tau": params.d_tau,
            "max_setpoint_offset": params.max_setpoint_offset,
            "invert_gyro": params.invert_gyro,
        }

def set_params_from_dict(d: dict):
    with param_lock:
        if "kp" in d: params.kp = float(d["kp"])
        if "ki" in d: params.ki = float(d["ki"])
        if "kd" in d: params.kd = float(d["kd"])
        if "i_lim" in d: params.i_lim = float(d["i_lim"])
        if "bal_max_rpm" in d: params.bal_max_rpm = float(d["bal_max_rpm"])
        if "max_rpm_step" in d: params.max_rpm_step = float(d["max_rpm_step"])
        if "d_tau" in d: params.d_tau = float(d["d_tau"])
        if "max_setpoint_offset" in d: params.max_setpoint_offset = float(d["max_setpoint_offset"])
        if "invert_gyro" in d: params.invert_gyro = bool(d["invert_gyro"])

def start_web_server():
    web_require()
    app = Flask(__name__)

    HTML = f"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Balance PID + Estabilidad</title>
  <style>
    body {{ font-family: sans-serif; margin: 18px; max-width: 860px; }}
    .row {{ margin: 14px 0; }}
    .label {{ display:flex; justify-content:space-between; margin-bottom: 6px; }}
    input[type=range] {{ width: 100%; }}
    .box {{ padding: 12px; border: 1px solid #ddd; border-radius: 12px; }}
    .small {{ color:#555; font-size: 0.95em; line-height: 1.35; }}
    code {{ background:#f6f6f6; padding:2px 6px; border-radius:6px; }}
    h2 {{ margin-bottom: 6px; }}
    h3 {{ margin: 14px 0 6px; }}
  </style>
</head>
<body>
  <h2>Self-Balancing: PID + parámetros de estabilidad</h2>
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
      <div class="label"><b>Slew-rate (MAX_RPM_STEP_PER_S)</b><span id="slewv"></span></div>
      <input id="slew" type="range" min="{MAX_RPM_STEP_RANGE[0]}" max="{MAX_RPM_STEP_RANGE[1]}" step="10">
    </div>
    <div class="row">
      <div class="label"><b>D_TAU (filtro derivada)</b><span id="dtauv"></span></div>
      <input id="dtau" type="range" min="{D_TAU_RANGE[0]}" max="{D_TAU_RANGE[1]}" step="0.005">
    </div>
    <div class="row">
      <div class="label"><b>MAX_SETPOINT_OFFSET_DEG</b><span id="spoffv"></span></div>
      <input id="spoff" type="range" min="{SETPOINT_OFF_RANGE[0]}" max="{SETPOINT_OFF_RANGE[1]}" step="0.5">
    </div>
    <div class="row">
      <label><input id="invgyro" type="checkbox"> Invertir signo gyro_rate</label>
    </div>

    <div class="small">
      API: <code>GET /params</code>, <code>POST /params</code> (JSON).
      <br/>Recomendación: ajusta primero <code>BAL_MAX_RPM</code> y <code>Slew-rate</code> para evitar latigazos,
      luego <code>Kp</code> y <code>Kd</code>. Usa <code>Ki=0</code> hasta que esté estable.
    </div>
  </div>

<script>
async function getParams() {{
  const r = await fetch('/params');
  return await r.json();
}}
async function setParams(p) {{
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
  const p = await getParams();

  const kp = bindSlider('kp','kpv',v=>v.toFixed(1));
  const ki = bindSlider('ki','kiv',v=>v.toFixed(2));
  const kd = bindSlider('kd','kdv',v=>v.toFixed(1));

  const bal = bindSlider('bal','balv',v=>v.toFixed(0));
  const slew = bindSlider('slew','slewv',v=>v.toFixed(0));
  const dtau = bindSlider('dtau','dtauv',v=>v.toFixed(3));
  const spoff = bindSlider('spoff','spoffv',v=>v.toFixed(1));

  const invgyro = document.getElementById('invgyro');

  kp.slider.value = p.kp; kp.updateLabel();
  ki.slider.value = p.ki; ki.updateLabel();
  kd.slider.value = p.kd; kd.updateLabel();
  bal.slider.value = p.bal_max_rpm; bal.updateLabel();
  slew.slider.value = p.max_rpm_step; slew.updateLabel();
  dtau.slider.value = p.d_tau; dtau.updateLabel();
  spoff.slider.value = p.max_setpoint_offset; spoff.updateLabel();
  invgyro.checked = !!p.invert_gyro;

  let t = null;
  const push = () => {{
    clearTimeout(t);
    t = setTimeout(() => {{
      setParams({{
        kp: parseFloat(kp.slider.value),
        ki: parseFloat(ki.slider.value),
        kd: parseFloat(kd.slider.value),
        bal_max_rpm: parseFloat(bal.slider.value),
        max_rpm_step: parseFloat(slew.slider.value),
        d_tau: parseFloat(dtau.slider.value),
        max_setpoint_offset: parseFloat(spoff.slider.value),
        invert_gyro: invgyro.checked
      }});
    }}, 120);
  }};

  for (const s of [kp.slider,ki.slider,kd.slider,bal.slider,slew.slider,dtau.slider,spoff.slider]) {{
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
        return jsonify(get_params_dict())

    @app.post("/params")
    def params_post():
        d = request.get_json(force=True, silent=True) or {}

        # clamping
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
        fclamp("max_rpm_step", *MAX_RPM_STEP_RANGE)
        fclamp("d_tau", *D_TAU_RANGE)
        fclamp("max_setpoint_offset", *SETPOINT_OFF_RANGE)

        if "invert_gyro" in d:
            d["invert_gyro"] = bool(d["invert_gyro"])

        set_params_from_dict(d)
        return jsonify({"ok": True, **get_params_dict()})

    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)

# ===================== MAIN =====================
def main() -> int:
    i2c_require()
    if Flask is None:
        print("ERROR: Flask no instalado. Instala: pip3 install flask", file=sys.stderr)
        return 1

    # Web en hilo
    th = threading.Thread(target=start_web_server, daemon=True)
    th.start()
    print(f"Web: http://<IP_RPI>:{WEB_PORT}/")

    # I2C init
    bus = SMBus(I2C_BUS)
    try:
        mpu_wake(bus)
    except Exception as e:
        print(f"ERROR I2C: no se pudo inicializar MPU6050 en 0x{MPU_ADDR:02X}: {e}", file=sys.stderr)
        bus.close()
        return 3

    print("Calibrando IMU (quieto y en posición de equilibrio)...")
    try:
        gyro_y_bias_dps = calibrate_gyro_y_bias(bus)
        accel_zero_deg = calibrate_accel_angle_zero(bus)
    except Exception as e:
        print(f"ERROR I2C: calibración falló: {e}", file=sys.stderr)
        bus.close()
        return 4

    kf = Kalman1D()
    kf.set_angle(0.0)
    print(f"OK. Bias Gy={gyro_y_bias_dps:.6f} °/s | AccelZero={accel_zero_deg:.6f} ° | Ang inicial=0.000°")

    # RS485 init
    ser = open_serial()

    # Joystick init
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("ERROR: no hay joystick detectado por pygame.")
        ser.close()
        bus.close()
        return 2

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Joystick: {joy.get_name()} | axes={joy.get_numaxes()} buttons={joy.get_numbuttons()}")

    # Enable motores (robusto)
    send_enable_robust(ser, ADDR_LEFT, True)
    send_enable_robust(ser, ADDR_RIGHT, True)
    time.sleep(0.05)

    # PID states
    integ = 0.0
    prev_err = 0.0
    d_filt = 0.0
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

            # --- IMU read ---
            try:
                ax, az, gy_raw = read_accel_gyro(bus)
                accel_angle = accel_angle_deg_from_ax_az(ax, az) - accel_zero_deg
                gyro_rate = (gy_raw / GYRO_LSB_PER_DPS) - gyro_y_bias_dps

                p = get_params_dict()
                if p["invert_gyro"]:
                    gyro_rate = -gyro_rate

                angle = kf.update(accel_angle, gyro_rate, dt)

            except Exception as e:
                print(f"\nWARNING I2C: lectura IMU falló: {e}", file=sys.stderr)
                angle = kf.angle
                gyro_rate = 0.0
                accel_angle = 0.0
                p = get_params_dict()

            # --- Safety cutoff ---
            if abs(angle) > ANGLE_CUTOFF_DEG:
                send_paced(ser, cmd_speed(ADDR_LEFT, 0, 0))
                send_paced(ser, cmd_speed(ADDR_RIGHT, 0, 0))
                sys.stdout.write(f"\rCAIDO: Ang={angle:+07.2f} deg (cutoff) -> motores 0 rpm                 ")
                sys.stdout.flush()
                integ = 0.0
                prev_err = 0.0
                d_filt = 0.0
                base_rpm_cmd = 0.0

                t_next += period
                sleep_s = t_next - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    t_next = time.monotonic()
                continue

            # --- Joystick -> setpoint y giro ---
            thr = -joy.get_axis(AXIS_THROTTLE)
            trn = joy.get_axis(AXIS_TURN)

            thr = clamp(dz(thr, DEADZONE), -1.0, 1.0)
            trn = clamp(dz(trn, DEADZONE), -1.0, 1.0)

            setpoint = SETPOINT_DEG + thr * p["max_setpoint_offset"]

            # --- PID balance -> base rpm ---
            err = setpoint - angle

            integ += err * dt
            integ = clamp(integ, -p["i_lim"], +p["i_lim"])

            derr = (err - prev_err) / dt if dt > 0 else 0.0
            prev_err = err

            # Filtro derivada (1er orden) para reducir vibración por ruido
            d_tau = p["d_tau"]
            if d_tau <= 0.0:
                d_filt = derr
            else:
                alpha = dt / (d_tau + dt)
                d_filt += alpha * (derr - d_filt)

            base_rpm = (p["kp"] * err) + (p["ki"] * integ) + (p["kd"] * d_filt)

            # Limitador de salida (balanceo)
            base_rpm = clamp(base_rpm, -p["bal_max_rpm"], +p["bal_max_rpm"])

            # Slew-rate (anti-latigazo)
            max_step = p["max_rpm_step"] * dt
            delta = clamp(base_rpm - base_rpm_cmd, -max_step, +max_step)
            base_rpm_cmd += delta
            base_rpm_cmd = clamp(base_rpm_cmd, -p["bal_max_rpm"], +p["bal_max_rpm"])

            # Turn en rpm (si quieres limitar giro, aplica aquí un factor)
            turn_rpm = trn * MAX_RPM
            mc = motors_from_balance(base_rpm_cmd, turn_rpm)

            # --- RS485 send ---
            send_paced(ser, cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
            send_paced(ser, cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

            # --- Print status ---
            sys.stdout.write(
                f"\rAng={angle:+07.2f} | Acc={accel_angle:+07.2f} | Gy={gyro_rate:+07.2f} | "
                f"SP={setpoint:+06.2f} | base={base_rpm_cmd:+06.1f} | "
                f"PID({p['kp']:.1f},{p['ki']:.2f},{p['kd']:.2f}) "
                f"bal={p['bal_max_rpm']:.0f} slew={p['max_rpm_step']:.0f} dtau={p['d_tau']:.3f} "
                f"L={mc.left_rpm:+04d} R={mc.right_rpm:+04d}    "
            )
            sys.stdout.flush()

            # timing
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
