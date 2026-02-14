#!/usr/bin/env python3
import time
import sys
import math
import serial
import threading
import traceback
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

# ===================== CONTROL =====================
UPDATE_HZ = 150
DT_MAX = 0.05
ANGLE_CUTOFF_DEG = 35.0
SETPOINT_DEG = 0.0

MAX_RPM_STEP_PER_S = 700.0
BAL_MAX_RPM_DEFAULT = 120.0
I_LIM_DEFAULT = 200.0
# ===================================================

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
# ===========================================================

# ===================== WEB =====================
WEB_HOST = "0.0.0.0"
WEB_PORT = 8000

KP_RANGE = (0.0, 80.0)
KI_RANGE = (0.0, 20.0)
KD_RANGE = (0.0, 50.0)

BAL_MAX_RPM_RANGE = (0.0, 300.0)
SLEW_RANGE = (0.0, 6000.0)
I_LIM_RANGE = (0.0, 2000.0)

ACC_TAU_RANGE = (0.0, 0.30)
GYRO_TAU_RANGE = (0.0, 0.30)

K_QA_RANGE = (1e-6, 5e-2)
K_QB_RANGE = (1e-6, 5e-1)
K_RM_RANGE = (1e-4, 2e-1)
# ===========================================================

# ---- Tolerancia a fallos IMU ----
IMU_MAX_CONSECUTIVE_ERRORS = 20   # 20 iteraciones seguidas con error
IMU_ERROR_MOTOR_ZERO = True       # en error IMU: manda 0 rpm (seguro)
IMU_ERROR_SLEEP_S = 0.002         # pequeña pausa para no quemar I2C
# --------------------------------

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

def motors_from_balance(base_rpm: float) -> MotorCmd:
    left_rpm = int(round(clamp(base_rpm, -MAX_RPM, MAX_RPM)))
    right_rpm = int(round(clamp(base_rpm, -MAX_RPM, MAX_RPM)))
    if INVERT_LEFT:
        left_rpm = -left_rpm
    if INVERT_RIGHT:
        right_rpm = -right_rpm
    return MotorCmd(left_rpm=left_rpm, right_rpm=right_rpm)

def i2c_require():
    if SMBus is None:
        raise RuntimeError("No se encontró smbus2/smbus. Instala: pip3 install smbus2")

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
    ax_g = ax_raw / ACC_LSB_PER_G
    az_g = az_raw / ACC_LSB_PER_G
    ang = math.degrees(math.atan2(ax_g, az_g))
    return -ang

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

def lpf_update(x_f: float, x: float, tau: float, dt: float) -> float:
    if tau <= 0.0 or dt <= 0.0:
        return x
    a = dt / (tau + dt)
    return x_f + a * (x - x_f)

class Kalman1D:
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

        angle_new = self.angle + K0 * y
        bias_new  = self.bias  + K1 * y

        P00_old, P01_old, P10_old, P11_old = self.P00, self.P01, self.P10, self.P11
        self.P00 = P00_old - K0 * P00_old
        self.P01 = P01_old - K0 * P01_old
        self.P10 = P10_old - K1 * P00_old
        self.P11 = P11_old - K1 * P01_old

        self.angle = angle_new
        self.bias = bias_new
        return self.angle

class Params:
    def __init__(self):
        self.kp = 18.0
        self.ki = 0.0
        self.kd = 0.9

        self.i_lim = I_LIM_DEFAULT
        self.bal_max_rpm = BAL_MAX_RPM_DEFAULT
        self.max_rpm_step = MAX_RPM_STEP_PER_S

        self.acc_tau = 0.03
        self.gyro_tau = 0.02

        self.k_q_angle = 0.001
        self.k_q_bias = 0.003
        self.k_r_meas = 0.03

        self.invert_gyro = False

param_lock = threading.Lock()
params = Params()

def get_params():
    with param_lock:
        return {
            "kp": params.kp, "ki": params.ki, "kd": params.kd,
            "i_lim": params.i_lim,
            "bal_max_rpm": params.bal_max_rpm,
            "max_rpm_step": params.max_rpm_step,
            "acc_tau": params.acc_tau, "gyro_tau": params.gyro_tau,
            "k_q_angle": params.k_q_angle, "k_q_bias": params.k_q_bias, "k_r_meas": params.k_r_meas,
            "invert_gyro": params.invert_gyro,
        }

def set_params(d: dict):
    with param_lock:
        for k in ("kp","ki","kd","i_lim","bal_max_rpm","max_rpm_step","acc_tau","gyro_tau","k_q_angle","k_q_bias","k_r_meas"):
            if k in d:
                setattr(params, k, float(d[k]))
        if "invert_gyro" in d:
            params.invert_gyro = bool(d["invert_gyro"])

def start_web_server():
    web_require()
    app = Flask(__name__)

    HTML = f"""<!doctype html><html lang="es"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Balance PID + Filtros IMU</title>
<style>
body {{ font-family:sans-serif; margin:20px; max-width:920px; }}
.row {{ margin:16px 0; }}
.label {{ display:flex; justify-content:space-between; margin-bottom:6px; }}
input[type=range] {{ width:100%; }}
.box {{ padding:12px; border:1px solid #ddd; border-radius:12px; }}
.small {{ color:#555; font-size:0.95em; line-height:1.35; }}
</style></head><body>
<h2>Self-Balancing (SIN mando): PID + Filtros IMU + Kalman</h2>
<div class="box">
<h3>PID</h3>
<div class="row"><div class="label"><b>Kp</b><span id="kpv"></span></div><input id="kp" type="range" min="{KP_RANGE[0]}" max="{KP_RANGE[1]}" step="0.1"></div>
<div class="row"><div class="label"><b>Ki</b><span id="kiv"></span></div><input id="ki" type="range" min="{KI_RANGE[0]}" max="{KI_RANGE[1]}" step="0.01"></div>
<div class="row"><div class="label"><b>Kd</b><span id="kdv"></span></div><input id="kd" type="range" min="{KD_RANGE[0]}" max="{KD_RANGE[1]}" step="0.1"></div>

<h3>Estabilidad</h3>
<div class="row"><div class="label"><b>BAL_MAX_RPM</b><span id="balv"></span></div><input id="bal" type="range" min="{BAL_MAX_RPM_RANGE[0]}" max="{BAL_MAX_RPM_RANGE[1]}" step="1"></div>
<div class="row"><div class="label"><b>Slew rpm/s</b><span id="slewv"></span></div><input id="slew" type="range" min="{SLEW_RANGE[0]}" max="{SLEW_RANGE[1]}" step="10"></div>
<div class="row"><div class="label"><b>I_LIM</b><span id="ilimv"></span></div><input id="ilim" type="range" min="{I_LIM_RANGE[0]}" max="{I_LIM_RANGE[1]}" step="1"></div>

<h3>Filtros IMU</h3>
<div class="row"><div class="label"><b>ACC_TAU (s)</b><span id="acctauv"></span></div><input id="acctau" type="range" min="{ACC_TAU_RANGE[0]}" max="{ACC_TAU_RANGE[1]}" step="0.005"></div>
<div class="row"><div class="label"><b>GYRO_TAU (s)</b><span id="gyrotauv"></span></div><input id="gyrotau" type="range" min="{GYRO_TAU_RANGE[0]}" max="{GYRO_TAU_RANGE[1]}" step="0.005"></div>

<h3>Kalman</h3>
<div class="row"><div class="label"><b>Q_angle</b><span id="qav"></span></div><input id="qa" type="range" min="{K_QA_RANGE[0]}" max="{K_QA_RANGE[1]}" step="0.000001"></div>
<div class="row"><div class="label"><b>Q_bias</b><span id="qbv"></span></div><input id="qb" type="range" min="{K_QB_RANGE[0]}" max="{K_QB_RANGE[1]}" step="0.000001"></div>
<div class="row"><div class="label"><b>R_measure</b><span id="rmv"></span></div><input id="rm" type="range" min="{K_RM_RANGE[0]}" max="{K_RM_RANGE[1]}" step="0.0001"></div>

<div class="row"><label><input id="invgyro" type="checkbox"> Invertir signo gyro_rate</label></div>
<div class="small">Setpoint fijo 0°. Derivada sin filtro.</div>
</div>

<script>
async function getP(){{ const r=await fetch('/params'); return await r.json(); }}
async function setP(p){{ await fetch('/params',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(p)}}); }}
function bind(id, vid, fmt){{ const s=document.getElementById(id); const v=document.getElementById(vid);
const upd=()=>v.textContent=fmt(parseFloat(s.value)); s.addEventListener('input',upd); return {{s,upd}}; }}

(async()=>{{
  const p=await getP();
  const kp=bind('kp','kpv',x=>x.toFixed(1));
  const ki=bind('ki','kiv',x=>x.toFixed(2));
  const kd=bind('kd','kdv',x=>x.toFixed(2));
  const bal=bind('bal','balv',x=>x.toFixed(0));
  const slew=bind('slew','slewv',x=>x.toFixed(0));
  const ilim=bind('ilim','ilimv',x=>x.toFixed(0));
  const acctau=bind('acctau','acctauv',x=>x.toFixed(3));
  const gyrotau=bind('gyrotau','gyrotauv',x=>x.toFixed(3));
  const qa=bind('qa','qav',x=>x.toExponential(3));
  const qb=bind('qb','qbv',x=>x.toExponential(3));
  const rm=bind('rm','rmv',x=>x.toExponential(3));
  const inv=document.getElementById('invgyro');

  kp.s.value=p.kp; kp.upd(); ki.s.value=p.ki; ki.upd(); kd.s.value=p.kd; kd.upd();
  bal.s.value=p.bal_max_rpm; bal.upd(); slew.s.value=p.max_rpm_step; slew.upd(); ilim.s.value=p.i_lim; ilim.upd();
  acctau.s.value=p.acc_tau; acctau.upd(); gyrotau.s.value=p.gyro_tau; gyrotau.upd();
  qa.s.value=p.k_q_angle; qa.upd(); qb.s.value=p.k_q_bias; qb.upd(); rm.s.value=p.k_r_meas; rm.upd();
  inv.checked=!!p.invert_gyro;

  let t=null;
  const push=()=>{{ clearTimeout(t); t=setTimeout(()=>{{
    setP({{
      kp:parseFloat(kp.s.value), ki:parseFloat(ki.s.value), kd:parseFloat(kd.s.value),
      bal_max_rpm:parseFloat(bal.s.value), max_rpm_step:parseFloat(slew.s.value), i_lim:parseFloat(ilim.s.value),
      acc_tau:parseFloat(acctau.s.value), gyro_tau:parseFloat(gyrotau.s.value),
      k_q_angle:parseFloat(qa.s.value), k_q_bias:parseFloat(qb.s.value), k_r_meas:parseFloat(rm.s.value),
      invert_gyro:inv.checked
    }});
  }},120); }};
  for (const el of [kp.s,ki.s,kd.s,bal.s,slew.s,ilim.s,acctau.s,gyrotau.s,qa.s,qb.s,rm.s]) el.addEventListener('input',push);
  inv.addEventListener('change',push);
}})();
</script></body></html>"""

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

        fclamp("kp", *KP_RANGE); fclamp("ki", *KI_RANGE); fclamp("kd", *KD_RANGE)
        fclamp("bal_max_rpm", *BAL_MAX_RPM_RANGE); fclamp("max_rpm_step", *SLEW_RANGE); fclamp("i_lim", *I_LIM_RANGE)
        fclamp("acc_tau", *ACC_TAU_RANGE); fclamp("gyro_tau", *GYRO_TAU_RANGE)
        fclamp("k_q_angle", *K_QA_RANGE); fclamp("k_q_bias", *K_QB_RANGE); fclamp("k_r_meas", *K_RM_RANGE)
        if "invert_gyro" in d:
            d["invert_gyro"] = bool(d["invert_gyro"])

        set_params(d)
        return jsonify({"ok": True, **get_params()})

    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)

def main() -> int:
    if SMBus is None:
        print("ERROR: smbus2/smbus no instalado. pip3 install smbus2", file=sys.stderr)
        return 1
    if Flask is None:
        print("ERROR: Flask no instalado. pip3 install flask", file=sys.stderr)
        return 1

    threading.Thread(target=start_web_server, daemon=True).start()
    print(f"Web: http://<IP_RPI>:{WEB_PORT}/")

    bus = SMBus(I2C_BUS)
    mpu_wake(bus)

    print("Calibrando IMU (quieto y en posición de equilibrio)...")
    gyro_y_bias_dps = calibrate_gyro_y_bias(bus)
    accel_zero_deg = calibrate_accel_angle_zero(bus)
    print(f"OK. BiasGy={gyro_y_bias_dps:.6f} °/s | AccZero={accel_zero_deg:.6f} °")

    kf = Kalman1D(params.k_q_angle, params.k_q_bias, params.k_r_meas)
    kf.set_angle(0.0)

    accel_angle_f = 0.0
    gyro_rate_f = 0.0

    ser = open_serial()
    send_enable_robust(ser, ADDR_LEFT, True)
    send_enable_robust(ser, ADDR_RIGHT, True)
    time.sleep(0.05)

    integ = 0.0
    prev_err = 0.0
    base_rpm_cmd = 0.0

    imu_err_streak = 0

    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()
    t_prev = t_next

    start_t = time.monotonic()
    last_alive_print = start_t

    try:
        while True:
            try:
                now = time.monotonic()
                dt = now - t_prev
                t_prev = now
                if dt < 0.0:
                    dt = 0.0
                elif dt > DT_MAX:
                    dt = DT_MAX

                p = get_params()
                kf.set_params(p["k_q_angle"], p["k_q_bias"], p["k_r_meas"])

                ax, az, gy_raw = read_accel_gyro(bus)
                accel_angle = accel_angle_deg_from_ax_az(ax, az) - accel_zero_deg
                gyro_rate = (gy_raw / GYRO_LSB_PER_DPS) - gyro_y_bias_dps
                if p["invert_gyro"]:
                    gyro_rate = -gyro_rate

                accel_angle_f = lpf_update(accel_angle_f, accel_angle, p["acc_tau"], dt)
                gyro_rate_f = lpf_update(gyro_rate_f, gyro_rate, p["gyro_tau"], dt)

                angle = kf.update(accel_angle_f, gyro_rate_f, dt)

                imu_err_streak = 0  # lectura ok

                if abs(angle) > ANGLE_CUTOFF_DEG:
                    send_paced(ser, cmd_speed(ADDR_LEFT, 0, 0))
                    send_paced(ser, cmd_speed(ADDR_RIGHT, 0, 0))
                    integ = 0.0
                    prev_err = 0.0
                    base_rpm_cmd = 0.0
                    sys.stdout.write(f"\rCAIDO: Ang={angle:+07.2f} deg -> motores 0 rpm                         ")
                    sys.stdout.flush()
                else:
                    err = SETPOINT_DEG - angle
                    integ += err * dt
                    integ = clamp(integ, -p["i_lim"], +p["i_lim"])

                    derr = (err - prev_err) / dt if dt > 0 else 0.0
                    prev_err = err

                    base_rpm = (p["kp"] * err) + (p["ki"] * integ) + (p["kd"] * derr)
                    base_rpm = clamp(base_rpm, -p["bal_max_rpm"], +p["bal_max_rpm"])

                    max_step = p["max_rpm_step"] * dt
                    base_rpm_cmd += clamp(base_rpm - base_rpm_cmd, -max_step, +max_step)
                    base_rpm_cmd = clamp(base_rpm_cmd, -p["bal_max_rpm"], +p["bal_max_rpm"])

                    mc = motors_from_balance(base_rpm_cmd)
                    send_paced(ser, cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
                    send_paced(ser, cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

                    sys.stdout.write(
                        f"\rAng={angle:+07.2f} | AccF={accel_angle_f:+07.2f} | GyF={gyro_rate_f:+07.2f} | "
                        f"err={err:+07.2f} | base={base_rpm_cmd:+06.1f} | "
                        f"L={mc.left_rpm:+04d} R={mc.right_rpm:+04d}    "
                    )
                    sys.stdout.flush()

                # “alive” cada 5 s
                if now - last_alive_print > 5.0:
                    last_alive_print = now
                    print(f"\n[alive] t={now-start_t:.1f}s imu_err_streak={imu_err_streak}")

                t_next += period
                sleep_s = t_next - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    t_next = time.monotonic()

            except Exception as e:
                imu_err_streak += 1
                print(f"\n[ERROR loop] {type(e).__name__}: {e}", file=sys.stderr)
                traceback.print_exc()

                if IMU_ERROR_MOTOR_ZERO:
                    try:
                        send_paced(ser, cmd_speed(ADDR_LEFT, 0, 0))
                        send_paced(ser, cmd_speed(ADDR_RIGHT, 0, 0))
                    except Exception:
                        pass

                if imu_err_streak >= IMU_MAX_CONSECUTIVE_ERRORS:
                    print("\n[ERROR] demasiados fallos IMU seguidos -> saliendo (seguro)", file=sys.stderr)
                    break

                time.sleep(IMU_ERROR_SLEEP_S)

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
            print(f"WARNING: stop/disable falló: {e}", file=sys.stderr)

        try:
            ser.close()
        except Exception:
            pass
        try:
            bus.close()
        except Exception:
            pass

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
