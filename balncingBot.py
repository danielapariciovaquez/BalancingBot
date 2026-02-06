#!/usr/bin/env python3
import time
import sys
import math
import serial
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

BAL_MAX_RPM = 40

INTER_FRAME_DELAY_S = 0.004
ENABLE_RETRIES = 2
ENABLE_RETRY_DELAY_S = 0.02

INVERT_LEFT  = False
INVERT_RIGHT = True
# ===========================================================

# ===================== CONTROL LOOP =====================
UPDATE_HZ = 50
DT_MAX = 0.05

ANGLE_CUTOFF_DEG = 40.0
MAX_RPM_STEP_PER_S = 500.0
# =========================================================

# ===================== CONTROL (PD+gyro) =====================
PID_INIT_KP = 1
PID_INIT_KI = 0.0
PID_INIT_KD = 0
I_LIM = 20.0
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

# Si el gyro sale con signo opuesto al accel en tu montaje:
INVERT_GYRO_RATE = False

# ===== Filtro complementario =====
# alpha cercano a 1: confías más en gyro (menos ruido) pero más deriva
# alpha más bajo: más accel (más ruido) pero menos deriva
COMP_ALPHA = 0.985
# ===========================================================

# ===================== WEB UI =====================
WEB_HOST = "0.0.0.0"
WEB_PORT = 8000

KP_RANGE = (0.0, 80.0)
KI_RANGE = (0.0, 50.0)
KD_RANGE = (0.0, 30.0)
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
    """
    Pitch from AX/AZ.
    Requisito: inclinar hacia delante => ángulo decrementa.
    """
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

def motors_from_balance(base_rpm: float) -> MotorCmd:
    left = base_rpm
    right = base_rpm

    left_rpm = int(round(clamp(left, -MAX_RPM, MAX_RPM)))
    right_rpm = int(round(clamp(right, -MAX_RPM, MAX_RPM)))

    if INVERT_LEFT:
        left_rpm = -left_rpm
    if INVERT_RIGHT:
        right_rpm = -right_rpm

    return MotorCmd(left_rpm=left_rpm, right_rpm=right_rpm)

# ===================== PID shared state (web <-> loop) =====================
class PIDGains:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

pid_lock = threading.Lock()
pid_gains = PIDGains(PID_INIT_KP, PID_INIT_KI, PID_INIT_KD)

def get_pid():
    with pid_lock:
        return pid_gains.kp, pid_gains.ki, pid_gains.kd

def set_pid(kp: float, ki: float, kd: float):
    with pid_lock:
        pid_gains.kp = float(kp)
        pid_gains.ki = float(ki)
        pid_gains.kd = float(kd)

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
  <title>PID Self-Balancing</title>
  <style>
    body {{ font-family: sans-serif; margin: 20px; max-width: 780px; }}
    .row {{ margin: 18px 0; }}
    .label {{ display:flex; justify-content:space-between; margin-bottom: 6px; }}
    input[type=range] {{ width: 100%; }}
    .box {{ padding: 12px; border: 1px solid #ddd; border-radius: 10px; }}
    .small {{ color:#555; font-size: 0.95em; }}
    code {{ background:#f6f6f6; padding:2px 6px; border-radius:6px; }}
  </style>
</head>
<body>
  <h2>PID (Kp, Ki, Kd)</h2>
  <div class="box">
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
    <div class="small">
      Filtro: <code>Complementario</code> (α={COMP_ALPHA}). Consejo: empieza con <code>Ki=0</code>.
    </div>
  </div>

<script>
async function getPID() {{
  const r = await fetch('/pid');
  return await r.json();
}}
async function setPID(kp,ki,kd) {{
  await fetch('/pid', {{
    method: 'POST',
    headers: {{'Content-Type':'application/json'}},
    body: JSON.stringify({{kp:kp, ki:ki, kd:kd}})
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
  const pid = await getPID();
  const kp = bindSlider('kp','kpv',v=>v.toFixed(1));
  const ki = bindSlider('ki','kiv',v=>v.toFixed(2));
  const kd = bindSlider('kd','kdv',v=>v.toFixed(1));
  kp.slider.value = pid.kp; kp.updateLabel();
  ki.slider.value = pid.ki; ki.updateLabel();
  kd.slider.value = pid.kd; kd.updateLabel();
  let t = null;
  const push = () => {{
    clearTimeout(t);
    t = setTimeout(() => {{
      setPID(parseFloat(kp.slider.value), parseFloat(ki.slider.value), parseFloat(kd.slider.value));
    }}, 120);
  }};
  kp.slider.addEventListener('input', push);
  ki.slider.addEventListener('input', push);
  kd.slider.addEventListener('input', push);
}})();
</script>
</body>
</html>
"""

    @app.get("/")
    def index():
        return HTML

    @app.get("/pid")
    def pid_get():
        kp, ki, kd = get_pid()
        return {"kp": kp, "ki": ki, "kd": kd}

    @app.post("/pid")
    def pid_post():
        j = request.get_json(force=True, silent=True) or {}
        try:
            kp = float(j.get("kp"))
            ki = float(j.get("ki"))
            kd = float(j.get("kd"))
        except Exception:
            return jsonify({"ok": False, "error": "kp/ki/kd inválidos"}), 400

        kp = clamp(kp, KP_RANGE[0], KP_RANGE[1])
        ki = clamp(ki, KI_RANGE[0], KI_RANGE[1])
        kd = clamp(kd, KD_RANGE[0], KD_RANGE[1])
        set_pid(kp, ki, kd)
        return jsonify({"ok": True, "kp": kp, "ki": ki, "kd": kd})

    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)

# ===================== main =====================
def main() -> int:
    i2c_require()
    if Flask is None:
        print("ERROR: Flask no instalado. Instala: pip3 install flask", file=sys.stderr)
        return 1

    th = threading.Thread(target=start_web_server, daemon=True)
    th.start()
    print(f"Web PID: http://<IP_RPI>:{WEB_PORT}/")

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

    # ángulo inicial (tras calibración) = 0
    angle = 0.0

    print(f"OK. BiasGy={gyro_y_bias_dps:.6f} °/s | AccZero={accel_zero_deg:.6f} ° | Ang0=0.000°")

    ser = open_serial()
    send_enable_robust(ser, ADDR_LEFT, True)
    send_enable_robust(ser, ADDR_RIGHT, True)

    integ = 0.0
    base_rpm_cmd = 0.0

    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()
    t_prev = t_next

    try:
        while True:
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            if dt < 0.0:
                dt = 0.0
            elif dt > DT_MAX:
                dt = DT_MAX

            # IMU -> accel angle + gyro rate
            try:
                ax, az, gy_raw = read_accel_gyro(bus)
                accel_angle = accel_angle_deg_from_ax_az(ax, az) - accel_zero_deg
                gyro_rate = (gy_raw / GYRO_LSB_PER_DPS) - gyro_y_bias_dps
                if INVERT_GYRO_RATE:
                    gyro_rate = -gyro_rate
            except Exception as e:
                print(f"\nWARNING I2C: lectura IMU falló: {e}", file=sys.stderr)
                accel_angle = 0.0
                gyro_rate = 0.0

            # ======= Filtro complementario =======
            # Predicción con gyro (integración)
            angle_gyro = angle + gyro_rate * dt
            # Corrección con accel
            angle = (COMP_ALPHA * angle_gyro) + ((1.0 - COMP_ALPHA) * accel_angle)

            # Safety cutoff
            if abs(angle) > ANGLE_CUTOFF_DEG:
                send_paced(ser, cmd_speed(ADDR_LEFT, 0, 0))
                send_paced(ser, cmd_speed(ADDR_RIGHT, 0, 0))
                integ = 0.0
                base_rpm_cmd = 0.0
                angle = 0.0  # opcional: reancla para que no se quede "lejos"
                sys.stdout.write(f"\rCAIDO: Ang={angle:+07.2f} deg -> motores 0 rpm                           ")
                sys.stdout.flush()

                t_next += period
                sleep_s = t_next - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    t_next = time.monotonic()
                continue

            # PID params desde web
            kp, ki, kd = get_pid()

            # Error con setpoint=0
            err = 0.0 - angle

            integ += err * dt
            integ = clamp(integ, -I_LIM, I_LIM)

            # PD con gyro directo + integral opcional
            base_rpm = (kp * err) - (kd * gyro_rate) + (ki * integ)
            base_rpm = clamp(base_rpm, -BAL_MAX_RPM, +BAL_MAX_RPM)

            # Slew-rate
            max_step = MAX_RPM_STEP_PER_S * dt
            delta = clamp(base_rpm - base_rpm_cmd, -max_step, +max_step)
            base_rpm_cmd += delta
            base_rpm_cmd = clamp(base_rpm_cmd, -BAL_MAX_RPM, +BAL_MAX_RPM)

            mc = motors_from_balance(base_rpm_cmd)

            send_paced(ser, cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
            send_paced(ser, cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

            sys.stdout.write(
                f"\rAng={angle:+07.2f} | Acc={accel_angle:+07.2f} | Gy={gyro_rate:+07.2f} dps | "
                f"α={COMP_ALPHA:.3f} | PID(kp={kp:5.1f},ki={ki:5.2f},kd={kd:5.1f}) | "
                f"base={base_rpm_cmd:+06.1f} | L={mc.left_rpm:+04d} R={mc.right_rpm:+04d}     "
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

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
