#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import signal
import threading
from dataclasses import dataclass, asdict

import serial
from flask import Flask, request, jsonify

from smbus2 import SMBus


# ============================================================
# CONFIG HW
# ============================================================

I2C_BUS = 1
MPU_ADDR = 0x68

RS485_PORT = "/dev/ttyUSB0"
RS485_BAUD = 38400
RS485_TIMEOUT_S = 0.05
INTER_FRAME_DELAY_S = 0.002

MOTOR1_ADDR = 0x01
MOTOR2_ADDR = 0x02

M1_SIGN = +1
M2_SIGN = -1


# ============================================================
# PARÁMETROS
# ============================================================

@dataclass
class Params:
    loop_hz: float = 200.0

    safe_angle_deg: float = 25.0          # umbral de caída
    recover_angle_deg: float = 10.0       # umbral de rearme (histeresis)
    recover_hold_s: float = 0.5           # tiempo estable para rearme
    startup_grace_s: float = 1.0          # gracia tras arranque/calibración

    max_rpm: int = 200
    acc: int = 50

    kp: float = 25.0
    ki: float = 0.0
    kd: float = 0.8
    integrator_limit: float = 300.0

    invert_gyro: bool = False

    q_angle: float = 0.001
    q_bias: float = 0.003
    r_measure: float = 0.03

P = Params()


# ============================================================
# MKS SERVO RS485 (SOLO F3 y F6)
# ============================================================

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def mks_frame(addr: int, func: int, payload: bytes) -> bytes:
    base = bytes([0xFA, addr & 0xFF, func & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def mks_enable(ser: serial.Serial, addr: int, enable: bool) -> None:
    ser.write(mks_frame(addr, 0xF3, bytes([0x01 if enable else 0x00])))
    ser.flush()
    time.sleep(INTER_FRAME_DELAY_S)

def mks_speed_signed(ser: serial.Serial, addr: int, rpm_signed: float, acc: int) -> None:
    acc_i = int(max(0, min(255, int(acc))))
    rpm_i = int(round(rpm_signed))

    if rpm_i == 0:
        ser.write(mks_frame(addr, 0xF6, bytes([0x00, 0x00, acc_i])))
        ser.flush()
        time.sleep(INTER_FRAME_DELAY_S)
        return

    direction = 0 if rpm_i > 0 else 1
    rpm_mag = max(0, min(3000, abs(rpm_i)))

    b4 = ((direction & 0x01) << 7) | ((rpm_mag >> 8) & 0x0F)
    b5 = rpm_mag & 0xFF

    ser.write(mks_frame(addr, 0xF6, bytes([b4, b5, acc_i])))
    ser.flush()
    time.sleep(INTER_FRAME_DELAY_S)

def motors_stop_and_disable(ser: serial.Serial) -> None:
    try:
        for a in (MOTOR1_ADDR, MOTOR2_ADDR):
            ser.write(mks_frame(a, 0xF6, bytes([0x00, 0x00, 0x00])))
            ser.flush()
            time.sleep(INTER_FRAME_DELAY_S)
        for a in (MOTOR1_ADDR, MOTOR2_ADDR):
            ser.write(mks_frame(a, 0xF3, bytes([0x00])))
            ser.flush()
            time.sleep(INTER_FRAME_DELAY_S)
    except Exception:
        pass


# ============================================================
# MPU6050
# ============================================================

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

def twos_complement_16(msb: int, lsb: int) -> int:
    v = (msb << 8) | lsb
    if v & 0x8000:
        v -= 0x10000
    return v

class MPU6050:
    def __init__(self, bus: SMBus, addr: int = MPU_ADDR):
        self.bus = bus
        self.addr = addr
        self.accel_lsb_per_g = 16384.0
        self.gyro_lsb_per_dps = 131.0
        self.gyro_bias_dps = 0.0

    def write_reg(self, reg: int, val: int) -> None:
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def read_regs(self, start_reg: int, length: int) -> bytes:
        return bytes(self.bus.read_i2c_block_data(self.addr, start_reg, length))

    def init(self) -> None:
        self.write_reg(PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self.write_reg(SMPLRT_DIV, 0x04)
        self.write_reg(CONFIG, 0x03)
        self.write_reg(GYRO_CONFIG, 0x00)
        self.write_reg(ACCEL_CONFIG, 0x00)

    def read(self):
        d = self.read_regs(ACCEL_XOUT_H, 14)
        ax = twos_complement_16(d[0], d[1])
        ay = twos_complement_16(d[2], d[3])
        az = twos_complement_16(d[4], d[5])
        gx = twos_complement_16(d[8], d[9])
        gy = twos_complement_16(d[10], d[11])
        gz = twos_complement_16(d[12], d[13])

        ax_g = ax / self.accel_lsb_per_g
        ay_g = ay / self.accel_lsb_per_g
        az_g = az / self.accel_lsb_per_g

        gx_dps = gx / self.gyro_lsb_per_dps
        gy_dps = gy / self.gyro_lsb_per_dps
        gz_dps = gz / self.gyro_lsb_per_dps

        return ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps

    def calibrate_gyro(self, seconds: float = 2.0, sample_hz: float = 200.0) -> None:
        n = max(1, int(seconds * sample_hz))
        dt = 1.0 / sample_hz
        s = 0.0
        for _ in range(n):
            _, _, _, gx_dps, _, _ = self.read()
            s += gx_dps
            time.sleep(dt)
        self.gyro_bias_dps = s / n


# ============================================================
# Kalman 1D
# ============================================================

class Kalman1D:
    def __init__(self, q_angle: float, q_bias: float, r_measure: float):
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure
        self.angle = 0.0
        self.bias = 0.0
        self.P00 = 1.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 1.0

    def set_params(self, q_angle: float, q_bias: float, r_measure: float) -> None:
        self.q_angle = float(q_angle)
        self.q_bias = float(q_bias)
        self.r_measure = float(r_measure)

    def update(self, new_angle: float, new_rate: float, dt: float) -> float:
        rate = new_rate - self.bias
        self.angle += dt * rate

        self.P00 += dt * (dt*self.P11 - self.P01 - self.P10 + self.q_angle)
        self.P01 -= dt * self.P11
        self.P10 -= dt * self.P11
        self.P11 += self.q_bias * dt

        S = self.P00 + self.r_measure
        if S == 0.0:
            return self.angle

        K0 = self.P00 / S
        K1 = self.P10 / S
        y = new_angle - self.angle

        self.angle += K0 * y
        self.bias  += K1 * y

        P00_temp = self.P00
        P01_temp = self.P01
        self.P00 -= K0 * P00_temp
        self.P01 -= K0 * P01_temp
        self.P10 -= K1 * P00_temp
        self.P11 -= K1 * P01_temp

        return self.angle


# ============================================================
# PID
# ============================================================

class PID:
    def __init__(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, err: float, dt: float, kp: float, ki: float, kd: float, i_lim: float) -> float:
        if dt <= 0.0:
            return 0.0
        self.integral += err * dt
        if self.integral > i_lim:
            self.integral = i_lim
        elif self.integral < -i_lim:
            self.integral = -i_lim
        derr = (err - self.prev_err) / dt
        self.prev_err = err
        return kp*err + ki*self.integral + kd*derr


# ============================================================
# Web
# ============================================================

app = Flask(__name__)
_state_lock = threading.Lock()

telemetry = {"angle_deg": 0.0, "gyro_dps": 0.0, "pid_out": 0.0, "fallen": False}

HTML = """
<!doctype html><html><head><meta charset="utf-8"/>
<title>BalancingBot</title>
<style>
body{font-family:sans-serif;max-width:900px;margin:20px auto;}
.row{display:grid;grid-template-columns:160px 1fr 90px;gap:12px;align-items:center;margin:8px 0;}
input[type=range]{width:100%;}
code{background:#f4f4f4;padding:2px 6px;border-radius:6px;}
</style></head><body>
<h2>BalancingBot</h2>

<div class="row"><div>Kp</div><input id="kp" type="range" min="0" max="200" step="0.1"><div><span id="kp_v"></span></div></div>
<div class="row"><div>Ki</div><input id="ki" type="range" min="0" max="50" step="0.01"><div><span id="ki_v"></span></div></div>
<div class="row"><div>Kd</div><input id="kd" type="range" min="0" max="50" step="0.01"><div><span id="kd_v"></span></div></div>

<div class="row"><div>MAX RPM</div><input id="max_rpm" type="range" min="0" max="1000" step="1"><div><span id="max_rpm_v"></span></div></div>
<div class="row"><div>ACC</div><input id="acc" type="range" min="0" max="255" step="1"><div><span id="acc_v"></span></div></div>

<div class="row"><div>SAFE deg</div><input id="safe_angle_deg" type="range" min="5" max="60" step="1"><div><span id="safe_angle_deg_v"></span></div></div>
<div class="row"><div>RECOVER deg</div><input id="recover_angle_deg" type="range" min="1" max="30" step="1"><div><span id="recover_angle_deg_v"></span></div></div>
<div class="row"><div>REC HOLD s</div><input id="recover_hold_s" type="range" min="0.1" max="3.0" step="0.1"><div><span id="recover_hold_s_v"></span></div></div>
<div class="row"><div>GRACE s</div><input id="startup_grace_s" type="range" min="0.0" max="5.0" step="0.1"><div><span id="startup_grace_s_v"></span></div></div>

<h3>Kalman</h3>
<div class="row"><div>Q_angle</div><input id="q_angle" type="range" min="0.0001" max="0.02" step="0.0001"><div><span id="q_angle_v"></span></div></div>
<div class="row"><div>Q_bias</div><input id="q_bias" type="range" min="0.0001" max="0.02" step="0.0001"><div><span id="q_bias_v"></span></div></div>
<div class="row"><div>R_measure</div><input id="r_measure" type="range" min="0.001" max="0.2" step="0.001"><div><span id="r_measure_v"></span></div></div>

<div class="row"><div>Invert gyro</div><input id="invert_gyro" type="checkbox"><div></div></div>

<h3>Telemetría</h3>
<p>
angle: <code id="t_angle">0</code> deg |
gyro: <code id="t_gyro">0</code> dps |
pid: <code id="t_pid">0</code> |
fallen: <code id="t_fallen">false</code>
</p>

<script>
async function apiGet(){const r=await fetch('/api/get');return await r.json();}
async function apiSet(k,v){await fetch('/api/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({[k]:v})});}
function bindRange(id){
  const el=document.getElementById(id), out=document.getElementById(id+'_v');
  el.addEventListener('input', async()=>{ out.textContent=el.value; await apiSet(id, parseFloat(el.value)); });
}
async function init(){
  const st=await apiGet();
  const keys=['kp','ki','kd','max_rpm','acc','safe_angle_deg','recover_angle_deg','recover_hold_s','startup_grace_s','q_angle','q_bias','r_measure'];
  for(const k of keys){ const el=document.getElementById(k); el.value=st.params[k]; document.getElementById(k+'_v').textContent=el.value; bindRange(k); }
  const ig=document.getElementById('invert_gyro'); ig.checked=!!st.params.invert_gyro;
  ig.addEventListener('change', async()=> apiSet('invert_gyro', ig.checked));
  setInterval(async()=>{
    const t=await apiGet();
    document.getElementById('t_angle').textContent=t.telemetry.angle_deg.toFixed(2);
    document.getElementById('t_gyro').textContent=t.telemetry.gyro_dps.toFixed(2);
    document.getElementById('t_pid').textContent=t.telemetry.pid_out.toFixed(2);
    document.getElementById('t_fallen').textContent=String(t.telemetry.fallen);
  },200);
}
init();
</script></body></html>
"""

@app.get("/")
def index():
    return HTML

@app.get("/api/get")
def api_get():
    with _state_lock:
        return jsonify({"params": asdict(P), "telemetry": telemetry})

@app.post("/api/set")
def api_set():
    data = request.get_json(force=True, silent=True) or {}
    with _state_lock:
        for k, v in data.items():
            if not hasattr(P, k):
                continue
            if k in ("max_rpm", "acc"):
                setattr(P, k, int(v))
            elif k in ("invert_gyro",):
                setattr(P, k, bool(v))
            else:
                setattr(P, k, float(v))
    return jsonify({"ok": True})

def run_web():
    app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False)


# ============================================================
# MAIN
# ============================================================

stop_flag = False

def _sig_handler(signum, frame):
    global stop_flag
    stop_flag = True

signal.signal(signal.SIGINT, _sig_handler)
signal.signal(signal.SIGTERM, _sig_handler)

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def angle_from_accel_pitch(ax_g: float, ay_g: float, az_g: float) -> float:
    # Pitch robusto: atan2(ax, sqrt(ay^2 + az^2))
    denom = math.sqrt(ay_g*ay_g + az_g*az_g)
    return math.degrees(math.atan2(ax_g, denom))

def main():
    global stop_flag

    threading.Thread(target=run_web, daemon=True).start()

    bus = SMBus(I2C_BUS)
    mpu = MPU6050(bus, MPU_ADDR)
    mpu.init()
    mpu.calibrate_gyro(seconds=2.0, sample_hz=200.0)

    kal = Kalman1D(P.q_angle, P.q_bias, P.r_measure)
    pid = PID()

    ser = serial.Serial(
        RS485_PORT, RS485_BAUD,
        timeout=RS485_TIMEOUT_S,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    fallen = False
    grace_start = time.monotonic()
    recover_ok_since = None

    try:
        # SOLO enable + velocidad 0
        for a in (MOTOR1_ADDR, MOTOR2_ADDR):
            mks_enable(ser, a, True)

        with _state_lock:
            acc0 = int(P.acc)
        for a in (MOTOR1_ADDR, MOTOR2_ADDR):
            mks_speed_signed(ser, a, 0, acc0)

        ax, ay, az, gx, gy, gz = mpu.read()
        angle_acc = angle_from_accel_pitch(ax, ay, az)
        kal.angle = angle_acc

        dt_target = 1.0 / max(1.0, float(P.loop_hz))
        last = time.monotonic()

        while not stop_flag:
            now = time.monotonic()
            dt = now - last
            if dt <= 0.0:
                continue
            last = now

            ax, ay, az, gx, gy, gz = mpu.read()
            angle_acc = angle_from_accel_pitch(ax, ay, az)

            with _state_lock:
                if P.invert_gyro:
                    gyro_rate = -(gx - mpu.gyro_bias_dps)
                else:
                    gyro_rate = (gx - mpu.gyro_bias_dps)

                kal.set_params(P.q_angle, P.q_bias, P.r_measure)

                safe = float(P.safe_angle_deg)
                rec  = float(P.recover_angle_deg)
                hold = float(P.recover_hold_s)
                grace = float(P.startup_grace_s)

                max_rpm = int(P.max_rpm)
                acc = int(P.acc)
                kp, ki, kd = float(P.kp), float(P.ki), float(P.kd)
                i_lim = float(P.integrator_limit)

            angle = kal.update(angle_acc, gyro_rate, dt)

            # ---- Gestión de caída con gracia + recuperación ----
            in_grace = (now - grace_start) < grace

            if not fallen:
                if (not in_grace) and (abs(angle) > safe):
                    fallen = True
                    recover_ok_since = None
                    pid.reset()
                    # parar y deshabilitar inmediato
                    motors_stop_and_disable(ser)
            else:
                # fallen=True: mirar si vuelve a estar "estable" para rearme
                if abs(angle) < rec:
                    if recover_ok_since is None:
                        recover_ok_since = now
                    elif (now - recover_ok_since) >= hold:
                        # rearme
                        fallen = False
                        recover_ok_since = None
                        pid.reset()
                        grace_start = now  # pequeña gracia tras rearme
                        # SOLO enable + velocidad 0
                        for a in (MOTOR1_ADDR, MOTOR2_ADDR):
                            mks_enable(ser, a, True)
                        for a in (MOTOR1_ADDR, MOTOR2_ADDR):
                            mks_speed_signed(ser, a, 0, acc)
                else:
                    recover_ok_since = None

            out = 0.0
            if not fallen:
                err = 0.0 - angle
                out = pid.step(err, dt, kp, ki, kd, i_lim)
                out = clamp(out, -max_rpm, +max_rpm)
                mks_speed_signed(ser, MOTOR1_ADDR, M1_SIGN * out, acc)
                mks_speed_signed(ser, MOTOR2_ADDR, M2_SIGN * out, acc)

            with _state_lock:
                telemetry["angle_deg"] = float(angle)
                telemetry["gyro_dps"] = float(gyro_rate)
                telemetry["pid_out"] = float(out)
                telemetry["fallen"] = bool(fallen)

            elapsed = time.monotonic() - now
            sleep_s = dt_target - elapsed
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    finally:
        try:
            motors_stop_and_disable(ser)
        finally:
            try:
                ser.close()
            except Exception:
                pass
            try:
                bus.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
