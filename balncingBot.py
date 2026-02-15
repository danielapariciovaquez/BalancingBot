#!/usr/bin/env python3
import time
import threading
import signal
import sys
import math
from dataclasses import dataclass, asdict

from smbus2 import SMBus
import serial
from flask import Flask, jsonify, Response, request

# ===================== HW CONFIG =====================
# IMU (MPU6050 / GY-521)
I2C_BUS = 1
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_SCALE = 16384.0  # LSB/g (±2g)
GYRO_SCALE  = 131.0    # LSB/(°/s) (±250°/s)

# RS485 (MKS SERVO42D)
RS485_PORT = "/dev/ttyUSB0"
RS485_BAUD = 38400
RS485_TIMEOUT_S = 0.05
INTER_FRAME_DELAY_S = 0.004

MOTOR1_ADDR = 0x01
MOTOR2_ADDR = 0x02
MOTOR2_INVERT = True   # un motor invertido

# SOLO comandos F3 (enable) y F6 (velocidad)
DEFAULT_ACC_BYTE = 50  # byte "acc" dentro de F6 (0..255) - forma parte de F6
# =====================================================

# ===================== LOOP CONFIG =====================
IMU_HZ = 200
CTRL_HZ = 100
SETPOINT_DEG = 0.0

# Reafirmar enable periódicamente para evitar perder el único pulso
F3_REASSERT_S = 0.5  # 2 Hz
# =======================================================

app = Flask(__name__)
stop_evt = threading.Event()

# -------------------- Helpers --------------------
def _to_int16(v: int) -> int:
    return v - 0x10000 if (v & 0x8000) else v

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def lpf_alpha(dt: float, tau: float) -> float:
    if tau <= 0.0:
        return 1.0
    return dt / (tau + dt)

# -------------------- RS485 protocol (ONLY F3 + F6) --------------------
def crc_sum8(frame_wo_crc: bytes) -> int:
    return sum(frame_wo_crc) & 0xFF

def build_frame(addr: int, cmd: int, payload: bytes) -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([crc_sum8(base)])

def send_frame(ser: serial.Serial, addr: int, cmd: int, payload: bytes) -> bytes:
    fr = build_frame(addr, cmd, payload)
    ser.write(fr)
    ser.flush()
    time.sleep(INTER_FRAME_DELAY_S)
    return fr

def cmd_enable(ser: serial.Serial, addr: int, enable: bool) -> bytes:
    # F3: [0x01] enable, [0x00] disable
    return send_frame(ser, addr, 0xF3, bytes([0x01 if enable else 0x00]))

def cmd_run_rpm(ser: serial.Serial, addr: int, rpm: int, acc_byte: int) -> bytes:
    # F6: [rpm_hi rpm_lo acc]
    rpm16 = rpm & 0xFFFF  # int16 firmado en 2's complement
    payload = bytes([(rpm16 >> 8) & 0xFF, rpm16 & 0xFF, acc_byte & 0xFF])
    return send_frame(ser, addr, 0xF6, payload)

# -------------------- Shared params/state --------------------
@dataclass
class Tunables:
    # filtros
    tau_acc: float = 0.10
    tau_gyro: float = 0.02
    alpha_comp: float = 0.98

    # PID
    kp: float = 20.0
    ki: float = 0.0
    kd: float = 0.5

    # límite
    max_rpm: int = 80

    # enable
    enabled: bool = False

tun = Tunables()
tun_lock = threading.Lock()

@dataclass
class Runtime:
    # señales filtradas
    az_g: float = 0.0
    gz_dps: float = 0.0

    # ángulo
    angle_acc_deg: float = 0.0     # acos(Az_f) (0..180)
    angle_fused_deg: float = 0.0   # fusionado (antes de offset)
    angle_zero_offset_deg: float = 0.0
    angle_out_deg: float = 0.0     # (fused - offset), entra al PID

    # PID
    err_deg: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    u_rpm: float = 0.0

    # salida
    rpm_m1: int = 0
    rpm_m2: int = 0

    # estados
    ok_imu: bool = False
    err_imu: str = ""
    ok_rs485: bool = False
    err_rs485: str = ""

    ts: float = 0.0
    dt_ctrl: float = 0.0

    # telemetría TX (para comprobar enable)
    last_f3_ts: float = 0.0
    last_f3_enable: bool = False
    last_f3_hex: str = ""

    last_f6_ts: float = 0.0
    last_f6_hex_m1: str = ""
    last_f6_hex_m2: str = ""

rt = Runtime()
rt_lock = threading.Lock()

# Botón “hacer cero”
set_zero_evt = threading.Event()

# -------------------- IMU thread (Z only) --------------------
def imu_thread():
    az_f = 0.0
    gz_f = 0.0
    angle_fused = 0.0

    try:
        bus = SMBus(I2C_BUS)
    except Exception as e:
        with rt_lock:
            rt.ok_imu = False
            rt.err_imu = f"SMBus open failed: {repr(e)}"
        return

    try:
        bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    except Exception as e:
        with rt_lock:
            rt.ok_imu = False
            rt.err_imu = f"MPU init failed: {repr(e)}"
        try:
            bus.close()
        except Exception:
            pass
        return

    t_prev = time.time()
    period = 1.0 / float(IMU_HZ)

    while not stop_evt.is_set():
        t0 = time.time()
        dt = t0 - t_prev
        if dt <= 0.0:
            dt = 1.0 / IMU_HZ
        t_prev = t0

        try:
            data = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 14)
            az = _to_int16((data[4] << 8) | data[5]) / ACCEL_SCALE
            gz = _to_int16((data[12] << 8) | data[13]) / GYRO_SCALE

            with tun_lock:
                tau_acc = float(tun.tau_acc)
                tau_gyro = float(tun.tau_gyro)
                alpha = float(tun.alpha_comp)

            a_acc = lpf_alpha(dt, tau_acc)
            a_gyr = lpf_alpha(dt, tau_gyro)

            az_f = az_f + a_acc * (az - az_f)
            gz_f = gz_f + a_gyr * (gz - gz_f)

            az_c = clamp(az_f, -1.0, 1.0)
            angle_acc = math.degrees(math.acos(az_c))

            angle_fused = alpha * (angle_fused + gz_f * dt) + (1.0 - alpha) * angle_acc

            if set_zero_evt.is_set():
                set_zero_evt.clear()
                with rt_lock:
                    rt.angle_zero_offset_deg = float(angle_fused)

            with rt_lock:
                rt.az_g = float(az_f)
                rt.gz_dps = float(gz_f)
                rt.angle_acc_deg = float(angle_acc)
                rt.angle_fused_deg = float(angle_fused)
                rt.ok_imu = True
                rt.err_imu = ""

        except Exception as e:
            with rt_lock:
                rt.ok_imu = False
                rt.err_imu = repr(e)
            time.sleep(0.05)

        loop_dt = time.time() - t0
        sleep_s = period - loop_dt
        if sleep_s > 0:
            time.sleep(sleep_s)

    try:
        bus.close()
    except Exception:
        pass

# -------------------- Control + RS485 thread --------------------
def control_rs485_thread():
    try:
        ser = serial.Serial(RS485_PORT, RS485_BAUD, timeout=RS485_TIMEOUT_S)
    except Exception as e:
        with rt_lock:
            rt.ok_rs485 = False
            rt.err_rs485 = f"Serial open failed: {repr(e)}"
        return

    i_term = 0.0
    prev_err = 0.0
    t_prev = time.time()
    period = 1.0 / float(CTRL_HZ)

    last_enabled = None
    last_f3_reassert = 0.0

    try:
        while not stop_evt.is_set():
            t0 = time.time()
            dt = t0 - t_prev
            if dt <= 0.0:
                dt = 1.0 / CTRL_HZ
            t_prev = t0

            with tun_lock:
                enabled = bool(tun.enabled)
                kp = float(tun.kp)
                ki = float(tun.ki)
                kd = float(tun.kd)
                max_rpm = int(tun.max_rpm)

            with rt_lock:
                fused = float(rt.angle_fused_deg)
                zoff = float(rt.angle_zero_offset_deg)
                ok_imu = bool(rt.ok_imu)

            angle_out = fused - zoff
            err = SETPOINT_DEG - angle_out

            # PID
            if (not enabled) or (not ok_imu):
                i_term = 0.0
                prev_err = err
                u = 0.0
                p = 0.0
                d = 0.0
            else:
                p = kp * err
                i_term += ki * err * dt
                i_term = clamp(i_term, -float(max_rpm), float(max_rpm))
                derr = (err - prev_err) / dt
                d = kd * derr
                prev_err = err
                u = clamp(p + i_term + d, -float(max_rpm), float(max_rpm))

            rpm_cmd = int(round(u))
            rpm_m1 = rpm_cmd
            rpm_m2 = -rpm_cmd if MOTOR2_INVERT else rpm_cmd

            now = time.time()

            # RS485: SOLO F3 y F6
            try:
                need_f3 = (last_enabled is None) or (enabled != last_enabled)
                if enabled and (now - last_f3_reassert) >= F3_REASSERT_S:
                    need_f3 = True

                if need_f3:
                    fr1 = cmd_enable(ser, MOTOR1_ADDR, enabled)
                    fr2 = cmd_enable(ser, MOTOR2_ADDR, enabled)
                    last_enabled = enabled
                    last_f3_reassert = now

                    f3hex = fr1.hex(" ") + " | " + fr2.hex(" ")
                    print(f"TX F3 enable={enabled}: {f3hex}")

                    with rt_lock:
                        rt.last_f3_ts = now
                        rt.last_f3_enable = bool(enabled)
                        rt.last_f3_hex = f3hex

                if enabled and ok_imu:
                    fr1 = cmd_run_rpm(ser, MOTOR1_ADDR, rpm_m1, DEFAULT_ACC_BYTE)
                    fr2 = cmd_run_rpm(ser, MOTOR2_ADDR, rpm_m2, DEFAULT_ACC_BYTE)
                else:
                    fr1 = cmd_run_rpm(ser, MOTOR1_ADDR, 0, 0)
                    fr2 = cmd_run_rpm(ser, MOTOR2_ADDR, 0, 0)

                with rt_lock:
                    rt.ok_rs485 = True
                    rt.err_rs485 = ""
                    rt.rpm_m1 = int(rpm_m1 if (enabled and ok_imu) else 0)
                    rt.rpm_m2 = int(rpm_m2 if (enabled and ok_imu) else 0)

                    rt.last_f6_ts = now
                    rt.last_f6_hex_m1 = fr1.hex(" ")
                    rt.last_f6_hex_m2 = fr2.hex(" ")

            except Exception as e:
                with rt_lock:
                    rt.ok_rs485 = False
                    rt.err_rs485 = f"RS485 send failed: {repr(e)}"

            with rt_lock:
                rt.ts = t0
                rt.dt_ctrl = dt
                rt.angle_out_deg = float(angle_out)
                rt.err_deg = float(err)
                rt.p_term = float(p)
                rt.i_term = float(i_term)
                rt.d_term = float(d)
                rt.u_rpm = float(u)

            loop_dt = time.time() - t0
            sleep_s = period - loop_dt
            if sleep_s > 0:
                time.sleep(sleep_s)

    finally:
        # salida segura: SOLO F6 0 y F3 disable
        try:
            cmd_run_rpm(ser, MOTOR1_ADDR, 0, 0)
            cmd_run_rpm(ser, MOTOR2_ADDR, 0, 0)
            cmd_enable(ser, MOTOR1_ADDR, False)
            cmd_enable(ser, MOTOR2_ADDR, False)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

# -------------------- Web API --------------------
@app.get("/api/state")
def api_state():
    with tun_lock:
        t = asdict(tun)
    with rt_lock:
        r = asdict(rt)
    return jsonify({"tun": t, "rt": r})

@app.post("/api/tun")
def api_tun():
    data = request.get_json(force=True, silent=True) or {}
    with tun_lock:
        for k in ("tau_acc", "tau_gyro", "alpha_comp", "kp", "ki", "kd"):
            if k in data:
                setattr(tun, k, float(data[k]))
        if "max_rpm" in data:
            tun.max_rpm = int(data["max_rpm"])
        if "enabled" in data:
            tun.enabled = bool(data["enabled"])
    return jsonify({"ok": True})

@app.post("/api/zero")
def api_zero():
    set_zero_evt.set()
    return jsonify({"ok": True})

@app.get("/")
def index():
    html = """<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Balancing Z — PID + RS485</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 20px; max-width: 980px; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 14px; margin: 12px 0; }
    .row { display:flex; gap:12px; align-items:center; flex-wrap: wrap; margin: 10px 0; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .ok { color: #0a7; font-weight: 700; }
    .bad { color: #c22; font-weight: 700; }
    input[type=range] { width: 420px; }
    .k { width: 180px; color:#555; }
    button { padding: 6px 10px; }
    .small { color:#666; font-size: 12px; }
  </style>
</head>
<body>
  <h2>Balancing (Z-only): filtros + PID + motores RS485</h2>

  <div class="card">
    <div class="row">
      <div class="k">IMU</div><div id="imuok" class="mono">...</div>
      <div class="k">RS485</div><div id="busok" class="mono">...</div>
      <div class="k">dt_ctrl</div><div id="dt" class="mono">...</div>
    </div>
    <div class="row">
      <div class="k">Ángulo (deg)</div><div id="ang" class="mono">-</div>
      <div class="k">RPM cmd</div><div id="rpm" class="mono">-</div>
    </div>
    <div class="row">
      <div class="k">Último F3</div><div id="f3" class="mono">-</div>
    </div>
    <div class="row">
      <div class="k">F6 M1</div><div id="f6m1" class="mono small">-</div>
    </div>
    <div class="row">
      <div class="k">F6 M2</div><div id="f6m2" class="mono small">-</div>
    </div>
    <div class="row">
      <div class="k">Err IMU</div><div id="eimu" class="mono">-</div>
    </div>
    <div class="row">
      <div class="k">Err RS485</div><div id="ebus" class="mono">-</div>
    </div>
  </div>

  <div class="card">
    <div class="row">
      <div class="k">Enable motores</div>
      <input id="en" type="checkbox">
      <button id="btnZero">Hacer cero (0°)</button>
    </div>

    <div class="row">
      <div class="k">MAX_RPM</div>
      <input id="maxrpm" type="range" min="0" max="1000" step="1" value="80">
      <div id="maxrpmv" class="mono">80</div>
    </div>
  </div>

  <div class="card">
    <div style="font-weight:700;margin-bottom:8px;">Filtros IMU (Z)</div>

    <div class="row">
      <div class="k">τ_acc (s)</div>
      <input id="tauacc" type="range" min="0" max="1.0" step="0.005" value="0.10">
      <div id="tauaccv" class="mono">0.10</div>
    </div>

    <div class="row">
      <div class="k">τ_gyro (s)</div>
      <input id="taugyr" type="range" min="0" max="0.5" step="0.002" value="0.02">
      <div id="taugyrv" class="mono">0.02</div>
    </div>

    <div class="row">
      <div class="k">alpha</div>
      <input id="alpha" type="range" min="0" max="1.0" step="0.001" value="0.98">
      <div id="alphav" class="mono">0.98</div>
    </div>
  </div>

  <div class="card">
    <div style="font-weight:700;margin-bottom:8px;">PID (setpoint = 0°)</div>

    <div class="row">
      <div class="k">Kp</div>
      <input id="kp" type="range" min="0" max="300" step="0.1" value="20.0">
      <div id="kpv" class="mono">20.0</div>
    </div>

    <div class="row">
      <div class="k">Ki</div>
      <input id="ki" type="range" min="0" max="300" step="0.1" value="0.0">
      <div id="kiv" class="mono">0.0</div>
    </div>

    <div class="row">
      <div class="k">Kd</div>
      <input id="kd" type="range" min="0" max="300" step="0.1" value="0.5">
      <div id="kdv" class="mono">0.5</div>
    </div>
  </div>

<script>
const fmt = (v, n=3) => (typeof v === "number" ? v.toFixed(n) : String(v));

async function postJSON(url, obj){
  await fetch(url, {
    method: "POST",
    headers: {"Content-Type":"application/json"},
    body: JSON.stringify(obj)
  });
}

function bindRange(id, outId, key, castFloat=true){
  const s = document.getElementById(id);
  const o = document.getElementById(outId);
  const updateLabel = ()=>{ o.textContent = String(s.value); };
  const send = async ()=>{
    updateLabel();
    const payload = {};
    payload[key] = castFloat ? Number(s.value) : parseInt(s.value, 10);
    await postJSON("/api/tun", payload);
  };
  s.addEventListener("input", updateLabel);
  s.addEventListener("change", send);
  updateLabel();
}

bindRange("maxrpm","maxrpmv","max_rpm", false);
bindRange("tauacc","tauaccv","tau_acc", true);
bindRange("taugyr","taugyrv","tau_gyro", true);
bindRange("alpha","alphav","alpha_comp", true);
bindRange("kp","kpv","kp", true);
bindRange("ki","kiv","ki", true);
bindRange("kd","kdv","kd", true);

document.getElementById("en").addEventListener("change", async (e)=>{
  await postJSON("/api/tun", {enabled: e.target.checked});
});

document.getElementById("btnZero").addEventListener("click", async ()=>{
  await fetch("/api/zero", {method:"POST"});
});

async function tick(){
  try{
    const r = await fetch("/api/state", {cache:"no-store"});
    const d = await r.json();
    const rt = d.rt;
    const tun = d.tun;

    document.getElementById("en").checked = !!tun.enabled;

    const imuok = document.getElementById("imuok");
    imuok.textContent = rt.ok_imu ? "OK" : "ERROR";
    imuok.className = "mono " + (rt.ok_imu ? "ok" : "bad");

    const busok = document.getElementById("busok");
    busok.textContent = rt.ok_rs485 ? "OK" : "ERROR";
    busok.className = "mono " + (rt.ok_rs485 ? "ok" : "bad");

    document.getElementById("dt").textContent = fmt(rt.dt_ctrl, 4);
    document.getElementById("ang").textContent = fmt(rt.angle_out_deg, 3);
    document.getElementById("rpm").textContent = String(Math.round(rt.u_rpm));

    document.getElementById("f3").textContent = rt.last_f3_hex || "-";
    document.getElementById("f6m1").textContent = rt.last_f6_hex_m1 || "-";
    document.getElementById("f6m2").textContent = rt.last_f6_hex_m2 || "-";

    document.getElementById("eimu").textContent = rt.err_imu || "-";
    document.getElementById("ebus").textContent = rt.err_rs485 || "-";

  }catch(e){
    const imuok = document.getElementById("imuok");
    imuok.textContent = "HTTP ERROR";
    imuok.className = "mono bad";
    document.getElementById("eimu").textContent = String(e);
  }
}

tick();
setInterval(tick, 200);
</script>
</body>
</html>
"""
    return Response(html, mimetype="text/html")

# -------------------- Shutdown --------------------
def shutdown_handler(signum=None, frame=None):
    stop_evt.set()
    time.sleep(0.2)
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    th_imu = threading.Thread(target=imu_thread, daemon=True)
    th_ctl = threading.Thread(target=control_rs485_thread, daemon=True)
    th_imu.start()
    th_ctl.start()

    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

if __name__ == "__main__":
    main()
