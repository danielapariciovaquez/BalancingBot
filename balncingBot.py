#!/usr/bin/env python3
import time
import threading
import signal
import sys
from dataclasses import dataclass, asdict

import serial
from flask import Flask, jsonify, Response, request

# ===================== CONFIG =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05
INTER_FRAME_DELAY_S = 0.004

MOTOR1_ADDR = 0x01
MOTOR2_ADDR = 0x02

DEFAULT_ACC = 50          # 0..255 (byte final del comando F6)
DEFAULT_MAX_RPM = 200     # límite software
# =================================================

app = Flask(__name__)
stop_evt = threading.Event()

@dataclass
class MotorParams:
    enabled: bool = False
    rpm_cmd: int = 0              # RPM firmada (aplica a ambos)
    acc: int = DEFAULT_ACC        # 0..255
    max_rpm: int = DEFAULT_MAX_RPM
    invert_m1: bool = False
    invert_m2: bool = True        # típico: un lado invertido

params = MotorParams()
params_lock = threading.Lock()

@dataclass
class MotorState:
    last_tx_ts: float = 0.0
    last_rpm_m1: int = 0
    last_rpm_m2: int = 0
    ok: bool = True
    err: str = ""

state = MotorState()
state_lock = threading.Lock()

# ---------- Protocolo MKS RS485 ----------
def crc_sum8(frame_wo_crc: bytes) -> int:
    return sum(frame_wo_crc) & 0xFF

def build_frame(addr: int, cmd: int, payload: bytes) -> bytes:
    # Frame: FA addr cmd payload... crc
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([crc_sum8(base)])

def send_frame(ser: serial.Serial, addr: int, cmd: int, payload: bytes) -> None:
    fr = build_frame(addr, cmd, payload)
    ser.write(fr)
    ser.flush()
    time.sleep(INTER_FRAME_DELAY_S)

def cmd_enable(ser: serial.Serial, addr: int, enable: bool) -> None:
    # F3: 01 enable, 00 disable
    send_frame(ser, addr, 0xF3, bytes([0x01 if enable else 0x00]))

def cmd_run_rpm(ser: serial.Serial, addr: int, rpm: int, acc: int) -> None:
    # F6: [rpm_hi rpm_lo acc]
    # rpm: int16 signed, big-endian
    rpm16 = rpm & 0xFFFF
    payload = bytes([(rpm16 >> 8) & 0xFF, rpm16 & 0xFF, acc & 0xFF])
    send_frame(ser, addr, 0xF6, payload)

# ---------- Control loop ----------
def motor_loop():
    global params
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_S)
    except Exception as e:
        with state_lock:
            state.ok = False
            state.err = f"Serial open failed: {repr(e)}"
        return

    last_enabled = None
    last_sent_m1 = None
    last_sent_m2 = None

    try:
        while not stop_evt.is_set():
            with params_lock:
                enabled = bool(params.enabled)
                rpm_cmd = int(params.rpm_cmd)
                acc = int(params.acc)
                max_rpm = int(params.max_rpm)
                inv1 = bool(params.invert_m1)
                inv2 = bool(params.invert_m2)

            # límites duros
            if acc < 0: acc = 0
            if acc > 255: acc = 255
            if max_rpm < 0: max_rpm = 0
            if rpm_cmd > max_rpm: rpm_cmd = max_rpm
            if rpm_cmd < -max_rpm: rpm_cmd = -max_rpm

            rpm_m1 = -rpm_cmd if inv1 else rpm_cmd
            rpm_m2 = -rpm_cmd if inv2 else rpm_cmd

            try:
                # enable/disable sólo si cambia
                if last_enabled is None or enabled != last_enabled:
                    cmd_enable(ser, MOTOR1_ADDR, enabled)
                    cmd_enable(ser, MOTOR2_ADDR, enabled)
                    last_enabled = enabled
                    # forzar reenvío de rpm tras enable
                    last_sent_m1 = None
                    last_sent_m2 = None

                if enabled:
                    # manda RPM si cambió
                    if last_sent_m1 is None or rpm_m1 != last_sent_m1:
                        cmd_run_rpm(ser, MOTOR1_ADDR, rpm_m1, acc)
                        last_sent_m1 = rpm_m1
                    if last_sent_m2 is None or rpm_m2 != last_sent_m2:
                        cmd_run_rpm(ser, MOTOR2_ADDR, rpm_m2, acc)
                        last_sent_m2 = rpm_m2
                else:
                    # si está deshabilitado, garantiza 0rpm una vez
                    if last_sent_m1 != 0:
                        cmd_run_rpm(ser, MOTOR1_ADDR, 0, 0)
                        last_sent_m1 = 0
                    if last_sent_m2 != 0:
                        cmd_run_rpm(ser, MOTOR2_ADDR, 0, 0)
                        last_sent_m2 = 0

                with state_lock:
                    state.last_tx_ts = time.time()
                    state.last_rpm_m1 = int(rpm_m1 if enabled else 0)
                    state.last_rpm_m2 = int(rpm_m2 if enabled else 0)
                    state.ok = True
                    state.err = ""

            except Exception as e:
                with state_lock:
                    state.ok = False
                    state.err = f"RS485 send failed: {repr(e)}"

            time.sleep(0.02)  # ~50 Hz

    finally:
        # salida segura
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

# ---------- API ----------
@app.get("/api/status")
def api_status():
    with params_lock:
        p = asdict(params)
    with state_lock:
        s = asdict(state)
    return jsonify({"params": p, "state": s})

@app.post("/api/params")
def api_params():
    data = request.get_json(force=True, silent=True) or {}
    with params_lock:
        if "enabled" in data:
            params.enabled = bool(data["enabled"])
        if "rpm_cmd" in data:
            params.rpm_cmd = int(data["rpm_cmd"])
        if "acc" in data:
            params.acc = int(data["acc"])
        if "max_rpm" in data:
            params.max_rpm = int(data["max_rpm"])
        if "invert_m1" in data:
            params.invert_m1 = bool(data["invert_m1"])
        if "invert_m2" in data:
            params.invert_m2 = bool(data["invert_m2"])
    return jsonify({"ok": True})

@app.get("/")
def index():
    html = """<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>MKS SERVO RS485 — Motor Control</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 20px; max-width: 900px; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 14px; margin: 12px 0; }
    .row { display:flex; gap:12px; align-items:center; flex-wrap: wrap; margin: 10px 0; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .ok { color: #0a7; font-weight: 700; }
    .bad { color: #c22; font-weight: 700; }
    input[type=range] { width: 420px; }
    .k { width: 140px; color:#555; }
    button { padding: 6px 10px; }
  </style>
</head>
<body>
  <h2>MKS SERVO42D — RS485 control (2 motores)</h2>

  <div class="card">
    <div class="row">
      <div class="k">Estado</div>
      <div id="status" class="mono">...</div>
      <div class="k">TX ts</div>
      <div id="ts" class="mono">...</div>
    </div>
    <div class="row">
      <div class="k">RPM M1</div><div id="m1" class="mono">-</div>
      <div class="k">RPM M2</div><div id="m2" class="mono">-</div>
    </div>
    <div class="row">
      <div class="k">Error</div><div id="err" class="mono">-</div>
    </div>
  </div>

  <div class="card">
    <div class="row">
      <div class="k">Enable</div>
      <input id="en" type="checkbox">
      <button id="btnStop">STOP (0rpm + disable)</button>
    </div>

    <div class="row">
      <div class="k">RPM cmd</div>
      <input id="rpm" type="range" min="-200" max="200" step="1" value="0">
      <div id="rpmv" class="mono">0</div>
    </div>

    <div class="row">
      <div class="k">ACC (0..255)</div>
      <input id="acc" type="range" min="0" max="255" step="1" value="50">
      <div id="accv" class="mono">50</div>
    </div>

    <div class="row">
      <div class="k">MAX_RPM</div>
      <input id="maxrpm" type="range" min="0" max="1000" step="1" value="200">
      <div id="maxrpmv" class="mono">200</div>
    </div>

    <div class="row">
      <label><input id="inv1" type="checkbox"> Invert M1</label>
      <label><input id="inv2" type="checkbox" checked> Invert M2</label>
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

function bindRange(id, outId, key){
  const s = document.getElementById(id);
  const o = document.getElementById(outId);
  const updateLabel = ()=>{ o.textContent = String(s.value); };
  const send = async ()=>{
    updateLabel();
    const payload = {};
    payload[key] = Number(s.value);
    await postJSON("/api/params", payload);
  };
  s.addEventListener("input", updateLabel);
  s.addEventListener("change", send);
  updateLabel();
}

bindRange("rpm","rpmv","rpm_cmd");
bindRange("acc","accv","acc");
bindRange("maxrpm","maxrpmv","max_rpm");

document.getElementById("en").addEventListener("change", async (e)=>{
  await postJSON("/api/params", {enabled: e.target.checked});
});

document.getElementById("inv1").addEventListener("change", async (e)=>{
  await postJSON("/api/params", {invert_m1: e.target.checked});
});
document.getElementById("inv2").addEventListener("change", async (e)=>{
  await postJSON("/api/params", {invert_m2: e.target.checked});
});

document.getElementById("btnStop").addEventListener("click", async ()=>{
  await postJSON("/api/params", {rpm_cmd: 0, enabled: false});
  document.getElementById("rpm").value = 0;
  document.getElementById("rpmv").textContent = "0";
  document.getElementById("en").checked = false;
});

async function tick(){
  try{
    const r = await fetch("/api/status", {cache:"no-store"});
    const d = await r.json();
    const st = d.state;

    const s = document.getElementById("status");
    if(st.ok){
      s.textContent = "OK";
      s.className = "mono ok";
      document.getElementById("err").textContent = "-";
    }else{
      s.textContent = "ERROR";
      s.className = "mono bad";
      document.getElementById("err").textContent = st.err || "-";
    }

    document.getElementById("ts").textContent = fmt(st.last_tx_ts, 3);
    document.getElementById("m1").textContent = String(st.last_rpm_m1);
    document.getElementById("m2").textContent = String(st.last_rpm_m2);
  }catch(e){
    const s = document.getElementById("status");
    s.textContent = "HTTP ERROR";
    s.className = "mono bad";
    document.getElementById("err").textContent = String(e);
  }
}

tick();
setInterval(tick, 200);
</script>
</body>
</html>
"""
    return Response(html, mimetype="text/html")

def shutdown_handler(signum=None, frame=None):
    stop_evt.set()
    time.sleep(0.1)
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    t = threading.Thread(target=motor_loop, daemon=True)
    t.start()

    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

if __name__ == "__main__":
    main()
