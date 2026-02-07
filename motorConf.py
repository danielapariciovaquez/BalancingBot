#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import atexit
import signal
import threading
import time

import serial
from flask import Flask, request, Response

# ===================== CONFIG =====================
RS485_PORT = "/dev/ttyUSB0"
RS485_BAUD = 38400
RS485_TIMEOUT_S = 0.05
INTER_FRAME_DELAY_S = 0.003  # un pelín más conservador

MOTOR1_ADDR = 0x01
MOTOR2_ADDR = 0x02

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

# Debounce server-side (además del cliente)
MIN_SPEED_SEND_DT = 0.05   # 50 ms
MIN_PID_SEND_DT   = 0.20   # 200 ms
# ==================================================

# ------------------ MKS helpers ------------------

def checksum_8bit(frame_wo_crc: bytes) -> int:
    return sum(frame_wo_crc) & 0xFF

def build_frame(addr: int, code: int, data: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum_8bit(base)])

def frame_enable(addr: int, enable: bool) -> bytes:
    return build_frame(addr, 0xF3, bytes([0x01 if enable else 0x00]))

def frame_speed(addr: int, rpm: int, acc: int) -> bytes:
    if not (0 <= acc <= 255):
        raise ValueError("ACC fuera de rango (0..255)")
    rpm = int(rpm)
    speed = abs(rpm)
    if not (0 <= speed <= 3000):
        raise ValueError("RPM fuera de rango (-3000..3000)")

    dir_bit = 1 if rpm < 0 else 0
    byte4 = ((dir_bit & 1) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF
    return build_frame(addr, 0xF6, bytes([byte4, byte5, acc & 0xFF]))

def frame_pid(addr: int, mode: str, kp: int, ki: int, kd: int, kv: int) -> bytes:
    if mode not in ("vFOC", "CLOSE"):
        raise ValueError("PID mode debe ser 'vFOC' o 'CLOSE'")
    vals = [int(kp), int(ki), int(kd), int(kv)]
    for name, v in zip(("kp", "ki", "kd", "kv"), vals):
        if not (0 <= v <= 1024):
            raise ValueError(f"{name} fuera de rango (0..1024)")

    code = 0x96 if mode == "vFOC" else 0x97
    kp, ki, kd, kv = vals
    data = bytes([
        (kp >> 8) & 0xFF, kp & 0xFF,
        (ki >> 8) & 0xFF, ki & 0xFF,
        (kd >> 8) & 0xFF, kd & 0xFF,
        (kv >> 8) & 0xFF, kv & 0xFF,
    ])
    return build_frame(addr, code, data)

def frame_resp_active(addr: int, respond: bool, active: bool) -> bytes:
    # 8C XX YY (respond, active)
    return build_frame(addr, 0x8C, bytes([0x01 if respond else 0x00, 0x01 if active else 0x00]))

def frame_modbus_enable(addr: int, enable: bool) -> bytes:
    # 8E 00 desactiva MODBUS-RTU; 8E 01 lo activa
    return build_frame(addr, 0x8E, bytes([0x01 if enable else 0x00]))

def frame_work_mode(addr: int, mode: int) -> bytes:
    # 82 mode (ej: 0x05 SR_vFOC)
    return build_frame(addr, 0x82, bytes([mode & 0xFF]))

# ------------------ RS485 driver ------------------

class MksBus:
    def __init__(self, port: str, baud: int, timeout_s: float):
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=timeout_s)
        self._lock = threading.Lock()

    def _drain_rx(self):
        # Vacía cualquier byte pendiente (eco/ruido) antes de enviar
        n = self._ser.in_waiting
        if n:
            try:
                self._ser.read(n)
            except Exception:
                pass

    def send(self, payload: bytes):
        with self._lock:
            self._drain_rx()
            self._ser.write(payload)
            self._ser.flush()
        time.sleep(INTER_FRAME_DELAY_S)

    def close(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()

# ------------------ Web UI ------------------

app = Flask(__name__)
bus: MksBus | None = None

# Estado mínimo para no reenviar si no cambia
_last = {
    "enable": None,
    "rpm": None,
    "acc": None,
    "pid": None,     # tuple (mode,kp,ki,kd,kv)
    "t_speed": 0.0,
    "t_pid": 0.0,
}

HTML = r"""<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>MKS RS485 M1+M2</title>
<style>
  body { font-family: sans-serif; margin: 18px; max-width: 820px; }
  .card { border:1px solid #ddd; border-radius: 12px; padding: 14px; margin: 12px 0; }
  .row { display:grid; grid-template-columns: 180px 1fr 90px; gap: 12px; align-items:center; margin: 10px 0; }
  input[type="range"] { width: 100%; }
  .small { color:#555; font-size: 12px; }
  .ok { color:#0a0; }
  .bad { color:#a00; }
  select { padding: 6px; }
  .toggle { display:flex; align-items:center; gap:10px; }
</style>
</head>
<body>
<h2>MKS SERVO RS485 – Control común M1 + M2</h2>
<div class="small">Sin botones: envía en caliente.</div>

<div class="card">
  <h3>Enable</h3>
  <div class="toggle">
    <input id="en" type="checkbox" onchange="sendEnable()">
    <label for="en">Motores habilitados (F3)</label>
  </div>
  <div id="enStatus" class="small"></div>
</div>

<div class="card">
  <h3>Velocidad común</h3>
  <div class="row">
    <div>ACC (0..255)</div>
    <input id="acc" type="range" min="0" max="255" value="10" oninput="onSpeed()">
    <div><span id="accv">10</span></div>
  </div>
  <div class="row">
    <div>RPM (-3000..3000)</div>
    <input id="rpm" type="range" min="-3000" max="3000" value="0" oninput="onSpeed()">
    <div><span id="rpmv">0</span></div>
  </div>
  <div id="spStatus" class="small"></div>
</div>

<div class="card">
  <h3>PID común (manda speed=0 antes)</h3>
  <div class="row">
    <div>Modo</div>
    <select id="mode" onchange="onPid()">
      <option value="vFOC">vFOC (0x96)</option>
      <option value="CLOSE">CLOSE (0x97)</option>
    </select>
    <div></div>
  </div>
  <div class="row">
    <div>Kp (0..1024)</div>
    <input id="kp" type="range" min="0" max="1024" value="220" oninput="onPid()">
    <div><span id="kpv">220</span></div>
  </div>
  <div class="row">
    <div>Ki (0..1024)</div>
    <input id="ki" type="range" min="0" max="1024" value="100" oninput="onPid()">
    <div><span id="kiv">100</span></div>
  </div>
  <div class="row">
    <div>Kd (0..1024)</div>
    <input id="kd" type="range" min="0" max="1024" value="270" oninput="onPid()">
    <div><span id="kdv">270</span></div>
  </div>
  <div class="row">
    <div>Kv (0..1024)</div>
    <input id="kv" type="range" min="0" max="1024" value="320" oninput="onPid()">
    <div><span id="kvv">320</span></div>
  </div>
  <div id="pidStatus" class="small"></div>
</div>

<script>
let tSpeed=null, tPid=null;

function qs(id){ return document.getElementById(id); }

async function post(path, body){
  const r = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body)});
  const txt = await r.text();
  if(!r.ok) throw new Error(txt || ("HTTP " + r.status));
  return txt;
}

async function sendEnable(){
  try{
    const en = qs("en").checked;
    const t = await post("/api/enable", {enable: en});
    qs("enStatus").innerHTML = '<span class="ok">' + t + '</span>';
  }catch(e){
    qs("enStatus").innerHTML = '<span class="bad">' + e.message + '</span>';
  }
}

function onSpeed(){
  qs("accv").textContent = qs("acc").value;
  qs("rpmv").textContent = qs("rpm").value;
  if(tSpeed) clearTimeout(tSpeed);
  tSpeed = setTimeout(sendSpeed, 80);
}

async function sendSpeed(){
  try{
    const body = { rpm: parseInt(qs("rpm").value), acc: parseInt(qs("acc").value) };
    const t = await post("/api/speed", body);
    qs("spStatus").innerHTML = '<span class="ok">' + t + '</span>';
  }catch(e){
    qs("spStatus").innerHTML = '<span class="bad">' + e.message + '</span>';
  }
}

function onPid(){
  qs("kpv").textContent = qs("kp").value;
  qs("kiv").textContent = qs("ki").value;
  qs("kdv").textContent = qs("kd").value;
  qs("kvv").textContent = qs("kv").value;
  if(tPid) clearTimeout(tPid);
  tPid = setTimeout(sendPid, 150);
}

async function sendPid(){
  try{
    const body = {
      mode: qs("mode").value,
      kp: parseInt(qs("kp").value),
      ki: parseInt(qs("ki").value),
      kd: parseInt(qs("kd").value),
      kv: parseInt(qs("kv").value),
    };
    const t = await post("/api/pid", body);
    qs("pidStatus").innerHTML = '<span class="ok">' + t + '</span>';
  }catch(e){
    qs("pidStatus").innerHTML = '<span class="bad">' + e.message + '</span>';
  }
}

window.addEventListener("load", () => { onSpeed(); onPid(); });
</script>
</body>
</html>
"""

@app.get("/")
def index():
    return Response(HTML, mimetype="text/html")

def init_motor(addr: int):
    # Secuencia robusta para evitar “no hace nada”
    bus.send(frame_modbus_enable(addr, False))      # por si estuviera en MODBUS-RTU
    bus.send(frame_resp_active(addr, True, True))   # responder + activo (debug)
    bus.send(frame_work_mode(addr, 0x05))           # SR_vFOC (bus control)

@app.post("/api/enable")
def api_enable():
    enable = bool(request.get_json(force=True).get("enable"))
    if _last["enable"] is enable:
        return Response("OK (sin cambios)", mimetype="text/plain")

    bus.send(frame_enable(MOTOR1_ADDR, enable))
    bus.send(frame_enable(MOTOR2_ADDR, enable))
    _last["enable"] = enable
    return Response(f"OK: {'ENABLE' if enable else 'DISABLE'} M1+M2", mimetype="text/plain")

@app.post("/api/speed")
def api_speed():
    data = request.get_json(force=True)
    rpm = int(data.get("rpm", 0))
    acc = int(data.get("acc", 0))

    now = time.time()
    if now - _last["t_speed"] < MIN_SPEED_SEND_DT:
        return Response("OK (rate-limited)", mimetype="text/plain")

    if _last["rpm"] == rpm and _last["acc"] == acc:
        return Response("OK (sin cambios)", mimetype="text/plain")

    bus.send(frame_speed(MOTOR1_ADDR, rpm, acc))
    bus.send(frame_speed(MOTOR2_ADDR, rpm, acc))

    _last["rpm"] = rpm
    _last["acc"] = acc
    _last["t_speed"] = now
    return Response(f"OK: SPEED M1+M2 rpm={rpm} acc={acc}", mimetype="text/plain")

@app.post("/api/pid")
def api_pid():
    data = request.get_json(force=True)
    mode = str(data.get("mode", "vFOC"))
    kp = int(data.get("kp", 0))
    ki = int(data.get("ki", 0))
    kd = int(data.get("kd", 0))
    kv = int(data.get("kv", 0))
    pid_tuple = (mode, kp, ki, kd, kv)

    now = time.time()
    if now - _last["t_pid"] < MIN_PID_SEND_DT:
        return Response("OK (rate-limited)", mimetype="text/plain")

    if _last["pid"] == pid_tuple:
        return Response("OK (sin cambios)", mimetype="text/plain")

    # Parada previa obligatoria
    bus.send(frame_speed(MOTOR1_ADDR, 0, 0))
    bus.send(frame_speed(MOTOR2_ADDR, 0, 0))
    time.sleep(0.05)  # margen mínimo para que el driver “asiente” la parada

    bus.send(frame_pid(MOTOR1_ADDR, mode, kp, ki, kd, kv))
    bus.send(frame_pid(MOTOR2_ADDR, mode, kp, ki, kd, kv))

    _last["pid"] = pid_tuple
    _last["t_pid"] = now
    return Response(f"OK: PID {mode} M1+M2 (antes speed=0)", mimetype="text/plain")

# -------------- shutdown: disable --------------

_shutdown_once = False
def shutdown_disable():
    global _shutdown_once
    if _shutdown_once:
        return
    _shutdown_once = True
    try:
        if bus is not None:
            bus.send(frame_enable(MOTOR1_ADDR, False))
            bus.send(frame_enable(MOTOR2_ADDR, False))
    except Exception:
        pass

def _sig_handler(signum, frame):
    shutdown_disable()
    raise SystemExit(0)

def main():
    global bus
    bus = MksBus(RS485_PORT, RS485_BAUD, RS485_TIMEOUT_S)

    # Inicialización “anti no-responde”
    init_motor(MOTOR1_ADDR)
    init_motor(MOTOR2_ADDR)

    atexit.register(shutdown_disable)
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    app.run(host=HTTP_HOST, port=HTTP_PORT, debug=False, threaded=True)

if __name__ == "__main__":
    main()
