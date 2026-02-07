#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import atexit
import json
import signal
import threading
import time
from dataclasses import dataclass

import serial
from flask import Flask, request, Response

# ============================================================
# CONFIG
# ============================================================
RS485_PORT = "/dev/ttyUSB0"
RS485_BAUD = 38400
RS485_TIMEOUT_S = 0.05

MOTOR1_ADDR = 0x01
MOTOR2_ADDR = 0x02

# Inter-frame delay pequeño para no "aplastar" al driver
INTER_FRAME_DELAY_S = 0.002

# Web
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

# ============================================================
# RS485 / MKS helpers
# ============================================================

def checksum_8bit(frame_wo_crc: bytes) -> int:
    # CRC = (sum(bytes)) & 0xFF  (manual: "CHECKSUM 8bit") :contentReference[oaicite:5]{index=5}
    return sum(frame_wo_crc) & 0xFF

def build_frame(addr: int, code: int, data: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    crc = checksum_8bit(base)
    return base + bytes([crc])

def frame_enable(addr: int, enable: bool) -> bytes:
    # FA addr F3 enable CRC :contentReference[oaicite:6]{index=6}
    return build_frame(addr, 0xF3, bytes([0x01 if enable else 0x00]))

def frame_speed(addr: int, rpm: int, acc: int) -> bytes:
    """
    Speed mode F6 :contentReference[oaicite:7]{index=7}
      - dir: bit7 de Byte4 (0=CW, 1=CCW según manual dir 0/1; tú puedes mapear signos)
      - speed: 0..3000 codificado en 12 bits: high4 en (byte4 & 0x0F), low8 en byte5
      - acc: 0..255
    Aquí interpretamos:
      rpm >= 0 -> dir=0
      rpm <  0 -> dir=1, speed=abs(rpm)
    """
    if acc < 0 or acc > 255:
        raise ValueError("acc fuera de rango (0..255)")
    speed = abs(int(rpm))
    if speed < 0 or speed > 3000:
        raise ValueError("speed fuera de rango (0..3000)")

    dir_bit = 1 if rpm < 0 else 0

    byte4 = ((dir_bit & 0x01) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF
    data = bytes([byte4, byte5, acc & 0xFF])
    return build_frame(addr, 0xF6, data)

def frame_pid(addr: int, mode: str, kp: int, ki: int, kd: int, kv: int) -> bytes:
    """
    PID:
      - vFOC: code 0x96  (KP/KI/KD/KV uint16 big-endian) :contentReference[oaicite:8]{index=8}
      - CLOSE: code 0x97 (KP/KI/KD/KV uint16 big-endian) :contentReference[oaicite:9]{index=9}
    """
    if mode not in ("vFOC", "CLOSE"):
        raise ValueError("mode debe ser 'vFOC' o 'CLOSE'")
    for name, v in (("kp", kp), ("ki", ki), ("kd", kd), ("kv", kv)):
        if v < 0 or v > 1024:
            raise ValueError(f"{name} fuera de rango (0..1024)")

    code = 0x96 if mode == "vFOC" else 0x97
    # big-endian: >HHHH
    data = bytes([
        (kp >> 8) & 0xFF, kp & 0xFF,
        (ki >> 8) & 0xFF, ki & 0xFF,
        (kd >> 8) & 0xFF, kd & 0xFF,
        (kv >> 8) & 0xFF, kv & 0xFF,
    ])
    return build_frame(addr, code, data)

# ============================================================
# Driver (thread-safe)
# ============================================================

@dataclass
class State:
    enabled_1: bool = False
    enabled_2: bool = False
    rpm_1: int = 0
    rpm_2: int = 0
    acc: int = 10
    pid_mode: str = "vFOC"
    kp: int = 0xDC
    ki: int = 0x64
    kd: int = 0x10E
    kv: int = 0x140

class MksRs485:
    def __init__(self, port: str, baud: int, timeout_s: float):
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=timeout_s)
        self._lock = threading.Lock()

    def close(self):
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass

    def send(self, payload: bytes):
        # Requisito del manual: bytes continuos, sin pausas dentro de una trama :contentReference[oaicite:10]{index=10}
        with self._lock:
            self._ser.write(payload)
            self._ser.flush()
        time.sleep(INTER_FRAME_DELAY_S)

# ============================================================
# Web app
# ============================================================

app = Flask(__name__)
state = State()
bus = None  # se inicializa en main()

INDEX_HTML = r"""<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>MKS SERVO RS485 - M1/M2</title>
  <style>
    body { font-family: sans-serif; margin: 18px; max-width: 980px; }
    .row { display: grid; grid-template-columns: 220px 1fr 120px; gap: 12px; align-items: center; margin: 10px 0; }
    input[type="range"] { width: 100%; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 14px; margin: 14px 0; }
    .btn { padding: 8px 12px; cursor: pointer; }
    .btn-wide { width: 100%; }
    .small { color: #555; font-size: 12px; }
    .inline { display: flex; gap: 10px; flex-wrap: wrap; align-items: center; }
    select, input[type="number"] { padding: 6px; }
    .ok { color: #0a0; }
    .bad { color: #a00; }
    code { background: #f5f5f5; padding: 2px 4px; border-radius: 4px; }
  </style>
</head>
<body>
  <h2>MKS SERVO RS485 – Control M1/M2</h2>
  <div class="small">
    <div>• ENABLE/DISABLE: <code>F3</code> • SPEED: <code>F6</code> • PID vFOC: <code>96</code> • PID CLOSE: <code>97</code></div>
    <div>• PID: el servidor manda primero <code>speed=0</code> a ambos motores y luego escribe PID.</div>
  </div>

  <div class="card">
    <h3>Enable</h3>
    <div class="inline">
      <button class="btn" onclick="setEnable(1, true)">Enable M1</button>
      <button class="btn" onclick="setEnable(1, false)">Disable M1</button>
      <button class="btn" onclick="setEnable(2, true)">Enable M2</button>
      <button class="btn" onclick="setEnable(2, false)">Disable M2</button>
      <button class="btn" onclick="setEnableBoth(true)">Enable ambos</button>
      <button class="btn" onclick="setEnableBoth(false)">Disable ambos</button>
    </div>
    <div id="enableStatus" class="small"></div>
  </div>

  <div class="card">
    <h3>Velocidad (en caliente)</h3>

    <div class="row">
      <div>Acc (0..255)</div>
      <input id="acc" type="range" min="0" max="255" value="10" oninput="onAcc()"/>
      <div><span id="accv">10</span></div>
    </div>

    <div class="row">
      <div>M1 RPM (-3000..3000)</div>
      <input id="rpm1" type="range" min="-3000" max="3000" value="0" oninput="onSpeed()"/>
      <div><span id="rpm1v">0</span></div>
    </div>

    <div class="row">
      <div>M2 RPM (-3000..3000)</div>
      <input id="rpm2" type="range" min="-3000" max="3000" value="0" oninput="onSpeed()"/>
      <div><span id="rpm2v">0</span></div>
    </div>

    <button class="btn btn-wide" onclick="sendSpeedNow()">Enviar velocidad ahora</button>
    <div id="speedStatus" class="small"></div>
  </div>

  <div class="card">
    <h3>PID (en caliente, exige motor parado)</h3>

    <div class="row">
      <div>Modo PID</div>
      <select id="pidmode" onchange="onPid()">
        <option value="vFOC">vFOC (0x96)</option>
        <option value="CLOSE">CLOSE (0x97)</option>
      </select>
      <div></div>
    </div>

    <div class="row">
      <div>Kp (0..1024)</div>
      <input id="kp" type="range" min="0" max="1024" value="220" oninput="onPid()"/>
      <div><span id="kpv">220</span></div>
    </div>

    <div class="row">
      <div>Ki (0..1024)</div>
      <input id="ki" type="range" min="0" max="1024" value="100" oninput="onPid()"/>
      <div><span id="kiv">100</span></div>
    </div>

    <div class="row">
      <div>Kd (0..1024)</div>
      <input id="kd" type="range" min="0" max="1024" value="270" oninput="onPid()"/>
      <div><span id="kdv">270</span></div>
    </div>

    <div class="row">
      <div>Kv (0..1024)</div>
      <input id="kv" type="range" min="0" max="1024" value="320" oninput="onPid()"/>
      <div><span id="kvv">320</span></div>
    </div>

    <button class="btn btn-wide" onclick="sendPidNow()">Enviar PID ahora (manda speed=0 antes)</button>
    <div id="pidStatus" class="small"></div>
  </div>

<script>
let speedTimer = null;
let pidTimer = null;

function qs(id){ return document.getElementById(id); }

async function api(path, body){
  const r = await fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body)
  });
  const t = await r.text();
  if(!r.ok) throw new Error(t || ("HTTP " + r.status));
  return t;
}

function onAcc(){
  qs("accv").textContent = qs("acc").value;
  onSpeed();
}

function onSpeed(){
  qs("rpm1v").textContent = qs("rpm1").value;
  qs("rpm2v").textContent = qs("rpm2").value;
  // hot update: debounce 120ms
  if(speedTimer) clearTimeout(speedTimer);
  speedTimer = setTimeout(sendSpeedNow, 120);
}

async function sendSpeedNow(){
  const body = {
    rpm1: parseInt(qs("rpm1").value),
    rpm2: parseInt(qs("rpm2").value),
    acc:  parseInt(qs("acc").value)
  };
  try{
    const t = await api("/api/speed", body);
    qs("speedStatus").innerHTML = '<span class="ok">' + t + '</span>';
  }catch(e){
    qs("speedStatus").innerHTML = '<span class="bad">' + e.message + '</span>';
  }
}

function onPid(){
  qs("kpv").textContent = qs("kp").value;
  qs("kiv").textContent = qs("ki").value;
  qs("kdv").textContent = qs("kd").value;
  qs("kvv").textContent = qs("kv").value;

  if(pidTimer) clearTimeout(pidTimer);
  pidTimer = setTimeout(sendPidNow, 200);
}

async function sendPidNow(){
  const body = {
    mode: qs("pidmode").value,
    kp: parseInt(qs("kp").value),
    ki: parseInt(qs("ki").value),
    kd: parseInt(qs("kd").value),
    kv: parseInt(qs("kv").value)
  };
  try{
    const t = await api("/api/pid", body);
    qs("pidStatus").innerHTML = '<span class="ok">' + t + '</span>';
  }catch(e){
    qs("pidStatus").innerHTML = '<span class="bad">' + e.message + '</span>';
  }
}

async function setEnable(motor, en){
  try{
    const t = await api("/api/enable", {motor, enable: en});
    qs("enableStatus").innerHTML = '<span class="ok">' + t + '</span>';
  }catch(e){
    qs("enableStatus").innerHTML = '<span class="bad">' + e.message + '</span>';
  }
}

async function setEnableBoth(en){
  try{
    const t = await api("/api/enable_both", {enable: en});
    qs("enableStatus").innerHTML = '<span class="ok">' + t + '</span>';
  }catch(e){
    qs("enableStatus").innerHTML = '<span class="bad">' + e.message + '</span>';
  }
}

window.addEventListener("load", () => {
  onAcc();
  onPid();
});
</script>
</body>
</html>
"""

def json_response(obj, status=200):
    return Response(json.dumps(obj, ensure_ascii=False), status=status, mimetype="application/json")

@app.get("/")
def index():
    return Response(INDEX_HTML, mimetype="text/html")

@app.post("/api/enable")
def api_enable():
    data = request.get_json(force=True)
    motor = int(data.get("motor"))
    enable = bool(data.get("enable"))

    if motor not in (1, 2):
        return Response("motor debe ser 1 o 2", status=400)

    addr = MOTOR1_ADDR if motor == 1 else MOTOR2_ADDR
    bus.send(frame_enable(addr, enable))

    if motor == 1:
        state.enabled_1 = enable
    else:
        state.enabled_2 = enable

    return Response(f"OK: F3 {'ENABLE' if enable else 'DISABLE'} enviado a motor {motor} (addr=0x{addr:02X})", mimetype="text/plain")

@app.post("/api/enable_both")
def api_enable_both():
    data = request.get_json(force=True)
    enable = bool(data.get("enable"))

    bus.send(frame_enable(MOTOR1_ADDR, enable))
    bus.send(frame_enable(MOTOR2_ADDR, enable))

    state.enabled_1 = enable
    state.enabled_2 = enable

    return Response(f"OK: F3 {'ENABLE' if enable else 'DISABLE'} enviado a M1(addr=0x{MOTOR1_ADDR:02X}) y M2(addr=0x{MOTOR2_ADDR:02X})", mimetype="text/plain")

@app.post("/api/speed")
def api_speed():
    data = request.get_json(force=True)
    rpm1 = int(data.get("rpm1", 0))
    rpm2 = int(data.get("rpm2", 0))
    acc  = int(data.get("acc", 0))

    # Enviar F6 sólo a M1 y M2
    bus.send(frame_speed(MOTOR1_ADDR, rpm1, acc))
    bus.send(frame_speed(MOTOR2_ADDR, rpm2, acc))

    state.rpm_1 = rpm1
    state.rpm_2 = rpm2
    state.acc = acc

    return Response(f"OK: F6 enviado (M1={rpm1} rpm, M2={rpm2} rpm, acc={acc})", mimetype="text/plain")

@app.post("/api/pid")
def api_pid():
    data = request.get_json(force=True)
    mode = str(data.get("mode", "vFOC"))
    kp = int(data.get("kp", 0))
    ki = int(data.get("ki", 0))
    kd = int(data.get("kd", 0))
    kv = int(data.get("kv", 0))

    # Requisito: para mandar PID, motores parados -> mandar speed=0 justo antes
    # Usamos acc=0 para parada inmediata (speed=0) en speed mode :contentReference[oaicite:11]{index=11}
    bus.send(frame_speed(MOTOR1_ADDR, 0, 0))
    bus.send(frame_speed(MOTOR2_ADDR, 0, 0))
    state.rpm_1 = 0
    state.rpm_2 = 0

    # Ahora PID a ambos motores (sólo M1 y M2)
    bus.send(frame_pid(MOTOR1_ADDR, mode, kp, ki, kd, kv))
    bus.send(frame_pid(MOTOR2_ADDR, mode, kp, ki, kd, kv))

    state.pid_mode = mode
    state.kp, state.ki, state.kd, state.kv = kp, ki, kd, kv

    return Response(f"OK: PID {mode} enviado a M1/M2 (y antes speed=0)", mimetype="text/plain")

# ============================================================
# Shutdown handling: DISABLE on exit
# ============================================================

_shutdown_once = False
def shutdown_disable():
    global _shutdown_once
    if _shutdown_once:
        return
    _shutdown_once = True

    try:
        if bus is not None:
            # DISABLE (F3 enable=00) a ambos motores :contentReference[oaicite:12]{index=12}
            bus.send(frame_enable(MOTOR1_ADDR, False))
            bus.send(frame_enable(MOTOR2_ADDR, False))
    except Exception:
        pass

def _signal_handler(signum, frame):
    shutdown_disable()
    raise SystemExit(0)

def main():
    global bus
    bus = MksRs485(RS485_PORT, RS485_BAUD, RS485_TIMEOUT_S)

    atexit.register(shutdown_disable)
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Flask (simple, single process)
    app.run(host=HTTP_HOST, port=HTTP_PORT, debug=False, threaded=True)

if __name__ == "__main__":
    main()
