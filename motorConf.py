#!/usr/bin/env python3
import time
import threading
import serial
from flask import Flask, request, jsonify

# ===================== CONFIG =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.08

ADDR1 = 0x01
ADDR2 = 0x02

MAX_RPM = 300          # rango del manual para F6
ACC_DEFAULT = 255       # aceleración por defecto
INTER_FRAME_DELAY_S = 0.004

ENABLE_RETRIES = 2
ENABLE_RETRY_DELAY_S = 0.02

# Envío periódico de velocidad (hot update)
SPEED_HZ = 25           # frecuencia de envío de velocidad
SPEED_DEADBAND_RPM = 0  # si quieres evitar spam, pon p.ej. 1..5 rpm

WEB_HOST = "0.0.0.0"
WEB_PORT = 8000
# ==================================================

app = Flask(__name__)

ser_lock = threading.Lock()
ser: serial.Serial | None = None

param_lock = threading.Lock()
state = {
    "enabled": False,
    "mode": "CLOSE",   # 'vFOC' o 'CLOSE'
    "kp": 200,
    "ki": 80,
    "kd": 250,
    "kv": 300,
    "speed_rpm": 0,    # barra de velocidad común a los dos motores (signed)
    "acc": ACC_DEFAULT
}

# ===================== RS485 / FRAMES =====================
def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def u16be(x: int) -> bytes:
    x = int(x)
    if x < 0:
        x = 0
    if x > 0xFFFF:
        x = 0xFFFF
    return bytes([(x >> 8) & 0xFF, x & 0xFF])

def frame(addr: int, code: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def open_serial() -> serial.Serial:
    s = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT_S,
        write_timeout=TIMEOUT_S,
    )
    s.reset_input_buffer()
    s.reset_output_buffer()
    return s

def ensure_serial() -> serial.Serial:
    global ser
    if ser is None or not ser.is_open:
        ser = open_serial()
    return ser

def tx(s: serial.Serial, data: bytes) -> None:
    s.write(data)
    s.flush()
    if INTER_FRAME_DELAY_S > 0:
        time.sleep(INTER_FRAME_DELAY_S)

def send_enable_robust(s: serial.Serial, addr: int, en: bool) -> None:
    pkt = cmd_enable(addr, en)
    for i in range(ENABLE_RETRIES):
        tx(s, pkt)
        if i != ENABLE_RETRIES - 1:
            time.sleep(ENABLE_RETRY_DELAY_S)

def cmd_enable(addr: int, en: bool) -> bytes:
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_pid(addr: int, mode: str, kp: int, ki: int, kd: int, kv: int) -> bytes:
    """
    Manual:
      vFOC PID:  0x96
      CLOSE PID: 0x97
      payload: Kp(2) Ki(2) Kd(2) Kv(2) big-endian
      range típico: 0..1024
    """
    if mode == "vFOC":
        code = 0x96
    elif mode == "CLOSE":
        code = 0x97
    else:
        raise ValueError("mode must be 'vFOC' or 'CLOSE'")

    kp = max(0, min(1024, int(kp)))
    ki = max(0, min(1024, int(ki)))
    kd = max(0, min(1024, int(kd)))
    kv = max(0, min(1024, int(kv)))

    payload = u16be(kp) + u16be(ki) + u16be(kd) + u16be(kv)
    return frame(addr, code, payload)

def cmd_speed(addr: int, rpm: int, acc: int) -> bytes:
    """
    F6 speed:
      Byte4: bit7 dir, bits[3:0]=speed[11:8]
      Byte5: speed[7:0]
      Byte6: acc (0..255)
    """
    acc_u8 = max(0, min(255, int(acc)))
    if rpm >= 0:
        direction_bit = 0
        speed = rpm
    else:
        direction_bit = 1
        speed = -rpm

    speed = max(0, min(MAX_RPM, int(speed)))
    b4 = ((direction_bit & 0x01) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    payload = bytes([b4, b5, acc_u8])
    return frame(addr, 0xF6, payload)

# ===================== periodic speed sender =====================
def speed_loop():
    last_sent = None
    period = 1.0 / float(SPEED_HZ)
    t_next = time.monotonic()

    while True:
        with param_lock:
            en = bool(state["enabled"])
            rpm = int(state["speed_rpm"])
            acc = int(state["acc"])

        if en:
            # evita enviar si no cambia (opcional)
            if last_sent is None or abs(rpm - last_sent) > SPEED_DEADBAND_RPM:
                with ser_lock:
                    s = ensure_serial()
                    tx(s, cmd_speed(ADDR1, rpm, acc))
                    tx(s, cmd_speed(ADDR2, rpm, acc))
                last_sent = rpm
            else:
                # aunque no cambie, también puedes reenviar "keep-alive":
                with ser_lock:
                    s = ensure_serial()
                    tx(s, cmd_speed(ADDR1, rpm, acc))
                    tx(s, cmd_speed(ADDR2, rpm, acc))
        else:
            last_sent = None

        t_next += period
        sleep_s = t_next - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            t_next = time.monotonic()

# ===================== Web UI =====================
HTML = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>MKS SERVO PID + Speed (2 motores)</title>
  <style>
    body { font-family: sans-serif; margin: 18px; max-width: 900px; }
    .box { padding: 12px; border: 1px solid #ddd; border-radius: 12px; }
    .row { margin: 14px 0; }
    .label { display:flex; justify-content:space-between; margin-bottom: 6px; }
    input[type=range] { width: 100%; }
    button { padding: 10px 14px; border-radius: 10px; border: 1px solid #ccc; cursor: pointer; }
    .btns { display:flex; gap:10px; flex-wrap: wrap; margin: 10px 0; }
    code { background:#f6f6f6; padding:2px 6px; border-radius:6px; }
    pre { background:#0b0f14; color:#dfe7ef; padding:10px; border-radius:12px; overflow:auto; }
    select { padding: 8px; border-radius: 10px; border: 1px solid #ccc; }
    .small { color:#555; font-size: 0.95em; line-height:1.35; }
  </style>
</head>
<body>
  <h2>PID + Velocidad (hot) — ADDR 0x01 + 0x02</h2>
  <div class="box">
    <div class="btns">
      <button onclick="setEnable(true)">Enable (ambos)</button>
      <button onclick="setEnable(false)">Disable (ambos)</button>
      <button onclick="stopNow()">STOP (0 rpm)</button>
    </div>

    <div class="row">
      <div class="label"><b>Modo PID</b></div>
      <select id="mode">
        <option value="vFOC">vFOC (0x96)</option>
        <option value="CLOSE">CLOSE (0x97)</option>
      </select>
    </div>

    <h3>PID (se aplica a ambos)</h3>
    <div class="row">
      <div class="label"><b>Kp</b><span id="kpv"></span></div>
      <input id="kp" type="range" min="0" max="1024" step="1">
    </div>
    <div class="row">
      <div class="label"><b>Ki</b><span id="kiv"></span></div>
      <input id="ki" type="range" min="0" max="1024" step="1">
    </div>
    <div class="row">
      <div class="label"><b>Kd</b><span id="kdv"></span></div>
      <input id="kd" type="range" min="0" max="1024" step="1">
    </div>
    <div class="row">
      <div class="label"><b>Kv</b><span id="kvv"></span></div>
      <input id="kv" type="range" min="0" max="1024" step="1">
    </div>
    <div class="btns">
      <button onclick="defaults()">Defaults típicos (manual)</button>
    </div>

    <h3>Velocidad (hot, envío periódico)</h3>
    <div class="row">
      <div class="label"><b>Speed RPM</b><span id="spv"></span></div>
      <input id="speed" type="range" min="-3000" max="3000" step="1">
    </div>
    <div class="row">
      <div class="label"><b>ACC</b><span id="accv"></span></div>
      <input id="acc" type="range" min="0" max="255" step="1">
    </div>
    <div class="small">
      La velocidad se envía automáticamente a {SPEED_HZ} Hz (F6) a ambos motores.
      El PID se envía automáticamente al mover sliders (0x96/0x97), con antirebote.
    </div>

    <h3>Estado</h3>
    <pre id="out">(sin acciones)</pre>
  </div>

<script>
function out(x){ document.getElementById('out').textContent =
  (typeof x==='string') ? x : JSON.stringify(x,null,2); }

function bind(id, vid, fmt){
  const s=document.getElementById(id);
  const v=document.getElementById(vid);
  const upd=()=>v.textContent=fmt(parseFloat(s.value));
  s.addEventListener('input', upd);
  upd();
  return s;
}

const kp=bind('kp','kpv',v=>Math.round(v));
const ki=bind('ki','kiv',v=>Math.round(v));
const kd=bind('kd','kdv',v=>Math.round(v));
const kv=bind('kv','kvv',v=>Math.round(v));
const speed=bind('speed','spv',v=>Math.round(v));
const acc=bind('acc','accv',v=>Math.round(v));

async function getState(){
  const r=await fetch('/state');
  return await r.json();
}
async function setEnable(en){
  const r=await fetch('/enable',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({enable:!!en})});
  out(await r.json());
}
async function stopNow(){
  speed.value=0;
  speed.dispatchEvent(new Event('input'));
  const r=await fetch('/state',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({speed_rpm:0})});
  out(await r.json());
}

function defaults(){
  const mode=document.getElementById('mode').value;
  if(mode==="vFOC"){ kp.value=220; ki.value=100; kd.value=270; kv.value=320; }
  else { kp.value=200; ki.value=80; kd.value=250; kv.value=300; }
  kp.dispatchEvent(new Event('input'));
  ki.dispatchEvent(new Event('input'));
  kd.dispatchEvent(new Event('input'));
  kv.dispatchEvent(new Event('input'));
  out("Defaults cargados (se enviarán automáticamente al cabo de ~150ms).");
}

// antirebote para envíos automáticos de PID
let tPid=null;
async function pushPid(){
  clearTimeout(tPid);
  tPid=setTimeout(async ()=>{
    const mode=document.getElementById('mode').value;
    const payload={
      mode,
      kp:parseInt(kp.value,10),
      ki:parseInt(ki.value,10),
      kd:parseInt(kd.value,10),
      kv:parseInt(kv.value,10),
    };
    const r=await fetch('/pid',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify(payload)});
    out(await r.json());
  },150);
}

kp.addEventListener('input', pushPid);
ki.addEventListener('input', pushPid);
kd.addEventListener('input', pushPid);
kv.addEventListener('input', pushPid);
document.getElementById('mode').addEventListener('change', pushPid);

// velocidad en caliente: sólo actualiza estado; el envío lo hace el hilo periódico
let tSp=null;
async function pushSpeed(){
  clearTimeout(tSp);
  tSp=setTimeout(async ()=>{
    const payload={
      speed_rpm: parseInt(speed.value,10),
      acc: parseInt(acc.value,10)
    };
    const r=await fetch('/state',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify(payload)});
    out(await r.json());
  },60);
}
speed.addEventListener('input', pushSpeed);
acc.addEventListener('input', pushSpeed);

// init sliders desde backend
(async ()=>{
  const s=await getState();
  document.getElementById('mode').value=s.mode;
  kp.value=s.kp; ki.value=s.ki; kd.value=s.kd; kv.value=s.kv;
  speed.value=s.speed_rpm; acc.value=s.acc;
  kp.dispatchEvent(new Event('input'));
  ki.dispatchEvent(new Event('input'));
  kd.dispatchEvent(new Event('input'));
  kv.dispatchEvent(new Event('input'));
  speed.dispatchEvent(new Event('input'));
  acc.dispatchEvent(new Event('input'));
  out(s);
})();
</script>
</body>
</html>
"""

@app.get("/")
def index():
    return HTML

@app.get("/state")
def api_state_get():
    with param_lock:
        return jsonify(dict(state))

@app.post("/state")
def api_state_post():
    d = request.get_json(force=True, silent=True) or {}
    with param_lock:
        if "speed_rpm" in d:
            v = int(d["speed_rpm"])
            v = max(-MAX_RPM, min(MAX_RPM, v))
            state["speed_rpm"] = v
        if "acc" in d:
            a = int(d["acc"])
            a = max(0, min(255, a))
            state["acc"] = a
    return jsonify({"ok": True, **state})

@app.post("/enable")
def api_enable():
    d = request.get_json(force=True, silent=True) or {}
    en = bool(d.get("enable", False))

    with param_lock:
        state["enabled"] = en
        # si deshabilitas, fuerza speed=0 para no dejar estado raro
        if not en:
            state["speed_rpm"] = 0

    # manda enable/disable robusto
    with ser_lock:
        s = ensure_serial()
        send_enable_robust(s, ADDR1, en)
        send_enable_robust(s, ADDR2, en)

    return jsonify({"ok": True, "enabled": en})

@app.post("/pid")
def api_pid():
    d = request.get_json(force=True, silent=True) or {}
    mode = str(d.get("mode", "CLOSE"))
    if mode not in ("vFOC", "CLOSE"):
        return jsonify({"ok": False, "error": "mode debe ser 'vFOC' o 'CLOSE'"}), 400

    try:
        kp = int(d.get("kp", 0))
        ki = int(d.get("ki", 0))
        kd = int(d.get("kd", 0))
        kv = int(d.get("kv", 0))
    except Exception:
        return jsonify({"ok": False, "error": "kp/ki/kd/kv inválidos"}), 400

    kp = max(0, min(1024, kp))
    ki = max(0, min(1024, ki))
    kd = max(0, min(1024, kd))
    kv = max(0, min(1024, kv))

    with param_lock:
        state["mode"] = mode
        state["kp"] = kp
        state["ki"] = ki
        state["kd"] = kd
        state["kv"] = kv

    # envío inmediato del PID a ambos motores (sin botón)
    # (el usuario pidió periodicidad para velocidad; para PID conviene envío inmediato por evento)
    with ser_lock:
        s = ensure_serial()
        tx(s, cmd_pid(ADDR1, mode, kp, ki, kd, kv))
        tx(s, cmd_pid(ADDR2, mode, kp, ki, kd, kv))

    return jsonify({"ok": True, "mode": mode, "kp": kp, "ki": ki, "kd": kd, "kv": kv})

def main():
    # abre puerto al arranque
    with ser_lock:
        ensure_serial()

    # arranca hilo de envío periódico de velocidad
    t = threading.Thread(target=speed_loop, daemon=True)
    t.start()

    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)

if __name__ == "__main__":
    main()
