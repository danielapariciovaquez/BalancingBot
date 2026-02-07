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

INTER_FRAME_DELAY_S = 0.004

ENABLE_RETRIES = 2
ENABLE_RETRY_DELAY_S = 0.02

WEB_HOST = "0.0.0.0"
WEB_PORT = 8000
# ==================================================

app = Flask(__name__)

# Serial lock (evita colisiones TX/RX si llegan requests simultáneas)
ser_lock = threading.Lock()
ser: serial.Serial | None = None


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

def parse_ack(buf: bytes):
    # Esperado: FB addr code status crc  (5 bytes)
    if len(buf) != 5:
        return None
    if buf[0] != 0xFB:
        return None
    crc = checksum8(buf[:4])
    if crc != buf[4]:
        return None
    return {
        "header": buf[0],
        "addr": buf[1],
        "code": buf[2],
        "status": buf[3],  # 1 ok, 0 fail
        "crc_ok": True
    }

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

def txrx_ack(s: serial.Serial, data: bytes, expect_code: int, expect_addr: int) -> dict:
    """
    Envía frame y espera ACK 5 bytes: FB addr code status crc.
    Devuelve dict con info y validaciones.
    """
    # Limpia input para no mezclar ACK antiguos
    s.reset_input_buffer()
    tx(s, data)

    buf = s.read(5)
    ack = parse_ack(buf)
    if ack is None:
        return {
            "ok": False,
            "error": "no_ack_or_bad_crc_or_bad_header",
            "raw": buf.hex(" ")
        }

    if ack["addr"] != (expect_addr & 0xFF) or ack["code"] != (expect_code & 0xFF):
        return {
            "ok": False,
            "error": "ack_mismatch",
            "raw": buf.hex(" "),
            "ack": ack
        }

    return {
        "ok": bool(ack["status"] == 1),
        "status": ack["status"],
        "raw": buf.hex(" ")
    }

def cmd_enable(addr: int, en: bool) -> bytes:
    # FA addr F3 enable CRC
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_pid(addr: int, mode: str, kp: int, ki: int, kd: int, kv: int) -> bytes:
    """
    Manual 5.3:
      vFOC PID: code 0x96
      CLOSE PID: code 0x97
      payload: Kp(2) Ki(2) Kd(2) Kv(2), big-endian
      ranges: 0..1024
    """
    if mode == "vFOC":
        code = 0x96
    elif mode == "CLOSE":
        code = 0x97
    else:
        raise ValueError("mode must be 'vFOC' or 'CLOSE'")

    # clamp a rango del manual 0..1024
    kp = max(0, min(1024, int(kp)))
    ki = max(0, min(1024, int(ki)))
    kd = max(0, min(1024, int(kd)))
    kv = max(0, min(1024, int(kv)))

    payload = u16be(kp) + u16be(ki) + u16be(kd) + u16be(kv)
    return frame(addr, code, payload)

# ===================== WEB UI =====================
HTML = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>MKS SERVO PID (2 motores)</title>
  <style>
    body { font-family: sans-serif; margin: 18px; max-width: 860px; }
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
  <h2>PID MKS SERVO (ADDR 0x01 + 0x02)</h2>
  <div class="box">
    <div class="btns">
      <button onclick="doEnable(true)">Enable (ambos)</button>
      <button onclick="doEnable(false)">Disable (ambos)</button>
    </div>

    <div class="row">
      <div class="label"><b>Modo PID</b></div>
      <select id="mode">
        <option value="vFOC">vFOC (0x96)</option>
        <option value="CLOSE">CLOSE (0x97)</option>
      </select>
    </div>

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
      <button onclick="applyPID()">Apply PID (ambos)</button>
      <button onclick="loadDefaults()">Defaults típicos (manual)</button>
    </div>

    <div class="small">
      Envía <code>FA addr 96/97 Kp Ki Kd Kv CRC</code> y espera ACK
      <code>FB addr 96/97 status CRC</code>.
    </div>

    <h3>Resultado</h3>
    <pre id="out">(sin acciones)</pre>
  </div>

<script>
function bind(id, vid) {
  const s = document.getElementById(id);
  const v = document.getElementById(vid);
  const upd = ()=> v.textContent = parseInt(s.value,10);
  s.addEventListener('input', upd);
  upd();
  return s;
}
const kp = bind('kp','kpv');
const ki = bind('ki','kiv');
const kd = bind('kd','kdv');
const kv = bind('kv','kvv');

function out(x) {
  document.getElementById('out').textContent = (typeof x === 'string') ? x : JSON.stringify(x, null, 2);
}

async function doEnable(en) {
  const r = await fetch('/enable', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({enable: !!en})
  });
  out(await r.json());
}

async function applyPID() {
  const mode = document.getElementById('mode').value;
  const payload = {
    mode,
    kp: parseInt(kp.value,10),
    ki: parseInt(ki.value,10),
    kd: parseInt(kd.value,10),
    kv: parseInt(kv.value,10),
  };
  const r = await fetch('/pid', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(payload)
  });
  out(await r.json());
}

function loadDefaults() {
  // Defaults del manual:
  // vFOC:  Kp=0x00DC(220) Ki=0x0064(100) Kd=0x010E(270) Kv=0x0140(320)
  // CLOSE: Kp=0x00C8(200) Ki=0x0050(80)  Kd=0x00FA(250) Kv=0x012C(300)
  const mode = document.getElementById('mode').value;
  if (mode === "vFOC") { kp.value=220; ki.value=100; kd.value=270; kv.value=320; }
  else { kp.value=200; ki.value=80; kd.value=250; kv.value=300; }
  kp.dispatchEvent(new Event('input'));
  ki.dispatchEvent(new Event('input'));
  kd.dispatchEvent(new Event('input'));
  kv.dispatchEvent(new Event('input'));
  out("Defaults cargados en sliders (no enviados aún).");
}
</script>
</body>
</html>
"""

@app.get("/")
def index():
    return HTML

@app.post("/enable")
def api_enable():
    data = request.get_json(force=True, silent=True) or {}
    en = bool(data.get("enable", False))

    with ser_lock:
        s = ensure_serial()
        # enable/disable no siempre responde; lo hacemos robusto (reintento + pacing)
        for _ in range(ENABLE_RETRIES):
            tx(s, cmd_enable(ADDR1, en))
            time.sleep(ENABLE_RETRY_DELAY_S)
        for _ in range(ENABLE_RETRIES):
            tx(s, cmd_enable(ADDR2, en))
            time.sleep(ENABLE_RETRY_DELAY_S)

    return jsonify({"ok": True, "enable": en})

@app.post("/pid")
def api_pid():
    data = request.get_json(force=True, silent=True) or {}
    mode = str(data.get("mode", "vFOC"))

    try:
        kp = int(data.get("kp", 0))
        ki = int(data.get("ki", 0))
        kd = int(data.get("kd", 0))
        kv = int(data.get("kv", 0))
    except Exception:
        return jsonify({"ok": False, "error": "kp/ki/kd/kv inválidos"}), 400

    if mode not in ("vFOC", "CLOSE"):
        return jsonify({"ok": False, "error": "mode debe ser 'vFOC' o 'CLOSE'"}), 400

    # Frame code según modo
    code = 0x96 if mode == "vFOC" else 0x97

    # Enviar a ambos + leer ACK
    res = {
        "ok": True,
        "mode": mode,
        "set": {
            "kp": max(0, min(1024, kp)),
            "ki": max(0, min(1024, ki)),
            "kd": max(0, min(1024, kd)),
            "kv": max(0, min(1024, kv)),
        },
        "motor_0x01": None,
        "motor_0x02": None,
    }

    with ser_lock:
        s = ensure_serial()

        pkt1 = cmd_pid(ADDR1, mode, kp, ki, kd, kv)
        pkt2 = cmd_pid(ADDR2, mode, kp, ki, kd, kv)

        r1 = txrx_ack(s, pkt1, expect_code=code, expect_addr=ADDR1)
        # pequeña separación extra para no pisar el ACK del segundo
        time.sleep(0.006)
        r2 = txrx_ack(s, pkt2, expect_code=code, expect_addr=ADDR2)

    res["motor_0x01"] = r1
    res["motor_0x02"] = r2
    res["ok"] = bool(r1.get("ok", False) and r2.get("ok", False))

    return jsonify(res)

def main():
    # abre puerto al arranque para fallar rápido si no existe
    with ser_lock:
        ensure_serial()
    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)

if __name__ == "__main__":
    main()
