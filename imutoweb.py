#!/usr/bin/env python3
import time
import threading

# -------- MPU6050 / GY-521 --------
I2C_BUS = 1
MPU_ADDR = 0x68

REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B  # AX..AZ (6 bytes)

ACC_LSB_PER_G = 16384.0  # ±2g por defecto si no configuras rangos

READ_HZ = 200

# -------- Web --------
WEB_HOST = "0.0.0.0"
WEB_PORT = 8000

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

try:
    from flask import Flask, jsonify
except ImportError:
    Flask = None


def require_deps():
    if SMBus is None:
        raise RuntimeError("Falta smbus2. Instala: pip3 install smbus2")
    if Flask is None:
        raise RuntimeError("Falta flask. Instala: pip3 install flask")


def i16(msb: int, lsb: int) -> int:
    v = (msb << 8) | lsb
    return v - 0x10000 if v & 0x8000 else v


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class AccZToDeg:
    """
    Lee AZ (acelerómetro) y lo convierte a "grados" suponiendo que:
      - el eje Z está alineado con la gravedad cuando está vertical
      - |AZ_g| <= 1 g (clamp) para evitar NaN por ruido
    Conversión:
      az_g = az_raw / 16384
      angle_deg = acos(clamp(az_g, -1..1)) * 180/pi
    Nota importante:
      Este ángulo es el "inclinómetro" respecto a la vertical (0° = Z alineado con g),
      pero NO da el signo (no distingue hacia qué lado cae) con solo AZ.
      Para ángulo con signo necesitas al menos otro eje (AX o AY).
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.az_raw = 0
        self.az_g = 0.0
        self.angle_deg = 0.0
        self.seq = 0
        self.t = 0.0
        self._stop = False

        self.bus = SMBus(I2C_BUS)
        self.bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

    def stop(self):
        self._stop = True

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def read_once(self):
        data = self.bus.read_i2c_block_data(MPU_ADDR, REG_ACCEL_XOUT_H, 6)
        az = i16(data[4], data[5])

        az_g = az / ACC_LSB_PER_G
        az_g_clamped = clamp(az_g, -1.0, 1.0)

        # angle from vertical
        # 0° -> az_g=+1, 90° -> az_g=0, 180° -> az_g=-1
        import math
        angle = math.degrees(math.acos(az_g_clamped))

        t = time.monotonic()
        with self.lock:
            self.az_raw = az
            self.az_g = az_g
            self.angle_deg = angle
            self.seq += 1
            self.t = t

    def loop(self):
        period = 1.0 / float(READ_HZ)
        t_next = time.monotonic()
        while not self._stop:
            self.read_once()
            t_next += period
            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                t_next = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {
                "az_raw": self.az_raw,
                "az_g": self.az_g,
                "angle_deg": self.angle_deg,
                "seq": self.seq,
                "t_monotonic": self.t,
            }


def main():
    require_deps()

    conv = AccZToDeg()
    th = threading.Thread(target=conv.loop, daemon=True)
    th.start()

    app = Flask(__name__)

    PAGE = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>AZ → grados</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 18px; max-width: 720px; }
    .card { border: 1px solid #ddd; border-radius: 12px; padding: 14px; }
    .big { font-size: 44px; font-variant-numeric: tabular-nums; }
    .kv { display: grid; grid-template-columns: 120px 1fr; gap: 6px 10px; margin-top: 10px; }
    .k { color:#444; }
    .v { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; }
    .muted { color:#666; font-size: 0.95em; margin-bottom: 10px; }
  </style>
</head>
<body>
  <h2>MPU6050 — Acelerómetro Z → grados</h2>
  <div class="muted">0° = eje Z alineado con la gravedad (|AZ|≈1g). 90° = Z perpendicular a g.</div>

  <div class="card">
    <div class="big"><span id="ang">—</span>°</div>
    <div class="kv">
      <div class="k">AZ raw</div><div class="v" id="azraw">—</div>
      <div class="k">AZ (g)</div><div class="v" id="azg">—</div>
    </div>
  </div>

<script>
function fmt(n, d){ return (typeof n === "number") ? n.toFixed(d) : "—"; }

async function tick(){
  try{
    const r = await fetch("/api", {cache:"no-store"});
    const j = await r.json();
    document.getElementById("ang").textContent = fmt(j.angle_deg, 2);
    document.getElementById("azraw").textContent = j.az_raw;
    document.getElementById("azg").textContent = fmt(j.az_g, 4);
  }catch(e){
    document.getElementById("ang").textContent = "ERR";
  }
}
setInterval(tick, 100);
tick();
</script>
</body>
</html>
"""

    @app.get("/")
    def index():
        return PAGE

    @app.get("/api")
    def api():
        return jsonify(conv.snapshot())

    try:
        app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)
    finally:
        conv.stop()
        conv.close()


if __name__ == "__main__":
    main()
