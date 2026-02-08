#!/usr/bin/env python3
import time
import threading
from dataclasses import dataclass

# ---------------- I2C / MPU6050 ----------------
I2C_BUS = 1
MPU_ADDR = 0x68  # GY-521 típico (AD0=0). Si AD0=1 -> 0x69

REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B  # AX_H ... AZ_L (6 bytes) y TEMP (2) y GYRO (6)
REG_GYRO_XOUT_H  = 0x43

# Por defecto, si no configuras rangos:
# ACCEL ±2g  => 16384 LSB/g
# GYRO  ±250 => 131 LSB/(deg/s)
ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

READ_HZ = 200  # frecuencia del hilo de lectura (web puede refrescar más lento)

# ---------------- Web ----------------
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


@dataclass
class ImuSample:
    # raw
    ax: int = 0
    ay: int = 0
    az: int = 0
    gx: int = 0
    gy: int = 0
    gz: int = 0
    temp_raw: int = 0

    # scaled
    ax_g: float = 0.0
    ay_g: float = 0.0
    az_g: float = 0.0
    gx_dps: float = 0.0
    gy_dps: float = 0.0
    gz_dps: float = 0.0
    temp_c: float = 0.0

    t_monotonic: float = 0.0
    seq: int = 0
    hz_est: float = 0.0


class ImuReader:
    def __init__(self):
        self.lock = threading.Lock()
        self.sample = ImuSample()
        self._stop = False

        self._last_t = None
        self._hz_ema = 0.0
        self._ema_alpha = 0.05  # suavizado de Hz

        self.bus = SMBus(I2C_BUS)
        self._wake()

    def _wake(self):
        # Wake up device (clear sleep bit)
        self.bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def stop(self):
        self._stop = True

    def read_once(self) -> ImuSample:
        # Leer 14 bytes desde ACCEL_XOUT_H:
        # AX,AY,AZ (6) + TEMP (2) + GX,GY,GZ (6)
        data = self.bus.read_i2c_block_data(MPU_ADDR, REG_ACCEL_XOUT_H, 14)

        ax = i16(data[0], data[1])
        ay = i16(data[2], data[3])
        az = i16(data[4], data[5])
        temp_raw = i16(data[6], data[7])
        gx = i16(data[8], data[9])
        gy = i16(data[10], data[11])
        gz = i16(data[12], data[13])

        # Escalado
        ax_g = ax / ACC_LSB_PER_G
        ay_g = ay / ACC_LSB_PER_G
        az_g = az / ACC_LSB_PER_G

        gx_dps = gx / GYRO_LSB_PER_DPS
        gy_dps = gy / GYRO_LSB_PER_DPS
        gz_dps = gz / GYRO_LSB_PER_DPS

        # Temp: datasheet MPU6050: Temp(°C) = temp_raw/340 + 36.53
        temp_c = (temp_raw / 340.0) + 36.53

        t = time.monotonic()

        # Hz estimada
        if self._last_t is None:
            hz = 0.0
        else:
            dt = t - self._last_t
            hz = (1.0 / dt) if dt > 1e-6 else 0.0
        self._last_t = t

        if self._hz_ema == 0.0:
            self._hz_ema = hz
        else:
            self._hz_ema += self._ema_alpha * (hz - self._hz_ema)

        with self.lock:
            self.sample.seq += 1
            self.sample.ax, self.sample.ay, self.sample.az = ax, ay, az
            self.sample.gx, self.sample.gy, self.sample.gz = gx, gy, gz
            self.sample.temp_raw = temp_raw

            self.sample.ax_g, self.sample.ay_g, self.sample.az_g = ax_g, ay_g, az_g
            self.sample.gx_dps, self.sample.gy_dps, self.sample.gz_dps = gx_dps, gy_dps, gz_dps
            self.sample.temp_c = temp_c

            self.sample.t_monotonic = t
            self.sample.hz_est = self._hz_ema

            return ImuSample(**self.sample.__dict__)

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

    def snapshot(self) -> ImuSample:
        with self.lock:
            return ImuSample(**self.sample.__dict__)


def main():
    require_deps()

    imu = ImuReader()
    th = threading.Thread(target=imu.loop, daemon=True)
    th.start()

    app = Flask(__name__)

    PAGE = r"""
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>IMU GY-521 / MPU6050</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 18px; max-width: 920px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .card { border: 1px solid #ddd; border-radius: 12px; padding: 12px; }
    .kv { display: grid; grid-template-columns: 140px 1fr; gap: 6px 10px; }
    .k { color: #444; }
    .v { font-variant-numeric: tabular-nums; }
    .muted { color: #666; font-size: 0.95em; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; }
    h2 { margin: 0 0 8px; }
  </style>
</head>
<body>
  <h2>MPU6050 (GY-521) — Acelerómetro + Giróscopo</h2>
  <div class="muted">Actualización: <span id="status">—</span></div>

  <div class="grid" style="margin-top:12px;">
    <div class="card">
      <b>Acelerómetro</b>
      <div class="kv" style="margin-top:10px;">
        <div class="k">AX raw</div><div class="v mono" id="ax">—</div>
        <div class="k">AY raw</div><div class="v mono" id="ay">—</div>
        <div class="k">AZ raw</div><div class="v mono" id="az">—</div>
        <div class="k">AX (g)</div><div class="v mono" id="axg">—</div>
        <div class="k">AY (g)</div><div class="v mono" id="ayg">—</div>
        <div class="k">AZ (g)</div><div class="v mono" id="azg">—</div>
      </div>
    </div>

    <div class="card">
      <b>Giróscopo</b>
      <div class="kv" style="margin-top:10px;">
        <div class="k">GX raw</div><div class="v mono" id="gx">—</div>
        <div class="k">GY raw</div><div class="v mono" id="gy">—</div>
        <div class="k">GZ raw</div><div class="v mono" id="gz">—</div>
        <div class="k">GX (°/s)</div><div class="v mono" id="gxd">—</div>
        <div class="k">GY (°/s)</div><div class="v mono" id="gyd">—</div>
        <div class="k">GZ (°/s)</div><div class="v mono" id="gzd">—</div>
      </div>
    </div>
  </div>

  <div class="card" style="margin-top:12px;">
    <b>Estado</b>
    <div class="kv" style="margin-top:10px;">
      <div class="k">Temp (°C)</div><div class="v mono" id="temp">—</div>
      <div class="k">Seq</div><div class="v mono" id="seq">—</div>
      <div class="k">Hz estimada</div><div class="v mono" id="hz">—</div>
    </div>
  </div>

<script>
function fmt(n, digits=3){
  if (typeof n !== "number") return "—";
  return n.toFixed(digits);
}
async function tick(){
  try{
    const r = await fetch("/api/imu", {cache:"no-store"});
    const j = await r.json();

    document.getElementById("ax").textContent = j.ax;
    document.getElementById("ay").textContent = j.ay;
    document.getElementById("az").textContent = j.az;
    document.getElementById("axg").textContent = fmt(j.ax_g, 4);
    document.getElementById("ayg").textContent = fmt(j.ay_g, 4);
    document.getElementById("azg").textContent = fmt(j.az_g, 4);

    document.getElementById("gx").textContent = j.gx;
    document.getElementById("gy").textContent = j.gy;
    document.getElementById("gz").textContent = j.gz;
    document.getElementById("gxd").textContent = fmt(j.gx_dps, 3);
    document.getElementById("gyd").textContent = fmt(j.gy_dps, 3);
    document.getElementById("gzd").textContent = fmt(j.gz_dps, 3);

    document.getElementById("temp").textContent = fmt(j.temp_c, 2);
    document.getElementById("seq").textContent = j.seq;
    document.getElementById("hz").textContent = fmt(j.hz_est, 1);

    document.getElementById("status").textContent = "OK";
  }catch(e){
    document.getElementById("status").textContent = "ERROR";
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

    @app.get("/api/imu")
    def api_imu():
        s = imu.snapshot()
        return jsonify({
            "ax": s.ax, "ay": s.ay, "az": s.az,
            "gx": s.gx, "gy": s.gy, "gz": s.gz,
            "temp_raw": s.temp_raw,

            "ax_g": s.ax_g, "ay_g": s.ay_g, "az_g": s.az_g,
            "gx_dps": s.gx_dps, "gy_dps": s.gy_dps, "gz_dps": s.gz_dps,
            "temp_c": s.temp_c,

            "seq": s.seq,
            "hz_est": s.hz_est,
            "t_monotonic": s.t_monotonic,
        })

    try:
        app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)
    finally:
        imu.stop()
        imu.close()


if __name__ == "__main__":
    main()
