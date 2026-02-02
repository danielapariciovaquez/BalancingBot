#!/usr/bin/env python3
import time
import math
from mpu6050 import mpu6050

# =========================
# CONFIG
# =========================
IMU_I2C_ADDR = 0x68
CALIB_SECONDS = 1.0      # tiempo para promediar referencia inicial
PRINT_HZ = 20.0          # refresco consola
ALPHA = 0.15             # filtro IIR para señales (0..1)

# =========================
# Helpers
# =========================
def lp(prev: float, x: float, alpha: float) -> float:
    return prev + alpha * (x - prev)

def deg(rad: float) -> float:
    return rad * 180.0 / math.pi

def main():
    sensor = mpu6050(IMU_I2C_ADDR)

    # -------- Calibración (robot recto) --------
    t_end = time.monotonic() + max(0.2, CALIB_SECONDS)
    n = 0
    ax0 = ay0 = az0 = 0.0

    while time.monotonic() < t_end:
        a = sensor.get_accel_data()
        ax0 += float(a.get("x", 0.0))
        ay0 += float(a.get("y", 0.0))
        az0 += float(a.get("z", 0.0))
        n += 1
        time.sleep(0.005)

    if n == 0:
        raise RuntimeError("No se pudieron tomar muestras de calibración.")

    ax0 /= n
    ay0 /= n
    az0 /= n

    print(f"[CAL] N={n}  ax0={ax0:+.4f}  ay0={ay0:+.4f}  az0={az0:+.4f}  (m/s^2)")
    print("[INFO] Mueve el robot e inspecciona signos. Ctrl+C para salir.\n")

    # -------- Bucle de monitorización --------
    dt = 1.0 / PRINT_HZ
    next_t = time.monotonic()

    # filtros
    ax_f = ay_f = az_f = 0.0
    pitch_f = roll_f = 0.0
    ez_f = 0.0  # e = az - az0 filtrado

    while True:
        now = time.monotonic()
        if now < next_t:
            time.sleep(next_t - now)
            continue
        next_t += dt

        a = sensor.get_accel_data()
        ax = float(a.get("x", 0.0))
        ay = float(a.get("y", 0.0))
        az = float(a.get("z", 0.0))

        # Filtrado de aceleraciones
        ax_f = lp(ax_f, ax, ALPHA)
        ay_f = lp(ay_f, ay, ALPHA)
        az_f = lp(az_f, az, ALPHA)

        # Error respecto a referencia en Z (lo que estabas usando)
        ez = az - az0
        ez_f = lp(ez_f, ez, ALPHA)

        # Ángulos físicos desde acelerómetro (gravedad)
        # Pitch (inclinación “hacia delante/atrás” según convención)
        pitch = math.atan2(-ax_f, math.sqrt(ay_f * ay_f + az_f * az_f))
        # Roll (inclinación lateral)
        roll  = math.atan2(ay_f, az_f)

        pitch_f = lp(pitch_f, pitch, ALPHA)
        roll_f  = lp(roll_f, roll, ALPHA)

        # Imprimir en una sola línea (sobrescribe)
        line = (
            f"Ax={ax_f:+7.3f}  Ay={ay_f:+7.3f}  Az={az_f:+7.3f}  |  "
            f"eZ=Az-Az0={ez_f:+7.3f}  |  "
            f"pitch={deg(pitch_f):+7.2f}°  roll={deg(roll_f):+7.2f}°"
        )
        print("\r" + line + "   ", end="", flush=True)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[EXIT]")

