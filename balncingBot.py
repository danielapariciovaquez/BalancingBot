#!/usr/bin/env python3
import time
import math
import signal
import serial
from mpu6050 import mpu6050

# ===================== CONFIG =====================
# RS485
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT = 0.05

ADDR1 = 0x01
ADDR2 = 0x02

# Motor 1 invertido (según tu prueba)
M1_SIGN = -1
M2_SIGN = +1

# F6 (speed)
ACC = 20              # 0..255
MAX_RPM = 250         # límite por seguridad

# Control proporcional
LOOP_HZ = 50.0        # <<< BAJAMOS a 50 Hz para estabilidad RS485
CALIB_SECONDS = 1.0
K_RPM_PER_DEG = 20.0  # rpm por grado
DEADZONE_DEG = 0.8

# Suavizado de pitch
PITCH_ALPHA = 0.15    # 0..1

# Inter-frame gap (muy importante en RS485)
INTER_FRAME_DELAY_S = 0.002  # 2 ms entre tramas

# Re-Enable periódico (opcional pero útil)
REENABLE_EVERY_S = 1.0

# IMU
IMU_ADDR = 0x68
# ==================================================

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def frame(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_set_mode(addr: int) -> bytes:
    # 82H set work mode, 05H = SR_vFOC
    return frame(addr, 0x82, bytes([0x05]))

def cmd_enable(addr: int, en: bool) -> bytes:
    # F3H enable: 01 = enable, 00 = disable
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_speed(addr: int, rpm_signed: int, acc: int) -> bytes:
    # F6H speed command
    rpm = int(rpm_signed)
    dir_bit = 1 if rpm < 0 else 0
    speed = abs(rpm)
    if speed > 3000:
        speed = 3000
    acc = max(0, min(255, int(acc)))
    b4 = ((dir_bit & 1) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return frame(addr, 0xF6, bytes([b4, b5, acc]))

def accel_to_pitch_deg(ax: float, ay: float, az: float) -> float:
    # pitch = atan2(-Ax, sqrt(Ay^2 + Az^2))
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return pitch * 180.0 / math.pi

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def main():
    stop_flag = {"stop": False}

    def on_sig(_sig, _frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    imu = mpu6050(IMU_ADDR)

    # Importante: write_timeout evita bloqueos si el adaptador se atasca
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT, write_timeout=0.2) as ser:

        def tx(b: bytes):
            ser.write(b)

        def tx_gap(b: bytes):
            ser.write(b)
            time.sleep(INTER_FRAME_DELAY_S)

        def safe_stop():
            # STOP + disable (con gaps)
            tx_gap(cmd_speed(ADDR1, 0, ACC))
            tx_gap(cmd_speed(ADDR2, 0, ACC))
            tx_gap(cmd_enable(ADDR1, False))
            tx_gap(cmd_enable(ADDR2, False))

        # ---------- Init motores ----------
        tx_gap(cmd_set_mode(ADDR1))
        tx_gap(cmd_set_mode(ADDR2))
        tx_gap(cmd_enable(ADDR1, True))
        tx_gap(cmd_enable(ADDR2, True))

        # ---------- Calibración pitch0 ----------
        t_end = time.monotonic() + max(0.2, CALIB_SECONDS)
        n = 0
        p0_sum = 0.0
        while time.monotonic() < t_end:
            a = imu.get_accel_data()
            ax = float(a.get("x", 0.0))
            ay = float(a.get("y", 0.0))
            az = float(a.get("z", 0.0))
            p0_sum += accel_to_pitch_deg(ax, ay, az)
            n += 1
            time.sleep(0.005)

        if n == 0:
            safe_stop()
            raise RuntimeError("Calibración fallida (0 muestras).")

        pitch0 = p0_sum / n
        print(f"[CAL] pitch0 = {pitch0:+.3f} deg  (N={n})")
        print(f"[CFG] LOOP_HZ={LOOP_HZ:.1f}  K={K_RPM_PER_DEG:.2f} rpm/deg  deadzone={DEADZONE_DEG:.2f}°  MAX_RPM={MAX_RPM}")
        print(f"[CFG] M1_SIGN={M1_SIGN:+d} (invertido)  M2_SIGN={M2_SIGN:+d}")
        print("[RUN] Inclinación -> velocidad. Ctrl+C para salir (STOP).")

        dt = 1.0 / LOOP_HZ
        next_t = time.monotonic()

        pitch_f = 0.0
        log_next = time.monotonic()
        log_dt = 1.0 / 20.0  # log a 20 Hz

        last_enable = time.monotonic()

        try:
            while not stop_flag["stop"]:
                now = time.monotonic()
                if now < next_t:
                    time.sleep(next_t - now)
                    continue
                next_t += dt

                a = imu.get_accel_data()
                ax = float(a.get("x", 0.0))
                ay = float(a.get("y", 0.0))
                az = float(a.get("z", 0.0))

                pitch = accel_to_pitch_deg(ax, ay, az) - pitch0
                pitch_f = pitch_f + PITCH_ALPHA * (pitch - pitch_f)

                if abs(pitch_f) < DEADZONE_DEG:
                    rpm_cmd = 0.0
                else:
                    rpm_cmd = K_RPM_PER_DEG * pitch_f

                rpm_cmd = clamp(rpm_cmd, -MAX_RPM, MAX_RPM)
                rpm_int = int(round(rpm_cmd))

                m1 = M1_SIGN * rpm_int
                m2 = M2_SIGN * rpm_int

                # Envío con gap entre tramas (clave)
                tx_gap(cmd_speed(ADDR1, m1, ACC))
                tx_gap(cmd_speed(ADDR2, m2, ACC))

                # Re-enable periódico (por si el driver entra/sale de algún estado)
                if (now - last_enable) >= REENABLE_EVERY_S:
                    last_enable = now
                    tx_gap(cmd_enable(ADDR1, True))
                    tx_gap(cmd_enable(ADDR2, True))

                if now >= log_next:
                    log_next += log_dt
                    print(f"\rpitch={pitch_f:+7.2f}°  rpm={rpm_int:+5d} | m1={m1:+5d}  m2={m2:+5d}    ",
                          end="", flush=True)

        finally:
            print("\n[STOP] Parando motores...")
            safe_stop()
            print("[STOP] Hecho.")

if __name__ == "__main__":
    main()
