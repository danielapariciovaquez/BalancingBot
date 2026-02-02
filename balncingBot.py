#!/usr/bin/env python3
import time
import math
import signal
import serial
from mpu6050 import mpu6050

# ===================== CONFIG =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400

ADDR1 = 0x01
ADDR2 = 0x02

# Motor 1 invertido
M1_SIGN = -1
M2_SIGN = +1

ACC = 10
MAX_RPM = 250

CALIB_SECONDS = 1.0
K_RPM_PER_DEG = 20.0
DEADZONE_DEG = 0.8
PITCH_ALPHA = 0.20

LOOP_HZ = 100.0
RPM_UPDATE_THRESHOLD = 2  # rpm

# IMPORTANTE para RS485
IF_GAP_S = 0.003          # 3 ms entre tramas (m1 y m2)
WRITE_TIMEOUT_S = 0.2

IMU_ADDR = 0x68
# ==================================================

def checksum8(b: bytes) -> int:
    return sum(b) & 0xFF

def frame(addr: int, cmd: int, payload: bytes=b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_set_mode(addr: int) -> bytes:
    return frame(addr, 0x82, bytes([0x05]))  # SR_vFOC

def cmd_enable(addr: int, en: bool) -> bytes:
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_speed(addr: int, rpm_signed: int, acc: int) -> bytes:
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

    with serial.Serial(
        PORT, BAUD,
        timeout=0,                 # NO RX
        write_timeout=WRITE_TIMEOUT_S
    ) as ser:

        def tx_expect(b: bytes, expect_len: int):
            n = ser.write(b)
            ser.flush()            # solo TX: fuerza envío
            if n != expect_len:
                # Esto es diagnóstico TX (no es RX). Si aparece, ahí está el problema.
                print(f"\n[WARN] write() devolvió {n} bytes, esperado {expect_len}")
            return n

        def tx_gap_expect(b: bytes, expect_len: int):
            n = tx_expect(b, expect_len)
            time.sleep(IF_GAP_S)
            return n

        def safe_stop():
            tx_gap_expect(cmd_speed(ADDR1, 0, ACC), 7)
            tx_gap_expect(cmd_speed(ADDR2, 0, ACC), 7)
            time.sleep(0.05)
            tx_gap_expect(cmd_enable(ADDR1, False), 5)
            tx_gap_expect(cmd_enable(ADDR2, False), 5)

        # ---- Init motores ----
        tx_gap_expect(cmd_set_mode(ADDR1), 5)
        tx_gap_expect(cmd_set_mode(ADDR2), 5)
        tx_gap_expect(cmd_enable(ADDR1, True), 5)
        tx_gap_expect(cmd_enable(ADDR2, True), 5)

        # ---- Calibración pitch0 ----
        t_end = time.monotonic() + max(0.2, CALIB_SECONDS)
        n = 0
        p0_sum = 0.0
        while time.monotonic() < t_end:
            a = imu.get_accel_data()
            p0_sum += accel_to_pitch_deg(float(a["x"]), float(a["y"]), float(a["z"]))
            n += 1
            time.sleep(0.005)
        pitch0 = p0_sum / max(1, n)

        print(f"[CAL] pitch0={pitch0:+.3f} deg (N={n})")
        print(f"[CFG] M1_SIGN={M1_SIGN:+d}  M2_SIGN={M2_SIGN:+d}")
        print(f"[CFG] K={K_RPM_PER_DEG:.2f} rpm/deg  deadzone={DEADZONE_DEG:.2f}°  MAX_RPM={MAX_RPM}")
        print(f"[CFG] Envío SOLO si cambia >= {RPM_UPDATE_THRESHOLD} rpm. IF_GAP={IF_GAP_S*1e3:.1f} ms. Ctrl+C STOP.")

        dt = 1.0 / LOOP_HZ
        next_t = time.monotonic()

        pitch_f = 0.0
        last_sent_rpm = None
        log_next = time.monotonic()
        log_dt = 0.1  # 10 Hz

        try:
            while not stop_flag["stop"]:
                now = time.monotonic()
                if now < next_t:
                    time.sleep(next_t - now)
                    continue
                next_t += dt

                a = imu.get_accel_data()
                pitch = accel_to_pitch_deg(float(a["x"]), float(a["y"]), float(a["z"])) - pitch0
                pitch_f = pitch_f + PITCH_ALPHA * (pitch - pitch_f)

                if abs(pitch_f) < DEADZONE_DEG:
                    rpm_cmd = 0.0
                else:
                    rpm_cmd = K_RPM_PER_DEG * pitch_f

                rpm_cmd = clamp(rpm_cmd, -MAX_RPM, MAX_RPM)
                rpm_int = int(round(rpm_cmd))

                if (last_sent_rpm is None) or (abs(rpm_int - last_sent_rpm) >= RPM_UPDATE_THRESHOLD):
                    m1 = M1_SIGN * rpm_int
                    m2 = M2_SIGN * rpm_int
                    tx_gap_expect(cmd_speed(ADDR1, m1, ACC), 7)
                    tx_gap_expect(cmd_speed(ADDR2, m2, ACC), 7)
                    last_sent_rpm = rpm_int

                if now >= log_next:
                    log_next += log_dt
                    print(f"\rpitch={pitch_f:+7.2f}°  rpm={rpm_int:+5d}  sent={last_sent_rpm:+5d}    ",
                          end="", flush=True)

        finally:
            print("\n[STOP] Parando motores...")
            safe_stop()
            print("[STOP] Hecho.")

if __name__ == "__main__":
    main()

