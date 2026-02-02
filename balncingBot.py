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

# Motor 1 invertido (según tu prueba)
M1_SIGN = -1
M2_SIGN = +1

ACC = 10            # 0..255
MAX_RPM = 250       # límite seguridad

# Proporcional a pitch (grados)
CALIB_SECONDS = 1.0
K_RPM_PER_DEG = 20.0
DEADZONE_DEG = 0.8
PITCH_ALPHA = 0.20  # filtro suave 0..1

# Frecuencia de cálculo (solo afecta a cuándo se recalcula; NO implica reenvío)
LOOP_HZ = 100.0

# Solo enviamos F6 si cambia la consigna al menos esto
RPM_UPDATE_THRESHOLD = 2  # rpm

# IMU
IMU_ADDR = 0x68
# ==================================================

def checksum8(b: bytes) -> int:
    return sum(b) & 0xFF

def frame(addr: int, cmd: int, payload: bytes=b"") -> bytes:
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

    with serial.Serial(PORT, BAUD, timeout=0, write_timeout=0.2) as ser:
        def tx(b: bytes):
            # NO leemos RX, NO hacemos keep-alive, solo escribimos
            ser.write(b)

        def safe_stop():
            # STOP + disable
            tx(cmd_speed(ADDR1, 0, ACC))
            tx(cmd_speed(ADDR2, 0, ACC))
            time.sleep(0.05)
            tx(cmd_enable(ADDR1, False))
            tx(cmd_enable(ADDR2, False))

        # ---- Init motores ----
        tx(cmd_set_mode(ADDR1)); time.sleep(0.05)
        tx(cmd_set_mode(ADDR2)); time.sleep(0.05)
        tx(cmd_enable(ADDR1, True)); time.sleep(0.05)
        tx(cmd_enable(ADDR2, True)); time.sleep(0.05)

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
        print(f"[CFG] M1_SIGN={M1_SIGN:+d}  M2_SIGN={M2_SIGN:+d}  K={K_RPM_PER_DEG:.2f} rpm/deg  deadzone={DEADZONE_DEG:.2f}°  MAX_RPM={MAX_RPM}")
        print(f"[CFG] Envío F6 SOLO si cambia la consigna >= {RPM_UPDATE_THRESHOLD} rpm. Ctrl+C para salir (STOP).")

        dt = 1.0 / LOOP_HZ
        next_t = time.monotonic()

        pitch_f = 0.0
        last_sent_rpm = None  # rpm_cmd antes de aplicar signos
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

                # Solo enviar si cambia lo suficiente
                if (last_sent_rpm is None) or (abs(rpm_int - last_sent_rpm) >= RPM_UPDATE_THRESHOLD):
                    m1 = M1_SIGN * rpm_int
                    m2 = M2_SIGN * rpm_int
                    tx(cmd_speed(ADDR1, m1, ACC))
                    tx(cmd_speed(ADDR2, m2, ACC))
                    last_sent_rpm = rpm_int

                if now >= log_next:
                    log_next += log_dt
                    # Ojo: el log muestra la consigna calculada, no garantiza que se haya enviado en ese instante
                    print(f"\rpitch={pitch_f:+7.2f}°  rpm={rpm_int:+5d}  sent={last_sent_rpm:+5d}    ", end="", flush=True)

        finally:
            print("\n[STOP] Parando motores...")
            safe_stop()
            print("[STOP] Hecho.")

if __name__ == "__main__":
    main()
