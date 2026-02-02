#!/usr/bin/env python3
import time
import math
import signal
import threading
from dataclasses import dataclass

import serial
from mpu6050 import mpu6050

# ============================================================
# CONFIG (TOCAR AQUÍ)
# ============================================================

# ---- RS485 ----
RS485_PORT = "/dev/ttyUSB0"
RS485_BAUD = 38400
RS485_TIMEOUT_S = 0.05
INTER_FRAME_DELAY_S = 0.002  # pequeño; evita meter latencia excesiva

MOTOR1_ADDR = 0x01
MOTOR2_ADDR = 0x02

# Signos por motor (ajuste de sentido)
# En un chasis típico, muchas veces un motor necesita signo opuesto al otro.
# Empieza así, y si ves que van mal, cámbialos:
M1_SIGN = +1
M2_SIGN = -1

# ---- Modo de trabajo y enable ----
# 82H: mode=05 => SR_vFOC (bus FOC)  :contentReference[oaicite:4]{index=4}
WORK_MODE = 0x05

# F3H: enable=01 => shaft lock (habilitado) :contentReference[oaicite:5]{index=5}
ENABLE_ON_START = True

# ---- Speed command (F6) ----
F6_ACC = 20        # 0..255  :contentReference[oaicite:6]{index=6}
MAX_RPM = 250      # límite duro de seguridad (en pruebas)

# ---- Control (proporcional a inclinación) ----
LOOP_HZ = 100.0
CALIB_SECONDS = 1.0

# Ganancia: RPM por grado de inclinación
# Empieza bajo (p.ej. 10..30) para pruebas con robot sujeto/ruedas al aire.
K_RPM_PER_DEG = 20.0

# Deadzone (grados): por debajo no mandamos movimiento
DEADZONE_DEG = 0.8

# Suavizado de pitch para que no vibre
PITCH_LP_ALPHA = 0.15  # 0..1

# Slew limiter (cambio máximo de rpm por ciclo)
MAX_STEP_RPM_PER_CYCLE = 40

# ---- IMU ----
IMU_I2C_ADDR = 0x68

# ============================================================
# PROTOCOLO MKS RS485 (manual)
# Downlink: FA addr code ... CRC, CRC = sum(bytes) & 0xFF  :contentReference[oaicite:7]{index=7}
# F6: Byte4 MSB=dir, (Byte4 low 4 bits + Byte5)=speed, acc byte6 :contentReference[oaicite:8]{index=8}
# ============================================================

def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF

def build_frame(addr: int, code: int, data: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum8(base)])

def encode_f6_speed(rpm_signed: int, acc: int) -> bytes:
    """
    F6: FA addr F6 Byte4 Byte5 acc CRC
    Byte4: bit7=dir, bits[3:0] = speed[11:8]
    Byte5: speed[7:0]
    speed: 0..3000
    acc: 0..255
    """
    rpm = int(rpm_signed)
    dir_bit = 1 if rpm < 0 else 0
    speed = abs(rpm)

    if speed > 3000:
        speed = 3000
    if acc < 0:
        acc = 0
    if acc > 255:
        acc = 255

    b4 = ((dir_bit & 0x01) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return bytes([b4, b5, acc])

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def lp(prev: float, x: float, alpha: float) -> float:
    return prev + alpha * (x - prev)

@dataclass
class MotorSigns:
    m1: int = +1
    m2: int = -1

class MksRs485:
    def __init__(self, port: str, baud: int, timeout_s: float, inter_frame_delay_s: float):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_s,
        )
        self.ifd = float(inter_frame_delay_s)
        self.lock = threading.Lock()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def tx(self, addr: int, code: int, data: bytes = b"", expect_reply: bool = True) -> bytes | None:
        frame = build_frame(addr, code, data)
        with self.lock:
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(self.ifd)
            if not expect_reply:
                return None
            # Respuesta típica: FB addr code status CRC (5 bytes) en muchos comandos
            resp = self.ser.read(64)
            return resp if resp else None

    def set_work_mode(self, addr: int, mode: int):
        # 82H mode :contentReference[oaicite:9]{index=9}
        self.tx(addr, 0x82, bytes([mode & 0xFF]), expect_reply=True)

    def enable(self, addr: int, en: bool):
        # F3H enable: 00 loose / 01 shaft lock :contentReference[oaicite:10]{index=10}
        self.tx(addr, 0xF3, bytes([0x01 if en else 0x00]), expect_reply=True)

    def speed_f6(self, addr: int, rpm_signed: int, acc: int):
        # F6H speed control :contentReference[oaicite:11]{index=11}
        self.tx(addr, 0xF6, encode_f6_speed(rpm_signed, acc), expect_reply=False)

    def stop_f6(self, addr: int, acc: int):
        # Stop en speed mode: speed=0, acc!=0 frena suave, acc=0 instantáneo :contentReference[oaicite:12]{index=12}
        self.speed_f6(addr, 0, acc)

def accel_to_pitch_deg(ax: float, ay: float, az: float) -> float:
    # Pitch desde acelerómetro (inclinación respecto a gravedad)
    # pitch = atan2(-ax, sqrt(ay^2 + az^2))
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return pitch * 180.0 / math.pi

def main():
    stop_flag = {"stop": False}

    def _sig_handler(_sig, _frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    imu = mpu6050(IMU_I2C_ADDR)
    bus = MksRs485(RS485_PORT, RS485_BAUD, RS485_TIMEOUT_S, INTER_FRAME_DELAY_S)
    signs = MotorSigns(m1=M1_SIGN, m2=M2_SIGN)

    def safe_shutdown():
        try:
            bus.stop_f6(MOTOR1_ADDR, acc=max(1, min(255, F6_ACC)))
            bus.stop_f6(MOTOR2_ADDR, acc=max(1, min(255, F6_ACC)))
            time.sleep(0.05)
            bus.enable(MOTOR1_ADDR, False)
            bus.enable(MOTOR2_ADDR, False)
        except Exception:
            pass
        bus.close()

    # ---- Init motores: modo + enable ----
    try:
        bus.set_work_mode(MOTOR1_ADDR, WORK_MODE)
        bus.set_work_mode(MOTOR2_ADDR, WORK_MODE)
        if ENABLE_ON_START:
            bus.enable(MOTOR1_ADDR, True)
            bus.enable(MOTOR2_ADDR, True)
    except Exception as e:
        safe_shutdown()
        raise RuntimeError(f"Fallo inicializando motores por RS485: {e}") from e

    # ---- Calibración: pitch0 con robot recto ----
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
        safe_shutdown()
        raise RuntimeError("Calibración fallida: 0 muestras")
    pitch0 = p0_sum / n

    print(f"[CAL] pitch0 = {pitch0:+.3f} deg (N={n})")
    print("[RUN] Inclinación -> velocidad (proporcional). Ctrl+C para parar (0 rpm).")
    print(f"[CFG] K={K_RPM_PER_DEG:.2f} rpm/deg, deadzone={DEADZONE_DEG:.2f} deg, MAX_RPM={MAX_RPM}, signs=({signs.m1},{signs.m2})")

    # ---- Loop ----
    dt = 1.0 / LOOP_HZ
    next_t = time.monotonic()
    pitch_f = 0.0
    rpm_prev = 0.0

    # Para log por consola a ~20 Hz
    log_dt = 1.0 / 20.0
    log_next = time.monotonic()

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
            pitch_f = lp(pitch_f, pitch, PITCH_LP_ALPHA)

            # deadzone
            if abs(pitch_f) < DEADZONE_DEG:
                rpm_cmd = 0.0
            else:
                rpm_cmd = K_RPM_PER_DEG * pitch_f

            rpm_cmd = clamp(rpm_cmd, -MAX_RPM, MAX_RPM)

            # slew limiter por ciclo
            dr = rpm_cmd - rpm_prev
            dr = clamp(dr, -MAX_STEP_RPM_PER_CYCLE, MAX_STEP_RPM_PER_CYCLE)
            rpm_cmd = rpm_prev + dr
            rpm_prev = rpm_cmd

            rpm_int = int(round(rpm_cmd))

            # Enviar a ambos motores (con signos por motor)
            bus.speed_f6(MOTOR1_ADDR, signs.m1 * rpm_int, acc=F6_ACC)
            bus.speed_f6(MOTOR2_ADDR, signs.m2 * rpm_int, acc=F6_ACC)

            # Log
            if now >= log_next:
                log_next += log_dt
                print(
                    f"\rpitch={pitch_f:+7.2f} deg  -> rpm={rpm_int:+5d}  |  "
                    f"m1={signs.m1*rpm_int:+5d}  m2={signs.m2*rpm_int:+5d}     ",
                    end="",
                    flush=True,
                )

    finally:
        print("\n[STOP] Parando motores...")
        safe_shutdown()
        print("[STOP] Hecho.")

if __name__ == "__main__":
    main()
