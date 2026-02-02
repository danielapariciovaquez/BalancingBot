#!/usr/bin/env python3
import time
import threading
import signal
from dataclasses import dataclass

import serial
from mpu6050 import mpu6050

# ============================================================
# CONFIGURACIÓN (TOCA AQUÍ)
# ============================================================

# --- RS485 ---
RS485_PORT = "/dev/ttyUSB0"
RS485_BAUD = 38400
RS485_TIMEOUT_S = 0.05
INTER_FRAME_DELAY_S = 0.01  # pausa entre tramas (no entre bytes)

MOTOR1_ADDR = 0x01
MOTOR2_ADDR = 0x02

# Si un motor gira al revés, cambia su SIGN a -1
# (esto invierte el signo de la consigna "rpm_cmd" hacia ese motor)
MOTOR1_SIGN = +1
MOTOR2_SIGN = +1

# --- Control ---
CONTROL_HZ = 100.0           # frecuencia del lazo
CALIB_SECONDS = 1.0          # tiempo de calibración Z0 al arranque (robot derecho)
F6_ACC = 20                  # 0..255 (aceleración interna del driver en comando F6)
MAX_RPM = 250                # limitador duro de velocidad
MAX_STEP_RPM_PER_CYCLE = 50  # slew limiter por ciclo (además del F6_ACC)

# Ganancias iniciales (ajustar en pruebas)
KP = 60.0
KD = 6.0

# Derivada filtrada (IIR 1er orden)
D_ALPHA = 0.20  # 0..1 (más alto = menos filtrado)

# --- IMU ---
IMU_I2C_ADDR = 0x68
# Eje usado: Z. Tú: inclinar hacia adelante => Z más negativo.
# Control usa e = Z - Z0 (si inclinas hacia delante => e < 0)

# ============================================================
# PROTOCOLO MKS (RS485)
#   Downlink: FA addr code ... CRC
#   Uplink:   FB addr code ... CRC
#   CRC: CHECKSUM8 = sum(bytes) & 0xFF
#   Datos: big-endian
#   Comandos usados:
#     0x82: Set working mode (05 = SR_vFOC)
#     0xF3: Enable (01 lock / 00 disable)
#     0xF6: Speed control (dir bit en MSB de Byte4, speed en Byte4[3:0]+Byte5, acc=Byte6)
# ============================================================

def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF

def build_frame(addr: int, code: int, data: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum8(base)])

def encode_f6_speed(rpm_signed: int, acc: int, dir_invert: bool = False) -> bytes:
    """
    F6 payload: Byte4 Byte5 acc
      dir_bit = 1 si rpm_signed < 0 (y speed = abs)
      Byte4 = (dir_bit<<7) | ((speed>>8) & 0x0F)
      Byte5 = speed & 0xFF
      acc   = 0..255
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

    if dir_invert:
        dir_bit ^= 1

    b4 = ((dir_bit & 0x01) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return bytes([b4, b5, acc])

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

@dataclass
class MotorSigns:
    m1: int = +1
    m2: int = +1

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

    def tx(self, addr: int, code: int, data: bytes = b"", expect_reply: bool = False) -> bytes | None:
        frame = build_frame(addr, code, data)
        with self.lock:
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(self.ifd)

            if not expect_reply:
                return None

            # Lectura simple de respuesta (si la necesitas en futuro). Aquí no la usamos en el lazo.
            buf = self.ser.read(64)
            return buf if buf else None

    # --- Alto nivel ---
    def set_work_mode_srvfoc(self, addr: int):
        # 0x82, data=0x05
        self.tx(addr, 0x82, bytes([0x05]), expect_reply=True)

    def enable(self, addr: int, en: bool):
        self.tx(addr, 0xF3, bytes([0x01 if en else 0x00]), expect_reply=True)

    def speed(self, addr: int, rpm_signed: int, acc: int, dir_invert: bool = False):
        self.tx(addr, 0xF6, encode_f6_speed(rpm_signed, acc, dir_invert=dir_invert), expect_reply=False)

    def stop(self, addr: int, acc: int = 2):
        # Stop en speed mode: speed=0
        self.speed(addr, 0, acc=acc, dir_invert=False)

def main():
    # ---------- Señales para parar limpio ----------
    stop_flag = {"stop": False}

    def _sig_handler(_sig, _frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    # ---------- IMU ----------
    imu = mpu6050(IMU_I2C_ADDR)

    # ---------- RS485 ----------
    bus = MksRs485(RS485_PORT, RS485_BAUD, RS485_TIMEOUT_S, INTER_FRAME_DELAY_S)

    signs = MotorSigns(m1=MOTOR1_SIGN, m2=MOTOR2_SIGN)

    def safe_shutdown():
        try:
            bus.stop(MOTOR1_ADDR, acc=max(1, min(255, F6_ACC)))
            bus.stop(MOTOR2_ADDR, acc=max(1, min(255, F6_ACC)))
            time.sleep(0.05)
            bus.enable(MOTOR1_ADDR, False)
            bus.enable(MOTOR2_ADDR, False)
        except Exception:
            pass
        finally:
            bus.close()

    # ---------- Inicialización motores ----------
    try:
        bus.set_work_mode_srvfoc(MOTOR1_ADDR)
        bus.set_work_mode_srvfoc(MOTOR2_ADDR)
        bus.enable(MOTOR1_ADDR, True)
        bus.enable(MOTOR2_ADDR, True)
    except Exception as e:
        safe_shutdown()
        raise RuntimeError(f"Fallo inicializando RS485/motores: {e}") from e

    # ---------- Calibración Z0 ----------
    z_sum = 0.0
    n = 0
    t_end = time.monotonic() + max(0.2, float(CALIB_SECONDS))
    while time.monotonic() < t_end:
        a = imu.get_accel_data()
        z_sum += float(a.get("z", 0.0))
        n += 1
        time.sleep(0.005)

    if n == 0:
        safe_shutdown()
        raise RuntimeError("Calibración IMU fallida: 0 muestras")

    z0 = z_sum / n
    print(f"[IMU] Z0 referencia = {z0:.6f} (m/s^2), N={n}")

    # ---------- Lazo de control ----------
    dt = 1.0 / float(CONTROL_HZ)
    next_t = time.monotonic()

    e_prev = 0.0
    d_f = 0.0
    u_prev = 0.0

    print("[CTRL] Ejecutando. Ctrl+C para salir (se mandará 0 rpm).")
    try:
        while not stop_flag["stop"]:
            now = time.monotonic()
            if now < next_t:
                time.sleep(next_t - now)
                continue
            next_t += dt

            # Lectura eje Z
            a = imu.get_accel_data()
            z = float(a.get("z", 0.0))

            # Error respecto a referencia (robot derecho al inicio)
            e = z - z0  # hacia delante => e < 0 (según tu caso)

            # Derivada + filtrado
            d = (e - e_prev) / dt
            d_f = d_f + D_ALPHA * (d - d_f)
            e_prev = e

            # PD -> rpm
            u = KP * e + KD * d_f

            # Saturación dura
            u = clamp(u, -MAX_RPM, MAX_RPM)

            # Slew limiter por ciclo
            du = u - u_prev
            du = clamp(du, -MAX_STEP_RPM_PER_CYCLE, MAX_STEP_RPM_PER_CYCLE)
            u = u_prev + du
            u_prev = u

            rpm_cmd = int(round(u))

            # Mismo comando para ambas ruedas (si un motor debe ir invertido, usa MOTORx_SIGN=-1)
            bus.speed(MOTOR1_ADDR, signs.m1 * rpm_cmd, acc=F6_ACC, dir_invert=False)
            bus.speed(MOTOR2_ADDR, signs.m2 * rpm_cmd, acc=F6_ACC, dir_invert=False)

    except Exception as e:
        print(f"[CTRL] Excepción: {e}")
    finally:
        print("[CTRL] Parando motores...")
        safe_shutdown()
        print("[CTRL] Listo.")

if __name__ == "__main__":
    main()
