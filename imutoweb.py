#!/usr/bin/env python3
import time
import math
import sys
import serial
from collections import deque

# ===================== RS485 / MOTORES =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05

ADDR_M1 = 0x01
ADDR_M2 = 0x02

# Inversión sencilla por motor (si están montados en sentidos opuestos)
INVERT_M1 = False
INVERT_M2 = True

INTER_FRAME_DELAY_S = 0.004
ENABLE_RETRIES = 2
ENABLE_RETRY_DELAY_S = 0.02

# F4 requiere speed+acc en el payload (aunque tú solo quieras "posición")
POS_SPEED_RPM = 300
POS_ACC = 10
# ===========================================================

# ===================== I2C / MPU6050 (GY-521) =====================
I2C_BUS = 1
MPU_ADDR = 0x68

REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B  # AX..AZ

ACC_LSB_PER_G = 16384.0  # ±2g default si no configuras rango
# ===============================================================

# ===================== Conversión grados -> axis =====================
COUNTS_PER_TURN = 16384  # 16384/turn (manual)
COUNTS_PER_DEG = COUNTS_PER_TURN / 360.0
# ===============================================================

# ===================== Loop + filtros =====================
UPDATE_HZ = 50
ANGLE_DEADBAND_DEG = 0.2   # umbral para enviar (evita spam por ruido)

# Media móvil sobre el ángulo en grados (recomendado 5..15)
MA_WINDOW = 100
# ===============================================================

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None


def require_i2c():
    if SMBus is None:
        raise RuntimeError("Falta smbus2. Instala: pip3 install smbus2")


def clamp_int(x: int, lo: int, hi: int) -> int:
    return lo if x < lo else hi if x > hi else x


def i16(msb: int, lsb: int) -> int:
    v = (msb << 8) | lsb
    return v - 0x10000 if v & 0x8000 else v


def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF


def frame(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])


def cmd_enable(addr: int, en: bool) -> bytes:
    # FA addr F3 01/00 CRC
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))


def cmd_pos_rel_axis_f4(addr: int, speed_rpm: int, acc: int, rel_axis: int) -> bytes:
    """
    Position mode 3: relative motion by axis (F4)
    Downlink: FA addr F4 speed(2B) acc(1B) relAxis(int32) CRC
    """
    speed_rpm = clamp_int(int(speed_rpm), 0, 3000)
    acc = clamp_int(int(acc), 0, 255)
    rel_axis = int(rel_axis)

    speed_hi = (speed_rpm >> 8) & 0xFF
    speed_lo = speed_rpm & 0xFF

    rel_u32 = rel_axis & 0xFFFFFFFF
    b7  = (rel_u32 >> 24) & 0xFF
    b8  = (rel_u32 >> 16) & 0xFF
    b9  = (rel_u32 >> 8) & 0xFF
    b10 = rel_u32 & 0xFF

    payload = bytes([speed_hi, speed_lo, acc, b7, b8, b9, b10])
    return frame(addr, 0xF4, payload)


def open_serial() -> serial.Serial:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT_S,
        write_timeout=TIMEOUT_S,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def send(ser: serial.Serial, data: bytes) -> None:
    ser.write(data)
    ser.flush()


def send_paced(ser: serial.Serial, data: bytes, delay_s: float = INTER_FRAME_DELAY_S) -> None:
    send(ser, data)
    if delay_s > 0:
        time.sleep(delay_s)


def send_enable_robust(ser: serial.Serial, addr: int, en: bool) -> None:
    pkt = cmd_enable(addr, en)
    for i in range(ENABLE_RETRIES):
        send_paced(ser, pkt, INTER_FRAME_DELAY_S)
        if i != ENABLE_RETRIES - 1:
            time.sleep(ENABLE_RETRY_DELAY_S)


def mpu_wake(bus: SMBus) -> None:
    bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.05)


def read_az_raw(bus: SMBus) -> int:
    # Leer AX..AZ (6 bytes) y tomar AZ (bytes 4..5)
    data = bus.read_i2c_block_data(MPU_ADDR, REG_ACCEL_XOUT_H, 6)
    return i16(data[4], data[5])


def az_to_angle_deg(az_raw: int) -> float:
    """
    Conversión sólo con Z:
      az_g = az_raw/LSB
      angle = acos(clamp(az_g,-1..1)) -> 0..180 deg
    (Sin signo: no distingue hacia qué lado cae con solo AZ)
    """
    az_g = az_raw / ACC_LSB_PER_G
    if az_g < -1.0:
        az_g = -1.0
    elif az_g > 1.0:
        az_g = 1.0
    return math.degrees(math.acos(az_g))


class MovingAverage:
    def __init__(self, n: int):
        n = int(n)
        if n < 1:
            n = 1
        self.buf = deque(maxlen=n)
        self.sum = 0.0

    def push(self, x: float) -> float:
        if len(self.buf) == self.buf.maxlen:
            self.sum -= self.buf[0]
        self.buf.append(x)
        self.sum += x
        return self.sum / len(self.buf)


def apply_inversion(rel_counts: int, invert: bool) -> int:
    return -rel_counts if invert else rel_counts


def main() -> int:
    require_i2c()

    bus = SMBus(I2C_BUS)
    mpu_wake(bus)

    ser = open_serial()

    # ===== SOLO: enable motores 1 y 2 =====
    send_enable_robust(ser, ADDR_M1, True)
    send_enable_robust(ser, ADDR_M2, True)

    ma = MovingAverage(MA_WINDOW)

    # Inicializa referencia (con media móvil ya “cebada”)
    az0 = read_az_raw(bus)
    ang0 = az_to_angle_deg(az0)
    ang0_f = ma.push(ang0)
    last_angle_f = ang0_f

    print(f"ENABLE OK. AngleZ start = {last_angle_f:.2f} deg | MA_WINDOW={MA_WINDOW}")
    print("Enviando SOLO: F3 (enable) + F4 (pos relativa por axis) a motor 1 y 2. Ctrl+C para salir.")

    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()

    try:
        while True:
            az = read_az_raw(bus)
            ang = az_to_angle_deg(az)
            ang_f = ma.push(ang)

            ddeg = ang_f - last_angle_f
            if abs(ddeg) >= ANGLE_DEADBAND_DEG:
                rel_counts = int(round(ddeg * COUNTS_PER_DEG))

                rel_m1 = apply_inversion(rel_counts, INVERT_M1)
                rel_m2 = apply_inversion(rel_counts, INVERT_M2)

                # ===== SOLO: manda F4 a ambos motores =====
                send_paced(ser, cmd_pos_rel_axis_f4(ADDR_M1, POS_SPEED_RPM, POS_ACC, rel_m1))
                send_paced(ser, cmd_pos_rel_axis_f4(ADDR_M2, POS_SPEED_RPM, POS_ACC, rel_m2))

                last_angle_f = ang_f

                sys.stdout.write(
                    f"\rAZ_raw={az:+6d}  Ang={ang:7.2f}  AngF={ang_f:7.2f}  d={ddeg:+6.2f} deg  "
                    f"rel={rel_counts:+7d}  M1={rel_m1:+7d} M2={rel_m2:+7d}     "
                )
                sys.stdout.flush()

            t_next += period
            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                t_next = time.monotonic()

    except KeyboardInterrupt:
        print("\nSaliendo (no se envían más tramas que enable + F4).")

    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            bus.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
