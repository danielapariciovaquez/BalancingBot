#!/usr/bin/env python3
import time
import math
import sys
import serial

# ===================== RS485 / MOTORES =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05

ADDR_M1 = 0x01
ADDR_M2 = 0x02

INTER_FRAME_DELAY_S = 0.004
ENABLE_RETRIES = 2
ENABLE_RETRY_DELAY_S = 0.02

# Comando F4 requiere speed + acc (aunque tú solo quieras "posición")
# Speed range: 0..3000 RPM, acc: 0..255 :contentReference[oaicite:2]{index=2}
POS_SPEED_RPM = 30
POS_ACC = 255
# ===========================================================

# ===================== I2C / MPU6050 (GY-521) =====================
I2C_BUS = 1
MPU_ADDR = 0x68

REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B  # AX..AZ

ACC_LSB_PER_G = 16384.0  # ±2g default si no configuras rango
# ===============================================================

# ===================== Conversión grados -> axis =====================
COUNTS_PER_TURN = 16384  # 16384/turn :contentReference[oaicite:3]{index=3}
DEG_PER_TURN = 360.0
COUNTS_PER_DEG = COUNTS_PER_TURN / DEG_PER_TURN
# ===============================================================

UPDATE_HZ = 50
ANGLE_DEADBAND_DEG = 0.2  # evita spam de tramas por ruido
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
    Downlink: FA addr F4 speed(2B) acc(1B) relAxis(int32) CRC :contentReference[oaicite:4]{index=4}
    speed: 0..3000 RPM, acc: 0..255, relAxis: int32_t :contentReference[oaicite:5]{index=5}
    """
    speed_rpm = clamp_int(int(speed_rpm), 0, 3000)
    acc = clamp_int(int(acc), 0, 255)
    rel_axis = int(rel_axis)

    speed_hi = (speed_rpm >> 8) & 0xFF
    speed_lo = speed_rpm & 0xFF

    # int32 big-endian
    rel_u32 = rel_axis & 0xFFFFFFFF
    b7 = (rel_u32 >> 24) & 0xFF
    b8 = (rel_u32 >> 16) & 0xFF
    b9 = (rel_u32 >> 8) & 0xFF
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
    az = i16(data[4], data[5])
    return az


def az_to_angle_deg(az_raw: int) -> float:
    # az_g clamped a [-1,1] para evitar NaN
    az_g = az_raw / ACC_LSB_PER_G
    az_g = -1.0 if az_g < -1.0 else 1.0 if az_g > 1.0 else az_g
    # 0° -> az_g=+1, 90° -> 0, 180° -> -1
    return math.degrees(math.acos(az_g))


def main() -> int:
    require_i2c()

    bus = SMBus(I2C_BUS)
    mpu_wake(bus)

    ser = open_serial()

    # ===== SOLO: enable motores 1 y 2 =====
    send_enable_robust(ser, ADDR_M1, True)
    send_enable_robust(ser, ADDR_M2, True)

    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()

    # Inicializa referencia de ángulo
    az0 = read_az_raw(bus)
    last_angle = az_to_angle_deg(az0)

    print(f"ENABLE OK. AngleZ(deg) start = {last_angle:.2f}")
    print("Enviando SOLO comandos F4 (posición relativa por axis) a motor 1 y 2... Ctrl+C para salir.")

    try:
        while True:
            az = read_az_raw(bus)
            ang = az_to_angle_deg(az)

            ddeg = ang - last_angle
            if abs(ddeg) >= ANGLE_DEADBAND_DEG:
                rel_counts = int(round(ddeg * COUNTS_PER_DEG))

                # Enviar SOLO posición relativa (F4) a ambos motores
                send_paced(ser, cmd_pos_rel_axis_f4(ADDR_M1, POS_SPEED_RPM, POS_ACC, rel_counts))
                send_paced(ser, cmd_pos_rel_axis_f4(ADDR_M2, POS_SPEED_RPM, POS_ACC, rel_counts))

                last_angle = ang

                sys.stdout.write(f"\rAZ_raw={az:+6d}  AngleZ={ang:7.2f} deg  d={ddeg:+6.2f} deg  relAxis={rel_counts:+7d}     ")
                sys.stdout.flush()

            t_next += period
            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                t_next = time.monotonic()

    except KeyboardInterrupt:
        print("\nSaliendo (no se envían más tramas que las indicadas).")

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
