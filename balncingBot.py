#!/usr/bin/env python3
import time
import sys
import serial
import pygame
from dataclasses import dataclass

# ===================== CONFIG (RS485 + JOYSTICK) =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05

ADDR_LEFT  = 0x01
ADDR_RIGHT = 0x02

MAX_RPM = 30
ACC = 255

DEADZONE = 0.08
UPDATE_HZ = 50

AXIS_THROTTLE = 1
AXIS_TURN     = 3

INVERT_LEFT  = False
INVERT_RIGHT = True
# ====================================================================

# ===================== CONFIG (GY-521 / MPU6050) =====================
I2C_BUS = 1
MPU_ADDR = 0x68  # típico si AD0=0; si AD0=1 -> 0x69

# Registros MPU6050
REG_PWR_MGMT_1   = 0x6B
REG_GYRO_XOUT_H  = 0x43  # GX_H=0x43, GX_L=0x44

# Sensibilidad típica por defecto (FS_SEL=0 => ±250 dps) => 131 LSB/(°/s)
GYRO_LSB_PER_DPS = 131.0

# Calibración bias al arranque
CAL_SAMPLES = 500

# Protección contra jitter (si el loop se bloquea, evita integrar dt enorme)
DT_MAX = 0.1  # s
# ====================================================================

# --- I2C backend (smbus2 preferente, si no smbus) ---
try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus  # type: ignore
    except ImportError:
        SMBus = None


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def dz(x: float, dead: float) -> float:
    return 0.0 if abs(x) < dead else x

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def frame(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_enable(addr: int, en: bool) -> bytes:
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_speed(addr: int, rpm: int, acc: int) -> bytes:
    acc_u8 = int(clamp(acc, 0, 255))
    if rpm >= 0:
        direction_bit = 0
        speed = rpm
    else:
        direction_bit = 1
        speed = -rpm

    speed = int(clamp(speed, 0, MAX_RPM))
    b4 = ((direction_bit & 0x01) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    payload = bytes([b4, b5, acc_u8])
    return frame(addr, 0xF6, payload)

@dataclass
class MotorCmd:
    left_rpm: int
    right_rpm: int

def mix_differential(throttle: float, turn: float) -> MotorCmd:
    left  = throttle - turn
    right = throttle + turn

    m = max(1.0, abs(left), abs(right))
    left /= m
    right /= m

    left_rpm = int(round(left * MAX_RPM))
    right_rpm = int(round(right * MAX_RPM))

    if INVERT_LEFT:
        left_rpm = -left_rpm
    if INVERT_RIGHT:
        right_rpm = -right_rpm

    return MotorCmd(left_rpm=left_rpm, right_rpm=right_rpm)

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

# ===================== MPU6050 helpers =====================

def i2c_require():
    if SMBus is None:
        raise RuntimeError(
            "No se encontró smbus2/smbus. Instala:\n"
            "  pip3 install smbus2\n"
            "y habilita i2c (raspi-config) + permisos /dev/i2c-1."
        )

def read_i16_be(bus: SMBus, addr: int, reg_hi: int) -> int:
    hi = bus.read_byte_data(addr, reg_hi)
    lo = bus.read_byte_data(addr, reg_hi + 1)
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v

def mpu_wake(bus: SMBus) -> None:
    # Mínimo: salir de sleep
    bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.05)

def calibrate_gyro_x_bias(bus: SMBus) -> float:
    """
    Bias del gyro X en °/s, calculado UNA SOLA VEZ al arranque.
    Sensor quieto durante CAL_SAMPLES.
    """
    s = 0.0
    for _ in range(CAL_SAMPLES):
        raw_gx = read_i16_be(bus, MPU_ADDR, REG_GYRO_XOUT_H)
        s += (raw_gx / GYRO_LSB_PER_DPS)
        time.sleep(0.001)
    return s / float(CAL_SAMPLES)

# ===================== main =====================

def main() -> int:
    i2c_require()

    # I2C init
    bus = SMBus(I2C_BUS)
    try:
        mpu_wake(bus)
    except Exception as e:
        print(f"ERROR I2C: no se pudo inicializar MPU6050 en 0x{MPU_ADDR:02X}: {e}", file=sys.stderr)
        bus.close()
        return 3

    # --- Calibración SOLO AL INICIO ---
    print("Calibrando gyro X (una sola vez). Mantén el GY-521 quieto...")
    try:
        gyro_x_bias_dps = calibrate_gyro_x_bias(bus)
    except Exception as e:
        print(f"ERROR I2C: no se pudo calibrar gyro X: {e}", file=sys.stderr)
        bus.close()
        return 4

    angle_x_deg = 0.0  # cero sólo al inicio
    print(f"OK. Bias Gx={gyro_x_bias_dps:.6f} °/s | Ángulo X inicial=0.000 °")

    # RS485 init
    ser = open_serial()

    # Joystick init
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("ERROR: no hay joystick detectado por pygame.")
        ser.close()
        bus.close()
        return 2

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Joystick: {joy.get_name()} | axes={joy.get_numaxes()} buttons={joy.get_numbuttons()}")

    # Enable motores
    send(ser, cmd_enable(ADDR_LEFT, True))
    send(ser, cmd_enable(ADDR_RIGHT, True))
    time.sleep(0.05)

    # Timing
    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()
    t_prev = t_next

    try:
        while True:
            pygame.event.pump()

            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            if dt < 0:
                dt = 0.0
            elif dt > DT_MAX:
                dt = DT_MAX  # evita “saltos” enormes si el loop se bloquea

            # --- Gyro X -> ángulo ---
            try:
                raw_gx = read_i16_be(bus, MPU_ADDR, REG_GYRO_XOUT_H)
                gx_dps = (raw_gx / GYRO_LSB_PER_DPS) - gyro_x_bias_dps
                angle_x_deg += gx_dps * dt
            except Exception as e:
                print(f"\nWARNING I2C: lectura gyro X falló: {e}", file=sys.stderr)
                gx_dps = 0.0

            # --- Joystick -> diferencial ---
            thr = -joy.get_axis(AXIS_THROTTLE)
            trn = joy.get_axis(AXIS_TURN)

            thr = clamp(dz(thr, DEADZONE), -1.0, 1.0)
            trn = clamp(dz(trn, DEADZONE), -1.0, 1.0)

            mc = mix_differential(thr, trn)

            # --- RS485 send ---
            send(ser, cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
            send(ser, cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

            # --- Print status (una línea) ---
            sys.stdout.write(
                f"\rAngX={angle_x_deg:+08.3f} deg | Gx={gx_dps:+07.3f} dps | "
                f"L={mc.left_rpm:+04d} rpm R={mc.right_rpm:+04d} rpm   "
            )
            sys.stdout.flush()

            # Loop timing
            t_next += period
            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                t_next = time.monotonic()

    except KeyboardInterrupt:
        print("\nCTRL+C -> Parando motores...")

    finally:
        try:
            send(ser, cmd_speed(ADDR_LEFT, 0, 0))
            send(ser, cmd_speed(ADDR_RIGHT, 0, 0))
            time.sleep(0.05)
            send(ser, cmd_enable(ADDR_LEFT, False))
            send(ser, cmd_enable(ADDR_RIGHT, False))
        except Exception as e:
            print(f"WARNING: error al parar/disable: {e}", file=sys.stderr)

        try:
            ser.close()
        except Exception:
            pass
        try:
            bus.close()
        except Exception:
            pass
        pygame.quit()

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
