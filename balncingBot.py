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
I2C_BUS = 1          # Raspberry Pi: normalmente 1
MPU_ADDR = 0x68      # GY-521 típico (AD0=0). Si AD0=1 -> 0x69

# Registros MPU6050
REG_PWR_MGMT_1 = 0x6B
REG_GYRO_ZOUT_H = 0x47  # GZ_H, GZ_L están en 0x47 y 0x48

# Sensibilidad por defecto (FS_SEL=0 => ±250 dps) => 131 LSB/(°/s)
GYRO_LSB_PER_DPS = 131.0

# Calibración de bias del gyro Z al arrancar (muestras)
CAL_SAMPLES = 500
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
            "y asegúrate de tener i2c habilitado (raspi-config) y permisos."
        )

def read_i16_be(bus: SMBus, addr: int, reg_hi: int) -> int:
    """Lee un int16 big-endian desde reg_hi y reg_hi+1."""
    hi = bus.read_byte_data(addr, reg_hi)
    lo = bus.read_byte_data(addr, reg_hi + 1)
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v

def mpu_wake(bus: SMBus) -> None:
    # Mínimo imprescindible: salir de SLEEP (PWR_MGMT_1)
    bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.05)

def calibrate_gyro_z_bias(bus: SMBus) -> float:
    """
    Calcula bias del gyro Z en °/s promediando CAL_SAMPLES lecturas.
    Durante esta fase el sensor debe estar quieto.
    """
    acc = 0.0
    for _ in range(CAL_SAMPLES):
        raw_gz = read_i16_be(bus, MPU_ADDR, REG_GYRO_ZOUT_H)
        acc += (raw_gz / GYRO_LSB_PER_DPS)
        # Pequeña espera para no saturar I2C; no fija la tasa de loop principal.
        time.sleep(0.001)
    return acc / float(CAL_SAMPLES)

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

    print("Calibrando gyro Z (mantén el GY-521 quieto)...")
    try:
        gyro_z_bias_dps = calibrate_gyro_z_bias(bus)
    except Exception as e:
        print(f"ERROR I2C: no se pudo calibrar gyro Z: {e}", file=sys.stderr)
        bus.close()
        return 4

    yaw_deg = 0.0
    print(f"Bias gyro Z = {gyro_z_bias_dps:.6f} °/s | Yaw inicial = 0.000 °")

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

    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()
    t_prev = t_next

    try:
        while True:
            pygame.event.pump()

            # --- GYRO Z -> yaw ---
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now

            try:
                raw_gz = read_i16_be(bus, MPU_ADDR, REG_GYRO_ZOUT_H)
                gz_dps = (raw_gz / GYRO_LSB_PER_DPS) - gyro_z_bias_dps
                yaw_deg += gz_dps * dt
            except Exception as e:
                # Si falla I2C puntualmente, no matamos el robot; sólo avisamos.
                print(f"\nWARNING I2C: lectura gyro Z falló: {e}", file=sys.stderr)
                gz_dps = 0.0

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
            # \r para refrescar en la misma línea
            sys.stdout.write(
                f"\rYaw(Z)={yaw_deg:+08.3f} deg | Gz={gz_dps:+07.3f} dps | L={mc.left_rpm:+04d} rpm R={mc.right_rpm:+04d} rpm   "
            )
            sys.stdout.flush()

            # Timing
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
