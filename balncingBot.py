#!/usr/bin/env python3
import time
import sys
import math
import serial
import pygame
from dataclasses import dataclass

# ===================== CONFIG (RS485 + JOYSTICK) =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05

ADDR_LEFT  = 0x01
ADDR_RIGHT = 0x02

MAX_RPM = 50
ACC = 255

DEADZONE = 0.01

# Para balanceo suele interesar más frecuencia que 50 Hz.
# Si tu RPi va bien, 100-200 Hz suele ser razonable. Aquí dejo 100 Hz por defecto.
UPDATE_HZ = 100

# Joystick (tus valores memorizados)
AXIS_THROTTLE = 1
AXIS_TURN     = 3

INVERT_LEFT  = False
INVERT_RIGHT = True
# ====================================================================

# ===================== CONFIG (GY-521 / MPU6050) =====================
I2C_BUS = 1
MPU_ADDR = 0x68  # AD0=0 típico; si AD0=1 -> 0x69

# Registros
REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B  # AX_H..AZ_L son 0x3B..0x40
REG_GYRO_YOUT_H  = 0x45  # GY_H=0x45, GY_L=0x46

# Escalas por defecto típicas si no configuras rangos:
# Accel ±2g => 16384 LSB/g
# Gyro  ±250 dps => 131 LSB/(°/s)
ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

# Calibraciones al arranque (sensor quieto y robot en posición vertical de equilibrio)
CAL_SAMPLES_GYRO = 800
CAL_SAMPLES_ACCEL = 200

# Seguridad
ANGLE_CUTOFF_DEG = 35.0  # si se cae más allá, corta salida de motor
DT_MAX = 0.05            # clamp dt para evitar integraciones gigantes por jitter
# ====================================================================

# ===================== BALANCE CONTROL =====================
# Setpoint de equilibrio (0 deg tras calibrar)
SETPOINT_DEG = 0.0

# Joystick -> setpoint (lean-to-go)
# throttle [-1..1] se convierte a offset de setpoint (grados).
# AJUSTA este límite según tu mecánica; no hay valor universal.
MAX_SETPOINT_OFFSET_DEG = 10.0

# PID sobre el ángulo (salida en "rpm base")
# OJO: hay que tunear en tu robot. Empezar con Ki=0 suele ser más seguro.
Kp = 10.0
Ki = 0.5
Kd = 0.8

# Limitación integral (anti-windup) en unidades de "rpm equivalente"
I_LIM = 200.0
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
    bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.05)

def read_accel_gyro(bus: SMBus):
    # Accel X,Z y Gyro Y
    ax = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H)       # 0x3B
    # ay = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H + 2)  # 0x3D (no usado)
    az = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H + 4)   # 0x3F
    gy = read_i16_be(bus, MPU_ADDR, REG_GYRO_YOUT_H)        # 0x45
    return ax, az, gy

def accel_angle_deg_from_ax_az(ax_raw: int, az_raw: int) -> float:
    """
    Ángulo de pitch derivado de acelerómetro usando AX y AZ.
    El resultado depende de cómo esté orientado tu módulo.
    Aquí usamos atan2(ax, az).
    REQUISITO tuyo: al inclinar hacia adelante, el ángulo DECREMENTA.
    Para imponerlo, invertimos el signo.
    """
    ax_g = ax_raw / ACC_LSB_PER_G
    az_g = az_raw / ACC_LSB_PER_G
    ang = math.degrees(math.atan2(ax_g, az_g))
    return -ang  # <-- fuerza: forward tilt => ángulo decrece

class Kalman1D:
    """
    Kalman 1D típico para ángulo con bias de gyro:
      state: [angle, bias]
      input: gyro_rate (deg/s)
      measurement: accel_angle (deg)
    """
    def __init__(self, q_angle=0.001, q_bias=0.003, r_measure=0.03):
        self.q_angle = float(q_angle)
        self.q_bias = float(q_bias)
        self.r_measure = float(r_measure)

        self.angle = 0.0
        self.bias = 0.0
        self.P00 = 1.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 1.0

    def set_angle(self, angle_deg: float):
        self.angle = float(angle_deg)

    def update(self, meas_angle_deg: float, gyro_rate_dps: float, dt: float) -> float:
        # Predict
        rate = gyro_rate_dps - self.bias
        self.angle += dt * rate

        # Covariance predict
        P00 = self.P00 + dt * (dt*self.P11 - self.P01 - self.P10 + self.q_angle)
        P01 = self.P01 - dt * self.P11
        P10 = self.P10 - dt * self.P11
        P11 = self.P11 + self.q_bias * dt

        self.P00, self.P01, self.P10, self.P11 = P00, P01, P10, P11

        # Update
        y = meas_angle_deg - self.angle
        S = self.P00 + self.r_measure
        K0 = self.P00 / S
        K1 = self.P10 / S

        self.angle += K0 * y
        self.bias  += K1 * y

        P00 = self.P00 - K0 * self.P00
        P01 = self.P01 - K0 * self.P01
        P10 = self.P10 - K1 * self.P00
        P11 = self.P11 - K1 * self.P01

        self.P00, self.P01, self.P10, self.P11 = P00, P01, P10, P11
        return self.angle

def calibrate_gyro_y_bias(bus: SMBus) -> float:
    s = 0.0
    for _ in range(CAL_SAMPLES_GYRO):
        _, _, gy_raw = read_accel_gyro(bus)
        s += (gy_raw / GYRO_LSB_PER_DPS)
        time.sleep(0.001)
    return s / float(CAL_SAMPLES_GYRO)

def calibrate_accel_angle_zero(bus: SMBus) -> float:
    """
    Promedia el ángulo del acelerómetro en la posición inicial.
    Se usa como offset para que el ángulo inicial sea 0 deg.
    """
    s = 0.0
    for _ in range(CAL_SAMPLES_ACCEL):
        ax, az, _ = read_accel_gyro(bus)
        s += accel_angle_deg_from_ax_az(ax, az)
        time.sleep(0.001)
    return s / float(CAL_SAMPLES_ACCEL)

def motors_from_balance(base_rpm: float, turn_rpm: float) -> MotorCmd:
    left = base_rpm - turn_rpm
    right = base_rpm + turn_rpm

    left_rpm = int(round(clamp(left, -MAX_RPM, MAX_RPM)))
    right_rpm = int(round(clamp(right, -MAX_RPM, MAX_RPM)))

    if INVERT_LEFT:
        left_rpm = -left_rpm
    if INVERT_RIGHT:
        right_rpm = -right_rpm

    return MotorCmd(left_rpm=left_rpm, right_rpm=right_rpm)

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

    print("Calibrando IMU (quieto y en posición de equilibrio)...")
    try:
        gyro_y_bias_dps = calibrate_gyro_y_bias(bus)
        accel_zero_deg = calibrate_accel_angle_zero(bus)
    except Exception as e:
        print(f"ERROR I2C: calibración falló: {e}", file=sys.stderr)
        bus.close()
        return 4

    # Kalman init: arrancamos el ángulo en 0 tras compensar offset
    kf = Kalman1D()
    kf.set_angle(0.0)

    print(f"OK. Bias Gy={gyro_y_bias_dps:.6f} °/s | AccelZero={accel_zero_deg:.6f} ° | Ang inicial=0.000°")

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

    # PID state
    integ = 0.0
    prev_err = 0.0

    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()
    t_prev = t_next

    try:
        while True:
            pygame.event.pump()

            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            if dt < 0.0:
                dt = 0.0
            elif dt > DT_MAX:
                dt = DT_MAX

            # --- IMU read ---
            try:
                ax, az, gy_raw = read_accel_gyro(bus)
                accel_angle = accel_angle_deg_from_ax_az(ax, az) - accel_zero_deg

                # gyro rate (deg/s), usando eje Y
                gyro_rate = (gy_raw / GYRO_LSB_PER_DPS) - gyro_y_bias_dps

                # coherencia de signo: si forward tilt => angle decrece.
                # Ya lo impusimos en accel_angle; el gyro debe seguir el mismo convenio.
                # Si al inclinar hacia adelante el gyro_rate sale con signo contrario al accel,
                # invierte aquí: gyro_rate = -gyro_rate
                # (lo dejamos como está por defecto; se corrige con una sola inversión si hiciera falta)

                angle = kf.update(accel_angle, gyro_rate, dt)

            except Exception as e:
                print(f"\nWARNING I2C: lectura IMU falló: {e}", file=sys.stderr)
                angle = kf.angle
                gyro_rate = 0.0
                accel_angle = 0.0

            # --- Safety cutoff ---
            if abs(angle) > ANGLE_CUTOFF_DEG:
                # Cae: corta mando a 0 hasta que el ángulo vuelva dentro de rango
                mc = MotorCmd(0, 0)
                send(ser, cmd_speed(ADDR_LEFT, 0, 0))
                send(ser, cmd_speed(ADDR_RIGHT, 0, 0))
                sys.stdout.write(f"\rCAIDO: Ang={angle:+07.2f} deg (cutoff) -> motores 0 rpm                 ")
                sys.stdout.flush()
                # sigue loop sin integrar PID (evita windup)
                integ = 0.0
                prev_err = 0.0
                # timing
                t_next += period
                sleep_s = t_next - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    t_next = time.monotonic()
                continue

            # --- Joystick -> setpoint y giro ---
            thr = -joy.get_axis(AXIS_THROTTLE)
            trn = joy.get_axis(AXIS_TURN)

            thr = clamp(dz(thr, DEADZONE), -1.0, 1.0)
            trn = clamp(dz(trn, DEADZONE), -1.0, 1.0)

            setpoint = SETPOINT_DEG + thr * MAX_SETPOINT_OFFSET_DEG

            # --- PID balance -> base rpm ---
            err = setpoint - angle

            # Integral (anti-windup)
            integ += err * dt
            integ = clamp(integ, -I_LIM, I_LIM)

            derr = (err - prev_err) / dt if dt > 0 else 0.0
            prev_err = err

            # Salida PID en rpm (escala directa por gains)
            base_rpm = (Kp * err) + (Ki * integ) + (Kd * derr)
            base_rpm = clamp(base_rpm, -MAX_RPM, MAX_RPM)

            # Turn: lo aplicamos como diferencial en rpm
            turn_rpm = trn * MAX_RPM
            mc = motors_from_balance(base_rpm, turn_rpm)

            # --- RS485 send ---
            send(ser, cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
            send(ser, cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

            # --- Print status ---
            sys.stdout.write(
                f"\rAng={angle:+07.2f} deg | Acc={accel_angle:+07.2f} | Gy={gyro_rate:+07.2f} dps | "
                f"SP={setpoint:+06.2f} | base={base_rpm:+06.1f} | L={mc.left_rpm:+04d} R={mc.right_rpm:+04d}    "
            )
            sys.stdout.flush()

            # timing
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
