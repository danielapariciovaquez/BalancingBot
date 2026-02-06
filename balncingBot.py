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

MAX_RPM = 300          # límite absoluto
ACC = 255

DEADZONE = 0.08
UPDATE_HZ = 100        # control más rápido para balanceo

AXIS_THROTTLE = 1
AXIS_TURN     = 3

INVERT_LEFT  = False
INVERT_RIGHT = True

# Limitar sensibilidad de giro (joystick)
# turn_scaled = clamp(turn, -1..1) * TURN_SCALE
TURN_SCALE = 0.20      # 0.2 = 20% del giro original (reduce MUCHO)
# Además limitamos el diferencial máximo a un valor fijo (rpm)
TURN_MAX_RPM = 60      # límite duro del giro (rpm)
# ====================================================================

# ===================== CONFIG (GY-521 / MPU6050) =====================
I2C_BUS = 1
MPU_ADDR = 0x68

REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B  # AX_H..AZ_L
REG_GYRO_YOUT_H  = 0x45  # GY_H..GY_L

ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

CAL_SAMPLES_GYRO  = 800
CAL_SAMPLES_ACCEL = 200

ANGLE_CUTOFF_DEG = 30.0
DT_MAX = 0.05
# ====================================================================

# ===================== KALMAN =====================
# Ajustes más “suaves” para reducir ruido de ángulo
KAL_Q_ANGLE = 0.0005
KAL_Q_BIAS  = 0.003
KAL_R_MEAS  = 0.05
# ====================================================================

# ===================== BALANCE CONTROL (PID) =====================
SETPOINT_DEG = 0.0

MAX_SETPOINT_OFFSET_DEG = 10.0  # throttle -> inclinación deseada

# Gains más conservadores + más amortiguación
Kp = 5
Ki = 0
Kd = 1

# Anti-windup (aunque Ki=0)
I_LIM = 200.0

# Limitación del balanceo (mucho más baja que MAX_RPM para evitar latigazos)
BAL_MAX_RPM = 120

# Derivative low-pass
D_TAU = 0.05  # s

# Slew-rate limit (rpm/s) para evitar vibración por cambios bruscos
MAX_RPM_STEP_PER_S = 800.0
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
    ax = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H)       # 0x3B
    az = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H + 4)   # 0x3F
    gy = read_i16_be(bus, MPU_ADDR, REG_GYRO_YOUT_H)        # 0x45
    return ax, az, gy

def accel_angle_deg_from_ax_az(ax_raw: int, az_raw: int) -> float:
    """
    Pitch a partir de AX/AZ.
    Requisito: inclinación hacia delante => ángulo decrementa.
    """
    ax_g = ax_raw / ACC_LSB_PER_G
    az_g = az_raw / ACC_LSB_PER_G
    ang = math.degrees(math.atan2(ax_g, az_g))
    return -ang  # forward tilt => decrement

class Kalman1D:
    def __init__(self, q_angle=KAL_Q_ANGLE, q_bias=KAL_Q_BIAS, r_measure=KAL_R_MEAS):
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
        rate = gyro_rate_dps - self.bias
        self.angle += dt * rate

        P00 = self.P00 + dt * (dt*self.P11 - self.P01 - self.P10 + self.q_angle)
        P01 = self.P01 - dt * self.P11
        P10 = self.P10 - dt * self.P11
        P11 = self.P11 + self.q_bias * dt
        self.P00, self.P01, self.P10, self.P11 = P00, P01, P10, P11

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

    kf = Kalman1D()
    kf.set_angle(0.0)

    print(f"OK. BiasGy={gyro_y_bias_dps:.6f} °/s | AccZero={accel_zero_deg:.6f} ° | Ang0=0.000°")

    ser = open_serial()

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

    # PID states
    integ = 0.0
    prev_err = 0.0
    d_filt = 0.0
    base_rpm_cmd = 0.0

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

            # IMU
            try:
                ax, az, gy_raw = read_accel_gyro(bus)
                accel_angle = accel_angle_deg_from_ax_az(ax, az) - accel_zero_deg

                gyro_rate = (gy_raw / GYRO_LSB_PER_DPS) - gyro_y_bias_dps
                # Si gyro tiene signo opuesto al accel en tu montaje, descomenta:
                # gyro_rate = -gyro_rate

                angle = kf.update(accel_angle, gyro_rate, dt)

            except Exception as e:
                print(f"\nWARNING I2C: lectura IMU falló: {e}", file=sys.stderr)
                angle = kf.angle
                gyro_rate = 0.0
                accel_angle = 0.0

            # Safety cutoff
            if abs(angle) > ANGLE_CUTOFF_DEG:
                send(ser, cmd_speed(ADDR_LEFT, 0, 0))
                send(ser, cmd_speed(ADDR_RIGHT, 0, 0))
                integ = 0.0
                prev_err = 0.0
                d_filt = 0.0
                base_rpm_cmd = 0.0

                sys.stdout.write(f"\rCAIDO: Ang={angle:+07.2f} deg -> motores 0 rpm                           ")
                sys.stdout.flush()

                t_next += period
                sleep_s = t_next - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    t_next = time.monotonic()
                continue

            # Joystick -> setpoint + turn
            thr = -joy.get_axis(AXIS_THROTTLE)
            trn = joy.get_axis(AXIS_TURN)

            thr = clamp(dz(thr, DEADZONE), -1.0, 1.0)
            trn = clamp(dz(trn, DEADZONE), -1.0, 1.0)

            setpoint = SETPOINT_DEG + thr * MAX_SETPOINT_OFFSET_DEG

            # Giro muy limitado
            trn = clamp(trn * TURN_SCALE, -1.0, 1.0)
            turn_rpm = clamp(trn * MAX_RPM, -TURN_MAX_RPM, +TURN_MAX_RPM)

            # PID balance
            err = setpoint - angle

            integ += err * dt
            integ = clamp(integ, -I_LIM, I_LIM)

            derr = (err - prev_err) / dt if dt > 0 else 0.0
            prev_err = err

            alpha = dt / (D_TAU + dt)
            d_filt += alpha * (derr - d_filt)

            base_rpm = (Kp * err) + (Ki * integ) + (Kd * d_filt)
            base_rpm = clamp(base_rpm, -BAL_MAX_RPM, +BAL_MAX_RPM)

            # Slew-rate
            max_step = MAX_RPM_STEP_PER_S * dt
            delta = clamp(base_rpm - base_rpm_cmd, -max_step, +max_step)
            base_rpm_cmd += delta
            base_rpm_cmd = clamp(base_rpm_cmd, -BAL_MAX_RPM, +BAL_MAX_RPM)

            mc = motors_from_balance(base_rpm_cmd, turn_rpm)

            # RS485 send
            send(ser, cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
            send(ser, cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

            # Debug
            sys.stdout.write(
                f"\rAng={angle:+07.2f} | Acc={accel_angle:+07.2f} | Gy={gyro_rate:+07.2f} dps | "
                f"SP={setpoint:+06.2f} | base={base_rpm_cmd:+06.1f} | turn={turn_rpm:+06.1f} | "
                f"L={mc.left_rpm:+04d} R={mc.right_rpm:+04d}     "
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
