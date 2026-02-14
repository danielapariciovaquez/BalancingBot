#!/usr/bin/env python3
import time
import sys
import math
import serial
import threading
from dataclasses import dataclass

# ===================== RS485 =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05

ADDR_LEFT  = 0x01
ADDR_RIGHT = 0x02

MAX_RPM = 300
ACC = 255

INTER_FRAME_DELAY_S = 0.004
ENABLE_RETRIES = 2
ENABLE_RETRY_DELAY_S = 0.02

INVERT_LEFT  = False
INVERT_RIGHT = True
# =================================================

# ===================== CONTROL =====================
UPDATE_HZ = 150
DT_MAX = 0.05
ANGLE_CUTOFF_DEG = 35.0

SETPOINT_DEG = 0.0

MAX_RPM_STEP_PER_S = 700.0
BAL_MAX_RPM = 120
I_LIM = 200.0
# ===================================================

# ===================== IMU =====================
I2C_BUS = 1
MPU_ADDR = 0x68

REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_YOUT_H  = 0x45

ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

CAL_SAMPLES_GYRO  = 800
CAL_SAMPLES_ACCEL = 200
# =================================================

# ===================== WEB =====================
WEB_HOST = "0.0.0.0"
WEB_PORT = 8000
# =================================================

try:
    from smbus2 import SMBus
except:
    SMBus = None

try:
    from flask import Flask, request, jsonify
except:
    Flask = None


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF


def frame(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])


def cmd_enable(addr: int, en: bool) -> bytes:
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))


def cmd_speed(addr: int, rpm: int, acc: int) -> bytes:
    acc_u8 = int(clamp(acc, 0, 255))
    direction_bit = 0 if rpm >= 0 else 1
    speed = abs(int(rpm))
    speed = int(clamp(speed, 0, MAX_RPM))
    b4 = ((direction_bit & 0x01) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return frame(addr, 0xF6, bytes([b4, b5, acc_u8]))


@dataclass
class MotorCmd:
    left_rpm: int
    right_rpm: int


def motors_from_balance(base_rpm: float) -> MotorCmd:
    left = base_rpm
    right = base_rpm
    if INVERT_LEFT:
        left = -left
    if INVERT_RIGHT:
        right = -right
    return MotorCmd(int(left), int(right))


# ===================== IMU helpers =====================
def read_i16_be(bus, addr, reg):
    hi = bus.read_byte_data(addr, reg)
    lo = bus.read_byte_data(addr, reg+1)
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v


def mpu_wake(bus):
    bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.05)


def read_accel_gyro(bus):
    ax = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H)
    az = read_i16_be(bus, MPU_ADDR, REG_ACCEL_XOUT_H + 4)
    gy = read_i16_be(bus, MPU_ADDR, REG_GYRO_YOUT_H)
    return ax, az, gy


def accel_angle(ax, az):
    ax_g = ax / ACC_LSB_PER_G
    az_g = az / ACC_LSB_PER_G
    return -math.degrees(math.atan2(ax_g, az_g))


# ===================== Kalman =====================
class Kalman1D:
    def __init__(self):
        self.q_angle = 0.001
        self.q_bias = 0.003
        self.r_measure = 0.03
        self.angle = 0.0
        self.bias = 0.0
        self.P00 = 1.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 1.0

    def update(self, meas, gyro_rate, dt):
        rate = gyro_rate - self.bias
        self.angle += dt * rate

        P00 = self.P00 + dt * (dt*self.P11 - self.P01 - self.P10 + self.q_angle)
        P01 = self.P01 - dt * self.P11
        P10 = self.P10 - dt * self.P11
        P11 = self.P11 + self.q_bias * dt
        self.P00, self.P01, self.P10, self.P11 = P00, P01, P10, P11

        y = meas - self.angle
        S = self.P00 + self.r_measure
        K0 = self.P00 / S
        K1 = self.P10 / S

        self.angle += K0 * y
        self.bias  += K1 * y

        self.P00 -= K0 * self.P00
        self.P01 -= K0 * self.P01
        self.P10 -= K1 * self.P00
        self.P11 -= K1 * self.P01

        return self.angle


# ===================== MAIN =====================
def main():
    if SMBus is None:
        print("Instala smbus2")
        return 1

    bus = SMBus(I2C_BUS)
    mpu_wake(bus)

    print("Calibrando...")
    gyro_bias = 0.0
    for _ in range(CAL_SAMPLES_GYRO):
        _, _, gy = read_accel_gyro(bus)
        gyro_bias += gy / GYRO_LSB_PER_DPS
        time.sleep(0.001)
    gyro_bias /= CAL_SAMPLES_GYRO

    acc_zero = 0.0
    for _ in range(CAL_SAMPLES_ACCEL):
        ax, az, _ = read_accel_gyro(bus)
        acc_zero += accel_angle(ax, az)
        time.sleep(0.001)
    acc_zero /= CAL_SAMPLES_ACCEL

    kf = Kalman1D()

    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_S)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(cmd_enable(ADDR_LEFT, True))
    ser.write(cmd_enable(ADDR_RIGHT, True))

    integ = 0.0
    prev_err = 0.0
    base_cmd = 0.0

    period = 1.0 / UPDATE_HZ
    t_prev = time.monotonic()

    while True:
        now = time.monotonic()
        dt = now - t_prev
        t_prev = now
        if dt > DT_MAX:
            dt = DT_MAX

        ax, az, gy = read_accel_gyro(bus)

        acc_ang = accel_angle(ax, az) - acc_zero
        gyro_rate = (gy / GYRO_LSB_PER_DPS) - gyro_bias

        angle = kf.update(acc_ang, gyro_rate, dt)

        if abs(angle) > ANGLE_CUTOFF_DEG:
            ser.write(cmd_speed(ADDR_LEFT, 0, 0))
            ser.write(cmd_speed(ADDR_RIGHT, 0, 0))
            integ = 0.0
            prev_err = 0.0
            base_cmd = 0.0
            continue

        err = SETPOINT_DEG - angle
        integ += err * dt
        integ = clamp(integ, -I_LIM, I_LIM)

        derr = (err - prev_err) / dt if dt > 0 else 0.0
        prev_err = err

        base = (18.0 * err) + (0.0 * integ) + (0.9 * derr)
        base = clamp(base, -BAL_MAX_RPM, BAL_MAX_RPM)

        max_step = MAX_RPM_STEP_PER_S * dt
        delta = clamp(base - base_cmd, -max_step, max_step)
        base_cmd += delta

        mc = motors_from_balance(base_cmd)

        ser.write(cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
        ser.write(cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

        time.sleep(period)


if __name__ == "__main__":
    main()
