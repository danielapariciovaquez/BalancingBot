#!/usr/bin/env python3
import time
import math
from smbus2 import SMBus

I2C_BUS = 1
MPU_ADDR = 0x68

# Registros
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# Sensibilidades por defecto (FS=±2g y ±250°/s)
ACCEL_SCALE = 16384.0     # LSB/g
GYRO_SCALE = 131.0        # LSB/(°/s)


def read_word(bus, reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def main():
    with SMBus(I2C_BUS) as bus:
        # Wake up MPU6050
        bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

        print("MPU6050 iniciado correctamente")

        while True:
            ax = read_word(bus, ACCEL_XOUT_H) / ACCEL_SCALE
            ay = read_word(bus, ACCEL_XOUT_H + 2) / ACCEL_SCALE
            az = read_word(bus, ACCEL_XOUT_H + 4) / ACCEL_SCALE

            gx = read_word(bus, ACCEL_XOUT_H + 8) / GYRO_SCALE
            gy = read_word(bus, ACCEL_XOUT_H + 10) / GYRO_SCALE
            gz = read_word(bus, ACCEL_XOUT_H + 12) / GYRO_SCALE

            print(f"A[g]  X:{ax: .3f}  Y:{ay: .3f}  Z:{az: .3f}")
            print(f"G[°/s] X:{gx: .3f}  Y:{gy: .3f}  Z:{gz: .3f}")
            print("-" * 50)

            time.sleep(0.2)


if __name__ == "__main__":
    main()
