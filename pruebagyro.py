from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68, bus=2)  # <- aquí el cambio

while True:
    accel = sensor.get_accel_data()
    gyro  = sensor.get_gyro_data()
    temp  = sensor.get_temp()

    print("Accel:", accel)
    print("Gyro :", gyro)
    print("Temp :", temp)
    print("----")
    time.sleep(0.5)
