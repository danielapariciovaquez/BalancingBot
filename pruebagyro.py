from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68)  # usa 0x69 si AD0 está a 3.3V

while True:
    accel = sensor.get_accel_data()
    gyro  = sensor.get_gyro_data()
    temp  = sensor.get_temp()

    print("Accel (m/s^2):", accel)
    print("Gyro  (deg/s):", gyro)
    print("Temp  (°C):", temp)
    print("----")

    time.sleep(0.5)

