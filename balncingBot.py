import serial, time

PORT="/dev/ttyUSB0"
BAUD=38400
ADDR=0x01
ACC=2

def checksum8(p): return sum(p) & 0xFF

def f3(enable: bool):
    p = bytes([0xFA, ADDR, 0xF3, 0x01 if enable else 0x00])
    return p + bytes([checksum8(p)])

def f6(rpm: int):
    if rpm < 0:
        direction = 1
        speed = -rpm
    else:
        direction = 0
        speed = rpm
    speed = max(0, min(3000, speed))
    b4 = ((direction & 1) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    p = bytes([0xFA, ADDR, 0xF6, b4, b5, ACC])
    return p + bytes([checksum8(p)])

ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.2)

def send(fr):
    ser.write(fr); ser.flush()
    time.sleep(0.02)

send(f3(True))
time.sleep(0.2)

try:
    while True:
        send(f6(100))
        print("100")
        time.sleep(2)
        send(f6(0))
        print("0")
        time.sleep(2)
except KeyboardInterrupt:
    send(f6(0))
    send(f3(False))
    ser.close()
