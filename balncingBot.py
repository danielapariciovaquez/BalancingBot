import time
import serial

PORT = "ttyUSB0"  # <-- cambia esto
BAUDRATE = 38400

ADDRS = [0x01, 0x02]
ACC = 2
RPM_RUN = 100
T_RUN = 5.0
T_STOP = 5.0

def checksum8(p: bytes) -> int:
    return sum(p) & 0xFF

def frame_f3(addr: int, en: bool) -> bytes:
    p = bytes([0xFA, addr, 0xF3, 0x01 if en else 0x00])
    return p + bytes([checksum8(p)])

def frame_f6(addr: int, rpm: int, acc: int) -> bytes:
    if rpm < 0:
        direction = 1
        speed = -rpm
    else:
        direction = 0
        speed = rpm
    speed = max(0, min(3000, int(speed)))
    b4 = ((direction & 1) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    p = bytes([0xFA, addr, 0xF6, b4, b5, acc & 0xFF])
    return p + bytes([checksum8(p)])

def send(ser: serial.Serial, fr: bytes):
    ser.write(fr)
    ser.flush()
    time.sleep(0.02)  # 20 ms

with serial.Serial(PORT, BAUDRATE, timeout=0.2, write_timeout=0.2) as ser:
    # Enable una vez
    for a in ADDRS:
        send(ser, frame_f3(a, True))
    time.sleep(0.2)

    while True:
        # RUN
        for a in ADDRS:
            send(ser, frame_f6(a, RPM_RUN, ACC))
        time.sleep(T_RUN)

        # STOP
        for a in ADDRS:
            send(ser, frame_f6(a, 0, ACC))
        time.sleep(T_STOP)
