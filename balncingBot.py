#!/usr/bin/env python3
import time
import serial

# ================= CONFIG =================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT = 0.2

ADDR1 = 0x01
ADDR2 = 0x02

RPM_TEST = 100
ACC = 10      # aceleración F6 (0..255)
RUN_TIME = 5  # segundos
# ==========================================

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def frame(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_set_mode(addr: int) -> bytes:
    # 82H = set work mode, 05H = SR_vFOC
    return frame(addr, 0x82, bytes([0x05]))

def cmd_enable(addr: int, en: bool) -> bytes:
    # F3H enable: 01 = enable, 00 = disable
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_speed(addr: int, rpm: int, acc: int) -> bytes:
    # F6H speed command
    dir_bit = 1 if rpm < 0 else 0
    speed = abs(rpm)
    if speed > 3000:
        speed = 3000

    b4 = ((dir_bit & 1) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return frame(addr, 0xF6, bytes([b4, b5, acc & 0xFF]))

def stop(addr: int) -> bytes:
    return cmd_speed(addr, 0, ACC)

# ================= MAIN =================
with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
    print("Inicializando motores...")

    # Modo SR_vFOC
    ser.write(cmd_set_mode(ADDR1)); ser.flush(); time.sleep(0.05)
    ser.write(cmd_set_mode(ADDR2)); ser.flush(); time.sleep(0.05)

    # Enable
    ser.write(cmd_enable(ADDR1, True)); ser.flush(); time.sleep(0.05)
    ser.write(cmd_enable(ADDR2, True)); ser.flush(); time.sleep(0.05)

    print(f"Motores a {RPM_TEST} rpm...")
    ser.write(cmd_speed(ADDR1, RPM_TEST, ACC)); ser.flush()
    ser.write(cmd_speed(ADDR2, RPM_TEST, ACC)); ser.flush()

    time.sleep(RUN_TIME)

    print("STOP")
    ser.write(stop(ADDR1)); ser.flush()
    ser.write(stop(ADDR2)); ser.flush()

    time.sleep(0.2)

    # Disable (opcional)
    ser.write(cmd_enable(ADDR1, False)); ser.flush()
    ser.write(cmd_enable(ADDR2, False)); ser.flush()

print("Fin.")
