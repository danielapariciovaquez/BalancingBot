#!/usr/bin/env python3
import time
import serial

PORT="/dev/ttyUSB0"
BAUD=38400
TIMEOUT=0.2
ACC=2  # acc!=0 => stop suave; acc=0 => stop inmediato

def checksum8(b: bytes) -> int:
    return sum(b) & 0xFF

def frame(addr: int, code: int, data: bytes=b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum8(base)])

def f6_stop(addr: int, acc: int) -> bytes:
    # dir=0, speed=0 => Byte4=0x00, Byte5=0x00
    return frame(addr, 0xF6, bytes([0x00, 0x00, acc & 0xFF]))

with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
    # STOP a direcciones que crees + broadcast(0)
    # 0 es broadcast según manual. :contentReference[oaicite:3]{index=3}
    for addr in (0x00, 0x01, 0x02):
        tx = f6_stop(addr, ACC)
        ser.write(tx); ser.flush()
        time.sleep(0.05)

print("STOP enviado.")
