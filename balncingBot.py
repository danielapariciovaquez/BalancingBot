#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 38400
ADDR = 0x01
ACC  = 0x02

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def frame(addr: int, code: int, payload: bytes=b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def read5(ser: serial.Serial, tag: str):
    # Respuestas “setting status” del manual son 5 bytes (FB addr code status CRC)
    rx = ser.read(5)
    if len(rx) == 5:
        print(f"RX {tag}: {rx.hex(' ')}")
    else:
        print(f"RX {tag}: <sin respuesta> ({len(rx)} bytes)")

with serial.Serial(PORT, BAUD, timeout=0.3) as ser:
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Set work mode SR_vFOC (speed mode): FA 01 82 05 82
    tx = frame(ADDR, 0x82, bytes([0x05]))
    ser.write(tx); ser.flush()
    print("TX mode:", tx.hex(" "))
    read5(ser, "mode")

    time.sleep(0.05)
    ser.reset_input_buffer()

    # ENABLE: FA 01 F3 01 EF
    tx = frame(ADDR, 0xF3, bytes([0x01]))
    ser.write(tx); ser.flush()
    print("TX enable:", tx.hex(" "))
    read5(ser, "enable")

    time.sleep(0.05)
    ser.reset_input_buffer()

    # RUN 10 rpm: F6 speed=0x000A acc=0x02 -> FA 01 F6 00 0A 02 FD
    tx = frame(ADDR, 0xF6, bytes([0x00, 0x0A, ACC]))
    ser.write(tx); ser.flush()
    print("TX run10:", tx.hex(" "))
    read5(ser, "run10")

    time.sleep(1.0)

    # STOP en speed mode: F6 speed=0 acc=2 -> FA 01 F6 00 00 02 F3 (ejemplo del manual)
    tx = frame(ADDR, 0xF6, bytes([0x00, 0x00, ACC]))
    ser.write(tx); ser.flush()
    print("TX stop:", tx.hex(" "))
    read5(ser, "stop")

    time.sleep(0.2)
    ser.reset_input_buffer()

    # DISABLE (loose shaft): FA 01 F3 00 EE
    tx = frame(ADDR, 0xF3, bytes([0x00]))
    ser.write(tx); ser.flush()
    print("TX disable:", tx.hex(" "))
    read5(ser, "disable")
