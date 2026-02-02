#!/usr/bin/env python3
import time
import serial

PORT="/dev/ttyUSB0"
BAUD=38400
TIMEOUT=0.08  # corto para escanear rápido
ADDR_RANGE = range(1, 21)  # prueba 1..20 (ajusta si quieres)

def checksum8(b: bytes) -> int:
    return sum(b) & 0xFF

def frame(addr: int, code: int, data: bytes=b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum8(base)])

def hexd(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def read_enable_cmd(addr: int) -> bytes:
    # 3A read enable status :contentReference[oaicite:7]{index=7}
    return frame(addr, 0x3A)

with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
    ser.reset_input_buffer()
    found = []

    for addr in ADDR_RANGE:
        ser.reset_input_buffer()
        tx = read_enable_cmd(addr)
        ser.write(tx); ser.flush()
        time.sleep(0.01)
        rx = ser.read(16)

        # Respuesta esperada mínima: FB addr 3A status CRC
        if len(rx) >= 5 and rx[0] == 0xFB and rx[1] == addr and rx[2] == 0x3A:
            status = rx[3]
            found.append((addr, status, rx[:5]))
            print(f"ADDR {addr:02d} -> enable_status={status}  RX={hexd(rx[:5])}")

    if not found:
        print("No he encontrado ningún dispositivo respondiendo a 3A.")
    else:
        print("\nEncontrados:", [a for a,_,_ in found])
