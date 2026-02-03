#!/usr/bin/env python3
import serial
import time

# ================= CONFIG =================
PORT = "/dev/ttyUSB0"   # tu Waveshare USB-RS485 suele enumerar aquí; ajusta si es ttyUSB1, etc.
BAUD = 38400            # típico en estos drivers (ajusta si lo cambiaste en el menú UartBaud)
ADDR = 0x01             # dirección del motor
ACC = 0x02              # aceleración (0..255)
RPM = 10                # velocidad objetivo (0..3000)
DIR_CW = False          # False => bit7=0 ; True => bit7=1 (según manual, bit alto indica dirección)
WAIT_RUN_S = 2.0        # tiempo que lo dejas girando antes de deshabilitar
TIMEOUT = 0.2
# =========================================

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF

def frame(addr: int, code: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_set_mode_speed(addr: int) -> bytes:
    # 82H set work mode; 05H = SR_vFOC (ejemplo del manual)
    return frame(addr, 0x82, bytes([0x05]))

def cmd_enable(addr: int, enable: bool) -> bytes:
    # F3H: enable=01 (lock), enable=00 (loose)
    return frame(addr, 0xF3, bytes([0x01 if enable else 0x00]))

def cmd_speed(addr: int, rpm: int, acc: int, dir_cw: bool) -> bytes:
    # F6H: Byte4 Byte5 codifican la velocidad como en el ejemplo 300RPM -> 0x012C -> [0x01,0x2C]
    # Bit7 de Byte4 marca dirección (manual). Aquí construimos un "word" de 16 bits:
    #   speed_word = rpm (0..3000) con bit15 = dir
    if not (0 <= rpm <= 3000):
        raise ValueError("RPM fuera de rango (0..3000)")
    if not (0 <= acc <= 255):
        raise ValueError("ACC fuera de rango (0..255)")

    speed_word = rpm & 0x7FFF
    if dir_cw:
        speed_word |= 0x8000

    b4 = (speed_word >> 8) & 0xFF
    b5 = speed_word & 0xFF
    return frame(addr, 0xF6, bytes([b4, b5, acc & 0xFF]))

def read_reply(ser: serial.Serial) -> bytes | None:
    # Uplink típico: FB addr code ... CRC (longitud variable).
    # Para este test mínimo, intentamos leer lo que haya en el buffer tras cada envío.
    time.sleep(0.05)
    n = ser.in_waiting
    if n:
        return ser.read(n)
    return None

def main():
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
        # Limpia cualquier basura previa
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 1) Set work mode = SR_vFOC (speed mode example)
        tx = cmd_set_mode_speed(ADDR)
        ser.write(tx)
        ser.flush()
        print("TX set_mode:", tx.hex(" "))
        rx = read_reply(ser)
        if rx:
            print("RX:", rx.hex(" "))

        # 2) ENABLE
        tx = cmd_enable(ADDR, True)
        ser.write(tx)
        ser.flush()
        print("TX enable:", tx.hex(" "))
        rx = read_reply(ser)
        if rx:
            print("RX:", rx.hex(" "))

        # 3) Speed = 10 RPM
        tx = cmd_speed(ADDR, RPM, ACC, DIR_CW)
        ser.write(tx)
        ser.flush()
        print("TX speed:", tx.hex(" "))
        rx = read_reply(ser)
        if rx:
            print("RX:", rx.hex(" "))

        # 4) Dejar girar
        time.sleep(WAIT_RUN_S)

        # 5) DISABLE
        tx = cmd_enable(ADDR, False)
        ser.write(tx)
        ser.flush()
        print("TX disable:", tx.hex(" "))
        rx = read_reply(ser)
        if rx:
            print("RX:", rx.hex(" "))

if __name__ == "__main__":
    main()
