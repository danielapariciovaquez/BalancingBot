#!/usr/bin/env python3
import time
import serial

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG01MQCG-if00-port0"
BAUD = 38400

ADDR1 = 0x01
ADDR2 = 0x02
RPM = 100
ACC = 10

def checksum8(b: bytes) -> int:
    return sum(b) & 0xFF

def frame(addr: int, cmd: int, payload: bytes=b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_set_mode(addr: int) -> bytes:
    return frame(addr, 0x82, bytes([0x05]))

def cmd_enable(addr: int, en: bool) -> bytes:
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_speed(addr: int, rpm_signed: int, acc: int) -> bytes:
    rpm = int(rpm_signed)
    dir_bit = 1 if rpm < 0 else 0
    speed = abs(rpm)
    if speed > 3000:
        speed = 3000
    acc = max(0, min(255, int(acc)))
    b4 = ((dir_bit & 1) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return frame(addr, 0xF6, bytes([b4, b5, acc]))

def stop(addr: int) -> bytes:
    return cmd_speed(addr, 0, ACC)

def tx(ser: serial.Serial, b: bytes, tag: str):
    n = ser.write(b)
    ser.flush()
    print(f"{tag}: wrote {n} bytes (len={len(b)})")
    return n

def main():
    print(f"Abriendo {PORT} @ {BAUD} ...")
    with serial.Serial(PORT, BAUD, timeout=0, write_timeout=0.5) as ser:
        # Init
        tx(ser, cmd_set_mode(ADDR1), "set_mode M1"); time.sleep(0.05)
        tx(ser, cmd_set_mode(ADDR2), "set_mode M2"); time.sleep(0.05)
        tx(ser, cmd_enable(ADDR1, True), "enable  M1"); time.sleep(0.05)
        tx(ser, cmd_enable(ADDR2, True), "enable  M2"); time.sleep(0.05)

        print("Mandando 100rpm en bucle (Ctrl+C para parar). Debe parpadear LED TX.")
        try:
            while True:
                tx(ser, cmd_speed(ADDR1, RPM, ACC), "F6 M1")
                time.sleep(0.01)
                tx(ser, cmd_speed(ADDR2, RPM, ACC), "F6 M2")
                time.sleep(0.20)
        except KeyboardInterrupt:
            print("\nSTOP...")
            tx(ser, stop(ADDR1), "stop M1"); time.sleep(0.05)
            tx(ser, stop(ADDR2), "stop M2"); time.sleep(0.05)
            tx(ser, cmd_enable(ADDR1, False), "disable M1"); time.sleep(0.05)
            tx(ser, cmd_enable(ADDR2, False), "disable M2"); time.sleep(0.05)
            print("Fin.")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("EXCEPCIÓN:", repr(e))
