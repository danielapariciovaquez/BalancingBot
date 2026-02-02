#!/usr/bin/env python3
import time
import serial

# ===== CONFIG =====
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT = 0.1
WRITE_TIMEOUT = 0.2

ADDR1 = 0x01
ADDR2 = 0x02

RPM = 100
ACC = 10

SEND_HZ = 10.0          # 10 Hz -> 2 tramas/iter = 20 tramas/s (muy conservador)
STATUS_EVERY_S = 1.0
INTER_FRAME_S = 0.003   # 3 ms entre tramas

# ==================

def checksum8(b: bytes) -> int:
    return sum(b) & 0xFF

def frame(addr: int, cmd: int, payload: bytes=b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_set_mode(addr: int) -> bytes:
    return frame(addr, 0x82, bytes([0x05]))  # SR_vFOC

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

def cmd_read_enable(addr: int) -> bytes:
    return frame(addr, 0x3A)

def cmd_read_oper_status(addr: int) -> bytes:
    return frame(addr, 0xF1)

def hexd(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def read_some(ser: serial.Serial, n=32) -> bytes:
    return ser.read(n)

def tx(ser: serial.Serial, b: bytes) -> int:
    # Devuelve bytes escritos; si no escribe todo, algo va mal
    return ser.write(b)

def main():
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT, write_timeout=WRITE_TIMEOUT) as ser:
        ser.reset_input_buffer()

        # Init
        for a in (ADDR1, ADDR2):
            n = tx(ser, cmd_set_mode(a)); ser.flush()
            if n != 5:
                print(f"[WARN] write set_mode addr={a} wrote {n} bytes")
            time.sleep(INTER_FRAME_S)

        for a in (ADDR1, ADDR2):
            n = tx(ser, cmd_enable(a, True)); ser.flush()
            if n != 5:
                print(f"[WARN] write enable addr={a} wrote {n} bytes")
            time.sleep(INTER_FRAME_S)

        print("[RUN] mandando 100 rpm a ambos a 10 Hz. Ctrl+C para salir.")
        dt = 1.0 / SEND_HZ
        next_t = time.monotonic()
        next_status = time.monotonic() + STATUS_EVERY_S

        try:
            while True:
                now = time.monotonic()
                if now < next_t:
                    time.sleep(next_t - now)
                next_t += dt

                # mandar velocidad
                n1 = tx(ser, cmd_speed(ADDR1, RPM, ACC)); ser.flush()
                time.sleep(INTER_FRAME_S)
                n2 = tx(ser, cmd_speed(ADDR2, RPM, ACC)); ser.flush()
                time.sleep(INTER_FRAME_S)

                if (n1 != 7) or (n2 != 7):
                    print(f"\n[WARN] write lengths: m1={n1}, m2={n2} (esperado 7)")

                # leer estados cada 1s
                if time.monotonic() >= next_status:
                    next_status += STATUS_EVERY_S
                    print("\n[STATUS]")

                    for a in (ADDR1, ADDR2):
                        ser.reset_input_buffer()

                        tx(ser, cmd_read_enable(a)); ser.flush()
                        time.sleep(0.02)
                        rx = read_some(ser, 16)
                        print(f" addr={a} 3A rx: {hexd(rx) if rx else '(sin respuesta)'}")

                        ser.reset_input_buffer()
                        tx(ser, cmd_read_oper_status(a)); ser.flush()
                        time.sleep(0.02)
                        rx = read_some(ser, 16)
                        print(f" addr={a} F1 rx: {hexd(rx) if rx else '(sin respuesta)'}")

        except KeyboardInterrupt:
            print("\n[STOP] stop & disable")
            tx(ser, cmd_speed(ADDR1, 0, ACC)); ser.flush(); time.sleep(INTER_FRAME_S)
            tx(ser, cmd_speed(ADDR2, 0, ACC)); ser.flush(); time.sleep(INTER_FRAME_S)
            tx(ser, cmd_enable(ADDR1, False)); ser.flush(); time.sleep(INTER_FRAME_S)
            tx(ser, cmd_enable(ADDR2, False)); ser.flush(); time.sleep(INTER_FRAME_S)

if __name__ == "__main__":
    main()
