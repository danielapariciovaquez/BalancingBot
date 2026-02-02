#!/usr/bin/env python3
import time
import serial

# =========================
# CONFIG
# =========================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.2

ADDRS = [0x01, 0x02]  # tus dos motores

# Prueba de movimiento
TEST_RPM = 300
TEST_ACC = 2
RUN_SECONDS = 2.0

# Si sospechas que están en MODBUS-RTU, deja esto en True para intentar desactivarlo
TRY_DISABLE_MODBUS_RTU = True

# =========================
# Protocolo MKS (manual)
# Downlink: FA addr code ... CRC, CRC = sum(bytes) & 0xFF
# =========================
def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF

def mks_frame(addr: int, code: int, data: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum8(base)])

def hexd(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def read_some(ser: serial.Serial, max_bytes: int = 64) -> bytes:
    # Lee lo que haya en el buffer dentro del timeout
    return ser.read(max_bytes)

def flush_in(ser: serial.Serial):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

# ---- Comandos relevantes del manual ----
# 82H: set work mode, 05H = SR_vFOC :contentReference[oaicite:6]{index=6}
def cmd_set_mode_srvfoc(addr: int) -> bytes:
    return mks_frame(addr, 0x82, bytes([0x05]))

# F3H: enable=00 loose, enable=01 shaft lock :contentReference[oaicite:7]{index=7}
def cmd_enable(addr: int, en: bool) -> bytes:
    return mks_frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

# 3AH: read enable status (FB addr 3A status CRC) :contentReference[oaicite:8]{index=8}
def cmd_read_enable(addr: int) -> bytes:
    return mks_frame(addr, 0x3A)

# F6H: speed mode.
# Manual example for forward 300rpm acc=2:
# FA 01 F6 01 2C 02 20 
def cmd_speed(addr: int, rpm: int, acc: int) -> bytes:
    # rpm signed -> dir bit
    dir_bit = 1 if rpm < 0 else 0
    speed = abs(int(rpm))
    if speed > 3000:
        speed = 3000
    acc = max(0, min(255, int(acc)))
    b4 = ((dir_bit & 1) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return mks_frame(addr, 0xF6, bytes([b4, b5, acc]))

def cmd_stop(addr: int, acc: int) -> bytes:
    return cmd_speed(addr, 0, acc)

# MODBUS disable frame from manual (to switch back to MKS protocol):
# 01 06 00 8E 00 00 E9 E1 :contentReference[oaicite:10]{index=10}
MODBUS_DISABLE = bytes([0x01, 0x06, 0x00, 0x8E, 0x00, 0x00, 0xE9, 0xE1])

def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_S)
    print(f"[SER] {PORT} @ {BAUD} 8N1 timeout={TIMEOUT_S}s")

    try:
        flush_in(ser)

        if TRY_DISABLE_MODBUS_RTU:
            print("\n[STEP] Intento desactivar MODBUS-RTU (si estuviera activo)...")
            print(" TX(MODBUS):", hexd(MODBUS_DISABLE))
            ser.write(MODBUS_DISABLE)
            ser.flush()
            time.sleep(0.2)
            rx = read_some(ser, 64)
            if rx:
                print(" RX:", hexd(rx))
            else:
                print(" RX: (sin respuesta)")

        for addr in ADDRS:
            print(f"\n===== MOTOR addr=0x{addr:02X} =====")

            # 1) Set work mode SR_vFOC (82 05)
            tx = cmd_set_mode_srvfoc(addr)
            flush_in(ser)
            print(" TX(82 mode=05):", hexd(tx))
            ser.write(tx); ser.flush()
            time.sleep(0.1)
            rx = read_some(ser, 64)
            print(" RX:", hexd(rx) if rx else "(sin respuesta)")

            # 2) Enable ON (F3 01)
            tx = cmd_enable(addr, True)
            flush_in(ser)
            print(" TX(F3 enable=01):", hexd(tx))
            ser.write(tx); ser.flush()
            time.sleep(0.1)
            rx = read_some(ser, 64)
            print(" RX:", hexd(rx) if rx else "(sin respuesta)")

            # 3) Read enable status (3A)
            tx = cmd_read_enable(addr)
            flush_in(ser)
            print(" TX(3A read enable):", hexd(tx))
            ser.write(tx); ser.flush()
            time.sleep(0.1)
            rx = read_some(ser, 64)
            print(" RX:", hexd(rx) if rx else "(sin respuesta)")

            # 4) Speed run
            tx = cmd_speed(addr, TEST_RPM, TEST_ACC)
            print(f" TX(F6 speed={TEST_RPM} acc={TEST_ACC}):", hexd(tx))
            ser.write(tx); ser.flush()
            time.sleep(RUN_SECONDS)

            # 5) Stop
            tx = cmd_stop(addr, TEST_ACC)
            print(" TX(F6 stop):", hexd(tx))
            ser.write(tx); ser.flush()
            time.sleep(0.2)

        print("\n[OK] Fin de diagnóstico.")
        print("Si NO hay movimiento y NO hay respuestas FB..., el problema está en la capa RS485/protocolo (puerto/baud/A-B/MODBUS).")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
