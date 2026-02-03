#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial

# ================= CONFIG =================
PORT = "/dev/ttyUSB0"
BAUD = 38400
ADDR = 0x01

RPM_RUN = 10        # 0..3000
ACC_RUN = 2         # 0..255

RUN_TIME_S = 1.0

# Ventanas para intentar capturar RX (no crítico)
SER_TIMEOUT = 0.05
T_PRE_READ  = 0.03
T_WINDOW    = 0.25
# =========================================


def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF


def frame(addr: int, code: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + payload
    return base + bytes([checksum8(base)])


def build_f6_speed(addr: int, rpm: int, acc: int, cw: bool = True) -> bytes:
    """
    Speed mode (F6): word de 16 bits con bit15=dir, bits0..14=rpm.
    Manual: speed 0..3000 rpm, acc 0..255; stop => speed=0. :contentReference[oaicite:4]{index=4}
    """
    if not (0 <= rpm <= 3000):
        raise ValueError("RPM fuera de rango (0..3000)")
    if not (0 <= acc <= 255):
        raise ValueError("ACC fuera de rango (0..255)")

    word = rpm & 0x7FFF
    if not cw:
        word |= 0x8000

    b4 = (word >> 8) & 0xFF
    b5 = word & 0xFF
    return frame(addr, 0xF6, bytes([b4, b5, acc & 0xFF]))


def read_window(ser: serial.Serial, t_window: float) -> bytes:
    t0 = time.time()
    buf = b""
    while (time.time() - t0) < t_window:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        time.sleep(0.005)
    return buf


def txrx(ser: serial.Serial, tx: bytes, tag: str):
    ser.write(tx)
    ser.flush()
    print(f"TX {tag}: {tx.hex(' ')}")

    # Intento de lectura de respuesta (no bloqueante)
    time.sleep(T_PRE_READ)
    raw = read_window(ser, T_WINDOW)
    if raw:
        print(f"RX {tag}: {raw.hex(' ')}")
    else:
        print(f"RX {tag}: <sin respuesta>")
    return raw


def restore_respond_active_default(ser: serial.Serial):
    """
    Si previamente te quedaste sin respuesta por 8C, esta es la trama
    para volver al default XX=01 YY=01:
      FA 01 8C 01 01 89  (para addr=1)
    Manual 5.2.12 :contentReference[oaicite:5]{index=5}
    """
    tx = frame(ADDR, 0x8C, bytes([0x01, 0x01]))
    txrx(ser, tx, "restore_8C_resp1_act1")


def main():
    with serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Si sospechas que el motor está en no-response por 8C, descomenta:
        # restore_respond_active_default(ser)
        # time.sleep(0.1)

        # 1) Speed mode: SR_vFOC (82 05) :contentReference[oaicite:6]{index=6}
        txrx(ser, frame(ADDR, 0x82, bytes([0x05])), "mode_82_sr_vfoc")

        # 2) Enable: F3 01 :contentReference[oaicite:7]{index=7}
        txrx(ser, frame(ADDR, 0xF3, bytes([0x01])), "enable_F3_01")

        # 3) Run 10 rpm: F6 speed=10 acc=2 :contentReference[oaicite:8]{index=8}
        txrx(ser, build_f6_speed(ADDR, RPM_RUN, ACC_RUN, cw=True), "run_F6_10rpm")

        time.sleep(RUN_TIME_S)

        # 4) STOP robusto:
        #    a) stop inmediato (acc=0) 2 veces
        #    b) stop suave (acc=ACC_RUN) 2 veces
        #    c) emergency stop F7 como "último recurso" 
        for i in range(2):
            txrx(ser, build_f6_speed(ADDR, 0, 0, cw=True), f"stop_F6_acc0_{i}")
            time.sleep(0.05)

        for i in range(2):
            txrx(ser, build_f6_speed(ADDR, 0, ACC_RUN, cw=True), f"stop_F6_accN_{i}")
            time.sleep(0.05)

        txrx(ser, frame(ADDR, 0xF7, b""), "estop_F7")

        time.sleep(0.2)

        # 5) Disable (loose shaft): F3 00 (esto NO es stop; es “soltar”) :contentReference[oaicite:10]{index=10}
        txrx(ser, frame(ADDR, 0xF3, bytes([0x00])), "disable_F3_00")


if __name__ == "__main__":
    main()
