#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial

# ================= CONFIG =================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT = 0.2

ADDR = 0x01

RPM_RUN = 10        # 0..3000
ACC_RUN = 0x02      # 0..255 (aceleración para run/stop suave)
WAIT_RUN_S = 1.0    # tiempo girando antes de parar

# Robustez RS485 (auto-direction): retardo antes de leer + ventana de captura
T_PRE_READ = 0.03   # 30 ms
T_WINDOW = 1.0      # 1 s
# ==========================================


def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF


def frame(addr: int, code: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + payload
    return base + bytes([checksum8(base)])


def read_window(ser: serial.Serial, t_window: float = 1.0) -> bytes:
    """Lee todo lo que llegue durante una ventana temporal."""
    t0 = time.time()
    buf = b""
    while (time.time() - t0) < t_window:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        time.sleep(0.01)
    return buf


def txrx(ser: serial.Serial, tx: bytes, tag: str,
         t_pre: float = T_PRE_READ, t_window: float = T_WINDOW,
         clear_rx: bool = True) -> bytes:
    """Envía una trama y captura cualquier respuesta durante una ventana."""
    if clear_rx:
        ser.reset_input_buffer()

    ser.write(tx)
    ser.flush()
    print(f"TX {tag}: {tx.hex(' ')}")

    # Deja tiempo a que el adaptador pase a RX + al driver a responder
    time.sleep(t_pre)

    rx = read_window(ser, t_window=t_window)
    if rx:
        print(f"RX {tag}: {rx.hex(' ')}")
    else:
        print(f"RX {tag}: <sin respuesta>")

    return rx


def main():
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 2) ENABLE (shaft lock): 0xF3 0x01
        txrx(ser, frame(ADDR, 0xF3, bytes([0x01])), "enable_F3_01")

        # 3) RUN 10 rpm (F6): speed_word=0x000A => [0x00,0x0A], acc=ACC_RUN
        txrx(ser, frame(ADDR, 0xF6, bytes([0x00, 100 & 0xFF, ACC_RUN])), "run_F6_10rpm")

        time.sleep(WAIT_RUN_S)

        txrx(ser, frame(ADDR, 0xF6, bytes([0x00, 10 & 0xFF, ACC_RUN])), "run_F6_10rpm")


        time.sleep(WAIT_RUN_S)

        # 4) STOP inmediato (F6 speed=0 acc=0)
        txrx(ser, frame(ADDR, 0xF6, bytes([0x00, 0x00, 0x00])), "stop_F6_acc0")

        # 5) STOP suave (opcional) (F6 speed=0 acc=ACC_RUN)
        #    Si no lo quieres, comenta estas dos líneas.
        txrx(ser, frame(ADDR, 0xF6, bytes([0x00, 0x00, ACC_RUN])), "stop_F6_accN")

        # 7) DISABLE (loose shaft): 0xF3 0x00
        txrx(ser, frame(ADDR, 0xF3, bytes([0x00])), "disable_F3_00")


if __name__ == "__main__":
    main()
