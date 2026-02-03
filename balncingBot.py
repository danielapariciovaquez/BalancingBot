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

WAIT_AFTER_RUN = 0.5
TIMEOUT_SERIAL = 0.05     # timeout corto; nosotros hacemos “polling” por ventana
T_PRE_READ = 0.03         # 30 ms para que el adaptador auto-direction pase a RX
T_WINDOW = 0.5            # ventana de captura de respuestas tras cada TX
# =========================================


def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF


def frame(addr: int, code: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + payload
    return base + bytes([checksum8(base)])


def build_f6_speed(addr: int, rpm: int, acc: int, cw: bool = True) -> bytes:
    """
    Speed mode (F6): Byte4..Byte5 codifican dirección + velocidad.
    Manual: bit más alto indica dirección. Rango velocidad 0..3000. :contentReference[oaicite:3]{index=3}
    Usamos un word: bit15=dir, bits0..14=rpm.
    """
    if not (0 <= rpm <= 3000):
        raise ValueError("RPM fuera de rango (0..3000)")
    if not (0 <= acc <= 255):
        raise ValueError("ACC fuera de rango (0..255)")

    word = rpm & 0x7FFF
    if not cw:
        word |= 0x8000  # si quieres invertir

    b4 = (word >> 8) & 0xFF
    b5 = word & 0xFF
    return frame(addr, 0xF6, bytes([b4, b5, acc & 0xFF]))


def read_window_raw(ser: serial.Serial, t_window: float) -> bytes:
    """Lee bytes durante una ventana de tiempo."""
    t0 = time.time()
    buf = b""
    while (time.time() - t0) < t_window:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        time.sleep(0.005)
    return buf


def extract_fb_frames(rx: bytes):
    """
    Extrae frames de uplink tipo:
      FB addr code data CRC  (5 bytes)
    Muchos ACK del manual son de 5 bytes. 
    Si llega basura o concatenación, intentamos resincronizar buscando 0xFB.
    """
    frames = []
    i = 0
    while True:
        j = rx.find(b"\xFB", i)
        if j < 0:
            break
        if j + 5 <= len(rx):
            cand = rx[j:j+5]
            if checksum8(cand[:-1]) == cand[-1]:
                frames.append(cand)
                i = j + 5
            else:
                # CRC no cuadra: avanza 1 byte para re-sincronizar
                i = j + 1
        else:
            break
    return frames


def txrx(ser: serial.Serial, tx: bytes, tag: str,
         t_pre: float = T_PRE_READ, t_window: float = T_WINDOW):
    # NO hacemos reset_input_buffer aquí a lo loco: leemos y parseamos.
    ser.write(tx)
    ser.flush()
    print(f"TX {tag}: {tx.hex(' ')}")

    time.sleep(t_pre)
    raw = read_window_raw(ser, t_window)

    if raw:
        frames = extract_fb_frames(raw)
        if frames:
            for k, fr in enumerate(frames):
                print(f"RX {tag}[{k}]: {fr.hex(' ')}")
        else:
            print(f"RX {tag}: bytes no parseados: {raw.hex(' ')}")
    else:
        print(f"RX {tag}: <sin respuesta>")

    return raw


def main():
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT_SERIAL) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 0) Configurar método de respuesta: RESPOND=1, ACTIVE=0
        # FA addr 8C XX YY CRC  (XX=01 respond, YY=00 active) :contentReference[oaicite:5]{index=5}
        txrx(ser, frame(ADDR, 0x8C, bytes([0x01, 0x00])), "cfg_8C_resp1_act0")

        # 1) Set work mode SR_vFOC (speed mode): FA 01 82 05 82 
        txrx(ser, frame(ADDR, 0x82, bytes([0x05])), "mode_82_sr_vfoc")

        # 2) ENABLE: FA 01 F3 01 EF :contentReference[oaicite:7]{index=7}
        txrx(ser, frame(ADDR, 0xF3, bytes([0x01])), "enable_F3_01")

        # 3) RUN 10 rpm (F6) :contentReference[oaicite:8]{index=8}
        txrx(ser, build_f6_speed(ADDR, RPM_RUN, ACC_RUN, cw=True), "run_F6_10rpm")

        time.sleep(WAIT_AFTER_RUN)

        # 4) STOP inmediato: F6 speed=0 acc=0  (ejemplo: FA 01 F6 00 00 00 F1) :contentReference[oaicite:9]{index=9}
        raw_stop = txrx(ser, build_f6_speed(ADDR, 0, 0, cw=True), "stop_F6_acc0")

        # 5) DISABLE: FA 01 F3 00 EE :contentReference[oaicite:10]{index=10}
        raw_dis = txrx(ser, frame(ADDR, 0xF3, bytes([0x00])), "disable_F3_00")

        # 6) Si no hay respuesta a stop o disable, lanza E-STOP (F7) como salvavidas
        # Manual: “You can also use emergency stop command F7H”. Ejemplo FA 01 F7 F2 
        if (not raw_stop) or (not raw_dis):
            txrx(ser, frame(ADDR, 0xF7, b""), "estop_F7")

if __name__ == "__main__":
    main()
