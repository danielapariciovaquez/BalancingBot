#!/usr/bin/env python3
import sys
import time
import tty
import termios
import threading
import signal
from dataclasses import dataclass

import serial

# ============================================================
# CONFIGURACIÓN (TOCA AQUÍ)
# ============================================================

RS485_PORT = "/dev/ttyUSB0"
RS485_BAUD = 38400
RS485_TIMEOUT_S = 0.10
INTER_FRAME_DELAY_S = 0.002  # pausa entre tramas

M1_ADDR = 0x01
M2_ADDR = 0x02

# Accel del driver en F6 (0..255). Para test, bajo-medio.
F6_ACC = 10

# RPM de pasos (test seguro)
RPM_STEP = 50
RPM_MAX = 400

# Si quieres que al iniciar haga enable y work mode:
DO_INIT = True
WORK_MODE = 0x05     # SR_vFOC
ENABLE_ON = True     # F3=01

# ============================================================
# Protocolo MKS RS485: FA addr code ... CRC (checksum8)
# ============================================================

def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF

def build_frame(addr: int, code: int, data: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, code & 0xFF]) + data
    return base + bytes([checksum8(base)])

def encode_f6_speed(rpm_signed: int, acc: int) -> bytes:
    rpm = int(rpm_signed)
    dir_bit = 1 if rpm < 0 else 0
    speed = abs(rpm)
    if speed > 3000:
        speed = 3000
    acc = max(0, min(255, int(acc)))
    b4 = ((dir_bit & 1) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return bytes([b4, b5, acc])

@dataclass
class State:
    m1_rpm: int = 0
    m2_rpm: int = 0
    selected: int = 0  # 0=ambos, 1=m1, 2=m2

class MksRs485:
    def __init__(self, port: str, baud: int, timeout_s: float, ifd_s: float):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_s,
        )
        self.ifd = float(ifd_s)
        self.lock = threading.Lock()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def tx(self, addr: int, code: int, data: bytes = b"", expect_reply: bool = False) -> bytes | None:
        frame = build_frame(addr, code, data)
        with self.lock:
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(self.ifd)
            if not expect_reply:
                return None
            return self.ser.read(64) or None

    def set_work_mode(self, addr: int, mode: int):
        self.tx(addr, 0x82, bytes([mode & 0xFF]), expect_reply=True)

    def enable(self, addr: int, en: bool):
        self.tx(addr, 0xF3, bytes([0x01 if en else 0x00]), expect_reply=True)

    def speed(self, addr: int, rpm: int, acc: int):
        self.tx(addr, 0xF6, encode_f6_speed(rpm, acc), expect_reply=False)

    def stop(self, addr: int, acc: int):
        self.speed(addr, 0, acc)

# ------------------------------------------------------------
# Entrada por teclado (sin Enter)
# ------------------------------------------------------------

class RawKey:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def getch(self) -> str:
        return sys.stdin.read(1)

def clamp_int(x: int, lo: int, hi: int) -> int:
    return lo if x < lo else hi if x > hi else x

def print_help():
    print(
        "\n=== MKS SERVO42D RS485 - Test de giro / signos ===\n"
        "Controles:\n"
        "  1  -> seleccionar Motor 1\n"
        "  2  -> seleccionar Motor 2\n"
        "  0  -> seleccionar Ambos\n"
        "  +  -> aumenta rpm (paso)\n"
        "  -  -> disminuye rpm (paso)\n"
        "  s  -> STOP (rpm=0) al seleccionado\n"
        "  a  -> STOP a ambos\n"
        "  r  -> invertir signo del seleccionado (multiplica por -1)\n"
        "  q  -> salir (hace STOP)\n\n"
        "Cómo usarlo:\n"
        "  - Pon el robot con ruedas al aire o sujeto.\n"
        "  - Selecciona 1, pulsa + varias veces: observa el sentido físico.\n"
        "  - Selecciona 2, pulsa + varias veces: observa el sentido físico.\n"
        "  - Ajusta con r si quieres invertir rápidamente.\n"
        "  - Objetivo típico: que con '+' (rpm positiva) ambas ruedas hagan 'avance' físico.\n"
    )

def main():
    stop_flag = {"stop": False}

    def _sig_handler(_sig, _frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    bus = MksRs485(RS485_PORT, RS485_BAUD, RS485_TIMEOUT_S, INTER_FRAME_DELAY_S)

    # Estado y signos "en vivo"
    st = State(m1_rpm=0, m2_rpm=0, selected=0)
    m1_sign = +1
    m2_sign = +1

    def apply():
        # aplica rpm actuales con signos
        bus.speed(M1_ADDR, m1_sign * st.m1_rpm, F6_ACC)
        bus.speed(M2_ADDR, m2_sign * st.m2_rpm, F6_ACC)

    def stop_all():
        st.m1_rpm = 0
        st.m2_rpm = 0
        apply()

    def stop_selected():
        if st.selected == 1:
            st.m1_rpm = 0
        elif st.selected == 2:
            st.m2_rpm = 0
        else:
            st.m1_rpm = 0
            st.m2_rpm = 0
        apply()

    try:
        if DO_INIT:
            # Modo + enable
            for addr in (M1_ADDR, M2_ADDR):
                bus.set_work_mode(addr, WORK_MODE)
            if ENABLE_ON:
                for addr in (M1_ADDR, M2_ADDR):
                    bus.enable(addr, True)

        print_help()
        print(f"[CFG] PORT={RS485_PORT} BAUD={RS485_BAUD}  M1={M1_ADDR} M2={M2_ADDR}  F6_ACC={F6_ACC}")
        print(f"[CFG] RPM_STEP={RPM_STEP} RPM_MAX={RPM_MAX}")
        print("\nEstado: (seleccion=ambos)")

        with RawKey() as rk:
            while not stop_flag["stop"]:
                # imprimir estado
                sel = "AMBOS" if st.selected == 0 else ("M1" if st.selected == 1 else "M2")
                line = (
                    f"\rSEL={sel:5s} | "
                    f"M1 rpm={st.m1_rpm:+4d} sign={m1_sign:+2d} -> cmd={m1_sign*st.m1_rpm:+4d} | "
                    f"M2 rpm={st.m2_rpm:+4d} sign={m2_sign:+2d} -> cmd={m2_sign*st.m2_rpm:+4d}   "
                )
                sys.stdout.write(line)
                sys.stdout.flush()

                ch = rk.getch()

                if ch == "q":
                    break
                elif ch == "1":
                    st.selected = 1
                elif ch == "2":
                    st.selected = 2
                elif ch == "0":
                    st.selected = 0
                elif ch == "+":
                    if st.selected in (0, 1):
                        st.m1_rpm = clamp_int(st.m1_rpm + RPM_STEP, -RPM_MAX, RPM_MAX)
                    if st.selected in (0, 2):
                        st.m2_rpm = clamp_int(st.m2_rpm + RPM_STEP, -RPM_MAX, RPM_MAX)
                    apply()
                elif ch == "-":
                    if st.selected in (0, 1):
                        st.m1_rpm = clamp_int(st.m1_rpm - RPM_STEP, -RPM_MAX, RPM_MAX)
                    if st.selected in (0, 2):
                        st.m2_rpm = clamp_int(st.m2_rpm - RPM_STEP, -RPM_MAX, RPM_MAX)
                    apply()
                elif ch == "s":
                    stop_selected()
                elif ch == "a":
                    stop_all()
                elif ch == "r":
                    # invierte signo del seleccionado
                    if st.selected == 1:
                        m1_sign *= -1
                    elif st.selected == 2:
                        m2_sign *= -1
                    else:
                        m1_sign *= -1
