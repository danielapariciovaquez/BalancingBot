#!/usr/bin/env python3
import time
import serial
import pygame
import sys
from dataclasses import dataclass

# ===================== CONFIG =====================
PORT = "/dev/ttyUSB0"
BAUD = 38400
TIMEOUT_S = 0.05

ADDR_LEFT  = 0x01
ADDR_RIGHT = 0x02

# MKS: speed range 0..3000 RPM (12 bits)
MAX_RPM = 200

# Aceleración (0..255). OJO: en el manual, acc=0 implica parada inmediata cuando speed=0.
ACC = 255

# Control joystick
DEADZONE = 0.008
UPDATE_HZ = 50

# Ejes típicos en mandos tipo Xbox:
#   Left stick Y: eje 1 (arriba negativo en pygame normalmente)
#   Right stick X: eje 3 (depende del driver)
AXIS_THROTTLE = 1
AXIS_TURN     = 3

# Si un motor está montado invertido, cambia aquí (muy típico en diferencial)
INVERT_LEFT  = False
INVERT_RIGHT = True
# ==================================================

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def dz(x: float, dead: float) -> float:
    return 0.0 if abs(x) < dead else x

def checksum8(data: bytes) -> int:
    return sum(data) & 0xFF  # manual: CHECKSUM 8bit

def frame(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    base = bytes([0xFA, addr & 0xFF, cmd & 0xFF]) + payload
    return base + bytes([checksum8(base)])

def cmd_enable(addr: int, en: bool) -> bytes:
    # FA addr F3 enable CRC
    return frame(addr, 0xF3, bytes([0x01 if en else 0x00]))

def cmd_speed(addr: int, rpm: int, acc: int) -> bytes:
    """
    F6 speed mode:
      Byte4: bit7 = dir, bits[3:0] = speed[11:8]
      Byte5: speed[7:0]
      Byte6: acc (0..255)
    speed range 0..3000 RPM
    dir: 0/1 según manual (bit7). Aquí: rpm>=0 => dir=0, rpm<0 => dir=1
    """
    acc_u8 = int(clamp(acc, 0, 255))
    if rpm >= 0:
        direction_bit = 0
        speed = rpm
    else:
        direction_bit = 1
        speed = -rpm

    speed = int(clamp(speed, 0, MAX_RPM))
    b4 = ((direction_bit & 0x01) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    payload = bytes([b4, b5, acc_u8])
    return frame(addr, 0xF6, payload)

@dataclass
class MotorCmd:
    left_rpm: int
    right_rpm: int

def mix_differential(throttle: float, turn: float) -> MotorCmd:
    """
    Mezcla diferencial estándar:
      left  = throttle - turn
      right = throttle + turn
    throttle, turn en [-1..1]
    """
    left  = throttle - turn
    right = throttle + turn

    # Normaliza si saturan (mantiene proporción)
    m = max(1.0, abs(left), abs(right))
    left /= m
    right /= m

    left_rpm = int(round(left * MAX_RPM))
    right_rpm = int(round(right * MAX_RPM))

    if INVERT_LEFT:
        left_rpm = -left_rpm
    if INVERT_RIGHT:
        right_rpm = -right_rpm

    return MotorCmd(left_rpm=left_rpm, right_rpm=right_rpm)

def open_serial() -> serial.Serial:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT_S,
        write_timeout=TIMEOUT_S,
    )
    # Limpieza buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def send(ser: serial.Serial, data: bytes) -> None:
    # Importante: bytes continuos sin pausas inter-byte (manual).
    ser.write(data)
    ser.flush()

def main() -> int:
    ser = open_serial()

    # --- init pygame / joystick ---
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("ERROR: no hay joystick detectado por pygame.")
        print("Tip: prueba `jstest /dev/input/js0` o revisa permisos /dev/input/*")
        ser.close()
        return 2

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Joystick: {joy.get_name()} | axes={joy.get_numaxes()} buttons={joy.get_numbuttons()}")

    # --- ENABLE motores ---
    send(ser, cmd_enable(ADDR_LEFT, True))
    send(ser, cmd_enable(ADDR_RIGHT, True))
    time.sleep(0.05)

    period = 1.0 / UPDATE_HZ
    t_next = time.monotonic()

    try:
        while True:
            # Bombea eventos (imprescindible en pygame)
            pygame.event.pump()

            # Lee ejes
            thr = -joy.get_axis(AXIS_THROTTLE)  # invert típico: arriba suele ser negativo
            trn = joy.get_axis(AXIS_TURN)/4

            thr = dz(thr, DEADZONE)
            trn = dz(trn, DEADZONE)

            thr = clamp(thr, -1.0, 1.0)
            trn = clamp(trn, -1.0, 1.0)

            mc = mix_differential(thr, trn)

            # Manda F6 a ambos motores
            send(ser, cmd_speed(ADDR_LEFT, mc.left_rpm, ACC))
            send(ser, cmd_speed(ADDR_RIGHT, mc.right_rpm, ACC))

            # loop timing
            t_next += period
            now = time.monotonic()
            sleep_s = t_next - now
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # vamos tarde; re-sincroniza
                t_next = now

    except KeyboardInterrupt:
        print("\nCTRL+C -> Parando motores...")
    finally:
        # 0 RPM con acc=0 (parada inmediata) y luego disable
        try:
            send(ser, cmd_speed(ADDR_LEFT, 0, 0))
            send(ser, cmd_speed(ADDR_RIGHT, 0, 0))
            time.sleep(0.05)
            send(ser, cmd_enable(ADDR_LEFT, False))
            send(ser, cmd_enable(ADDR_RIGHT, False))
        except Exception as e:
            print(f"WARNING: error al parar/disable: {e}", file=sys.stderr)

        try:
            ser.close()
        except Exception:
            pass
        pygame.quit()

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
