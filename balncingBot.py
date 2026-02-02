import time
import serial

try:
    from serial.rs485 import RS485Settings
except Exception:
    RS485Settings = None  # si no está disponible, seguiremos sin ello

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400

ADDRS = [0x01, 0x02]
ACC = 2

RPM_RUN = 100
T_RUN = 5.0
T_STOP = 5.0

INTER_FRAME_DELAY = 0.08   # 80 ms (conservador)
STOP_RETRIES = 3


def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF


def frame_f3_enable(addr: int, enable: bool) -> bytes:
    payload = bytes([0xFA, addr & 0xFF, 0xF3, 0x01 if enable else 0x00])
    return payload + bytes([checksum8(payload)])


def frame_f6_speed(addr: int, rpm: int, acc: int) -> bytes:
    rpm = int(rpm)
    acc = int(acc) & 0xFF

    if rpm < 0:
        direction = 1
        speed = -rpm
    else:
        direction = 0
        speed = rpm

    speed = max(0, min(3000, int(speed)))

    byte4 = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF

    payload = bytes([0xFA, addr & 0xFF, 0xF6, byte4, byte5, acc])
    return payload + bytes([checksum8(payload)])


def hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def send(ser: serial.Serial, frame: bytes, tag: str = ""):
    # Drenar RX por si hay eco/basura
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    ser.write(frame)
    ser.flush()

    if tag:
        print(f"{tag} TX: {hx(frame)}")
    else:
        print(f"TX: {hx(frame)}")

    time.sleep(INTER_FRAME_DELAY)


def main():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        write_timeout=0.2,
    )

    # Fuerza modo RS485 con RTS->DE si pyserial lo soporta
    # (esto soluciona muchos “solo llega la primera trama”)
    if RS485Settings is not None:
        try:
            ser.rs485_mode = RS485Settings(
                rts_level_for_tx=True,
                rts_level_for_rx=False,
                delay_before_tx=0.001,
                delay_before_rx=0.001,
            )
            print("[INFO] RS485Settings activado (RTS controla DE).")
        except Exception as e:
            print(f"[WARN] No se pudo activar RS485Settings: {e}")

    try:
        print("[INFO] ENABLE motores")
        for a in ADDRS:
            send(ser, frame_f3_enable(a, True), tag=f"EN[{a:02X}]")
        time.sleep(0.2)

        print("[INFO] Bucle: RUN 5s / STOP+DISABLE 5s")

        while True:
            # RUN
            print("== RUN ==")
            for a in ADDRS:
                send(ser, frame_f6_speed(a, RPM_RUN, ACC), tag=f"F6[{a:02X}]")
            time.sleep(T_RUN)

            # STOP + DISABLE
            print("== STOP ==")
            for _ in range(STOP_RETRIES):
                for a in ADDRS:
                    send(ser, frame_f6_speed(a, 0, ACC), tag=f"F6STOP[{a:02X}]")

            # Corta el driver (más contundente que 0 RPM)
            for a in ADDRS:
                send(ser, frame_f3_enable(a, False), tag=f"DIS[{a:02X}]")

            time.sleep(T_STOP)

            # Re-enable para el siguiente RUN
            for a in ADDRS:
                send(ser, frame_f3_enable(a, True), tag=f"EN[{a:02X}]")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] CTRL+C -> STOP y salir")
        for _ in range(STOP_RETRIES):
            for a in ADDRS:
                send(ser, frame_f6_speed(a, 0, ACC), tag=f"F6STOP[{a:02X}]")
        for a in ADDRS:
            send(ser, frame_f3_enable(a, False), tag=f"DIS[{a:02X}]")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
