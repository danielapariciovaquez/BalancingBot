import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400

ADDRS = [0x01, 0x02]   # motor 1 y motor 2
ACC = 2

RPM_RUN = 100
T_RUN = 5.0
T_STOP = 5.0

INTER_FRAME_DELAY = 0.02   # 20 ms entre tramas (clave)
STOP_RETRIES = 3           # repetir STOP para asegurar


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


def send(ser: serial.Serial, frame: bytes, label: str = ""):
    # drenar RX por si el adaptador acumula basura/eco (no dependemos de RX)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    ser.write(frame)
    ser.flush()

    if label:
        print(f"{label} TX: {hx(frame)}")
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

    try:
        # Enable motores
        for a in ADDRS:
            send(ser, frame_f3_enable(a, True), label=f"EN[{a:02X}]")
        time.sleep(0.2)

        print("Bucle: RUN 5s / STOP 5s (indefinido)")

        while True:
            # RUN
            print("== RUN ==")
            for a in ADDRS:
                send(ser, frame_f6_speed(a, RPM_RUN, ACC), label=f"F6[{a:02X}]")
            time.sleep(T_RUN)

            # STOP (repetido)
            print("== STOP ==")
            for _ in range(STOP_RETRIES):
                for a in ADDRS:
                    send(ser, frame_f6_speed(a, 0, ACC), label=f"F6STOP[{a:02X}]")
            time.sleep(T_STOP)

    except KeyboardInterrupt:
        print("\nCTRL+C -> STOP y salir")
        for _ in range(STOP_RETRIES):
            for a in ADDRS:
                send(ser, frame_f6_speed(a, 0, ACC), label=f"F6STOP[{a:02X}]")
        for a in ADDRS:
            send(ser, frame_f3_enable(a, False), label=f"DIS[{a:02X}]")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
