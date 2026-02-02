import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400

ADDR_M1 = 0x01
ADDR_M2 = 0x02

ACC = 255          # aceleración
RPM_RUN = 10    # rpm objetivo
T_RUN = 5.0      # segundos en marcha
T_STOP = 5.0     # segundos parado


# -----------------------------
# Protocolo MKS
# -----------------------------
def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF

def frame_f3_enable(addr: int, enable: bool) -> bytes:
    payload = bytes([0xFA, addr, 0xF3, 0x01 if enable else 0x00])
    return payload + bytes([checksum8(payload)])

def frame_f6_speed(addr: int, rpm: int, acc: int) -> bytes:
    if rpm < 0:
        direction = 1
        speed = -rpm
    else:
        direction = 0
        speed = rpm

    speed = max(0, min(3000, speed))

    byte4 = ((direction & 0x01) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF

    payload = bytes([0xFA, addr, 0xF6, byte4, byte5, acc & 0xFF])
    return payload + bytes([checksum8(payload)])


# -----------------------------
# Main
# -----------------------------
def main():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
        write_timeout=0.2,
    )

    def send(frame: bytes):
        ser.write(frame)
        ser.flush()

    try:
        # Enable ambos motores
        send(frame_f3_enable(ADDR_M1, True))
        send(frame_f3_enable(ADDR_M2, True))
        time.sleep(0.2)

        print("Bucle iniciado: RUN 5s / STOP 5s")

        while True:
            # --- RUN ---
            print("RUN 100 RPM")
            send(frame_f6_speed(ADDR_M1, RPM_RUN, ACC))
            send(frame_f6_speed(ADDR_M2, RPM_RUN, ACC))
            time.sleep(T_RUN)

            # --- STOP ---
            print("STOP")
            send(frame_f6_speed(ADDR_M1, 0, ACC))
            send(frame_f6_speed(ADDR_M2, 0, ACC))
            time.sleep(T_STOP)

    except KeyboardInterrupt:
        print("\nCTRL+C -> STOP y salir")
        send(frame_f6_speed(ADDR_M1, 0, ACC))
        send(frame_f6_speed(ADDR_M2, 0, ACC))
        send(frame_f3_enable(ADDR_M1, False))
        send(frame_f3_enable(ADDR_M2, False))

    finally:
        ser.close()


if __name__ == "__main__":
    main()
