import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01
ACC = 2


def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF


def send(ser: serial.Serial, payload: bytes):
    frame = payload + bytes([checksum8(payload)])
    ser.write(frame)
    ser.flush()
    print("TX:", " ".join(f"{b:02X}" for b in frame))
    time.sleep(0.05)   # pausa conservadora entre tramas


def frame_enable(addr: int) -> bytes:
    # FA addr F3 01
    return bytes([0xFA, addr, 0xF3, 0x01])


def frame_f6_speed(addr: int, rpm: int, acc: int) -> bytes:
    if rpm < 0:
        direction = 1
        speed = -rpm
    else:
        direction = 0
        speed = rpm

    speed = max(0, min(3000, int(speed)))

    byte4 = ((direction & 1) << 7) | ((speed >> 8) & 0x0F)
    byte5 = speed & 0xFF

    # FA addr F6 byte4 byte5 acc
    return bytes([0xFA, addr, 0xF6, byte4, byte5, acc & 0xFF])


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

    try:
        # ENABLE
        send(ser, frame_enable(ADDR))

        # 10 RPM
        send(ser, frame_f6_speed(ADDR, 10, ACC))

        # Espera 5 segundos
        time.sleep(5.0)

        # 100 RPM
        send(ser, frame_f6_speed(ADDR, 100, ACC))

    finally:
        ser.close()


if __name__ == "__main__":
    main()
