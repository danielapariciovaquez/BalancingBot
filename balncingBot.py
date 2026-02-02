import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
ADDR = 0x01
ACC = 2

def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF

def frame(payload: bytes) -> bytes:
    return payload + bytes([checksum8(payload)])

def hx(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def send(ser: serial.Serial, frm: bytes, label: str):
    # limpia RX para no confundirnos
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    print(f"\n{label}")
    print("TX:", hx(frm))

    n = ser.write(frm)
    print("write() bytes:", n, "/", len(frm))

    # bytes pendientes de salir por el driver (si pyserial lo soporta)
    try:
        print("out_waiting (antes flush):", ser.out_waiting)
    except Exception:
        pass

    ser.flush()  # espera a que el SO vacíe su buffer hacia el USB

    try:
        print("out_waiting (después flush):", ser.out_waiting)
    except Exception:
        pass

    # intenta leer una respuesta corta (si el driver responde)
    rx = ser.read(32)
    print("RX:", hx(rx) if rx else "(vacío)")

def payload_enable() -> bytes:
    return bytes([0xFA, ADDR, 0xF3, 0x01])

def payload_f6(rpm: int) -> bytes:
    rpm = int(rpm)
    if rpm < 0:
        direction = 1
        speed = -rpm
    else:
        direction = 0
        speed = rpm

    speed = max(0, min(3000, speed))
    b4 = ((direction & 1) << 7) | ((speed >> 8) & 0x0F)
    b5 = speed & 0xFF
    return bytes([0xFA, ADDR, 0xF6, b4, b5, ACC])

def main():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
        write_timeout=0.5,
    )

    try:
        send(ser, frame(payload_enable()), "1) ENABLE (F3=1)")
        time.sleep(0.2)

        send(ser, frame(payload_f6(10)), "2) F6 = 10 rpm")
        print("\nEsperando 5s...")
        time.sleep(5.0)

        send(ser, frame(payload_f6(100)), "3) F6 = 100 rpm")

        # CLAVE: no cierres inmediatamente, deja tiempo a que el FTDI/RS485 conmute y se vea en bus
        print("\nEsperando 1s antes de cerrar (para evitar truncado)...")
        time.sleep(1.0)

    finally:
        ser.close()

if __name__ == "__main__":
    main()
