import serial, time

PORT="/dev/ttyUSB0"
BAUD=38400
ADDR=0x01
ACC=2
INTER=0.03

def checksum8(p): return sum(p) & 0xFF

def send(ser, payload: bytes, rx_len=32):
    frm = payload + bytes([checksum8(payload)])
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    ser.write(frm)
    ser.flush()
    time.sleep(INTER)
    rx = ser.read(rx_len)
    print("TX:", " ".join(f"{b:02X}" for b in frm))
    print("RX:", " ".join(f"{b:02X}" for b in rx) if rx else "(vacio)")
    return rx

def f3(enable: bool):
    return bytes([0xFA, ADDR, 0xF3, 0x01 if enable else 0x00])

def f6(rpm: int):
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

ser = serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=0.2)

try:
    while True:
        print("\n== ENABLE + RUN ==")
        send(ser, f3(True))
        time.sleep(0.1)
        send(ser, f6(100))

        time.sleep(2)

        print("\n== STOP ==")
        send(ser, f6(0))

        time.sleep(2)

        print("\n== RE-ENABLE + RUN ==")
        send(ser, f3(True))
        time.sleep(0.1)
        send(ser, f6(100))

        time.sleep(2)

except KeyboardInterrupt:
    print("\n== SALIR: STOP + DISABLE ==")
    send(ser, f6(0))
    send(ser, f3(False))
finally:
    ser.close()
