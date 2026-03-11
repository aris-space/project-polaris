import serial
import time

PORT = "/dev/xsens_imu"
BAUD = 115200

GOTO_MEAS = bytes.fromhex("FA FF 10 00 F1")
SET_NO_ROT = bytes.fromhex("FA FF 22 02 00 03 DA")  # 3 s

def parse_mid(frame: bytes) -> int | None:
    # Expect at least: PRE (0) | BID (1) | MID (2) | LEN (3)
    if len(frame) < 4 or frame[0] != 0xFA:
        return None
    return frame[2]

with serial.Serial(PORT, BAUD, timeout=0.5) as ser:
    ser.reset_input_buffer()

    # 1) Try GoToMeasurement
    ser.write(GOTO_MEAS)
    ser.flush()

    ack = ser.read(5)
    print("GoToMeasurement response:", ack.hex().upper())

    mid = parse_mid(ack)
    if mid == 0x11:
        print("Info: GoToMeasurement ACK -> now in Measurement state.")
    elif mid == 0x42:
        print("Warning: IMU returned Error for GoToMeasurement (probably already in Measurement).")
    elif mid is None:
        print("Warning: No valid response to GoToMeasurement, assuming already in Measurement.")
    else:
        print(f"Warning: Unexpected MID 0x{mid:02X} after GoToMeasurement, continuing anyway.")

    # 2) Give it some time to stream data
    time.sleep(0.5)

    data = ser.read(200)
    print("Incoming data (first 200 bytes):", data.hex().upper())

    if not data:
        print("Error: No MTData2 received, aborting SetNoRotation.")
    else:
        # Optional: sanity check that we see at least one MTData2 preamble+MID (0xFA .. 0x36)
        if 0xFA in data:
            print("Info: Got some data, assuming Measurement state OK.")

        # 3) Send SetNoRotation
        ser.write(SET_NO_ROT)
        ser.flush()

        ack2 = ser.read(5)
        print("SetNoRotation response:", ack2.hex().upper())

        mid2 = parse_mid(ack2)
        if mid2 == 0x23:
            print("Info: SetNoRotation ACK -> gyro no-rotation update started (keep IMU still).")
        elif mid2 == 0x42:
            print("Error: IMU rejected SetNoRotation (Error MID 0x42).")
        elif mid2 is None:
            print("Warning: No valid response to SetNoRotation.")
        else:
            print(f"Warning: Unexpected MID 0x{mid2:02X} after SetNoRotation.")
