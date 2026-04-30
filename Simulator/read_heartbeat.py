from pymavlink import mavutil

PORT = "/dev/cu.usbmodem103"
BAUD = 115200

master = mavutil.mavlink_connection(PORT, baud=BAUD)

print("Waiting for HEARTBEAT...")

while True:
    msg = master.recv_match(blocking=True)

    if msg is None:
        continue

    print(msg)