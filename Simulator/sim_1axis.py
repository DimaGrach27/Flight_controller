import math
import time
import serial

PORT = "/dev/cu.usbmodem103"  # заміниш на свій порт
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

angle = math.radians(20.0)       # початковий нахил 20 градусів
angular_velocity = 0.0

left_motor = 0.5
right_motor = 0.5

dt = 0.01                        # 100 Hz для старту

arm_length = 0.15
inertia = 0.02
max_thrust = 1.0
damping = 0.02

def clamp(x, a, b):
    return max(a, min(b, x))

print("Simulation started")

while True:
    # fake IMU
    gyro_x_deg = math.degrees(angular_velocity)

    accel_y = math.sin(angle) * 9.81
    accel_z = math.cos(angle) * 9.81

    # send sensors to STM32
    packet = f"S,{dt:.6f},{gyro_x_deg:.6f},{accel_y:.6f},{accel_z:.6f}\n"
    ser.write(packet.encode("ascii"))

    # receive motors from STM32
    line = ser.readline().decode("ascii", errors="ignore").strip()

    if line.startswith("M,"):
        try:
            _, l, r = line.split(",")
            left_motor = clamp(float(l), 0.0, 1.0)
            right_motor = clamp(float(r), 0.0, 1.0)
        except ValueError:
            pass

    # physics
    torque = (right_motor - left_motor) * max_thrust * arm_length
    angular_acceleration = torque / inertia

    # damping against endless oscillation
    angular_acceleration -= damping * angular_velocity

    angular_velocity += angular_acceleration * dt
    angle += angular_velocity * dt

    print(
        f"angle={math.degrees(angle):7.2f} deg | "
        f"gyro={gyro_x_deg:7.2f} deg/s | "
        f"L={left_motor:.3f} R={right_motor:.3f}"
    )

    time.sleep(dt)