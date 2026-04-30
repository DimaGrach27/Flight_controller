import math
import time
import random
from pymavlink import mavutil
import matplotlib.pyplot as plt

PORT = "/dev/cu.usbmodem103"
BAUD = 115200

master = mavutil.mavlink_connection(PORT, baud=BAUD)

print("Waiting for Nucleo HEARTBEAT...")
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system}, component {master.target_component}")

angle = math.radians(20.0)
angular_velocity = 0.0

left_motor = 0.5
right_motor = 0.5

dt = 0.01

arm_length = 0.15
inertia = 0.02
max_thrust = 1.0
damping = 0.02

t = 0

times = []
angles = []
gyros = []
left_outputs = []
right_outputs = []

def clamp(x, a, b):
    return max(a, min(b, x))


def simulation():
    global angle
    global angular_velocity
    global left_motor
    global right_motor
    global t

    now_us = int(time.time() * 1_000_000)

    gyro_noise = random.gauss(0.0, math.radians(0.3))
    accel_noise = random.gauss(0.0, 0.03)

    gyro_x_rad = angular_velocity + gyro_noise

    accel_x = 0.0 + random.gauss(0.0, 0.03)
    accel_y = math.sin(angle) * 9.81 + accel_noise
    accel_z = math.cos(angle) * 9.81 + random.gauss(0.0, 0.03)

    # Send fake IMU to Nucleo
    master.mav.hil_sensor_send(
        now_us,
        accel_x,
        accel_y,
        accel_z,
        gyro_x_rad,
        0.0,
        0.0,
        0.0, 0.0, 0.0,       # magnetometer
        1013.25,             # abs pressure hPa
        0.0,                 # diff pressure
        0.0,                 # pressure altitude
        25.0,                # temperature
        0xFFFF               # fields_updated
    )

    # Read all pending messages
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        if msg.get_type() == "SERVO_OUTPUT_RAW":
            # servo1_raw and servo2_raw are 1000..2000
            left_motor = clamp((msg.servo1_raw - 1000) / 1000.0, 0.0, 1.0)
            right_motor = clamp((msg.servo2_raw - 1000) / 1000.0, 0.0, 1.0)

    # Physics update
    torque = (right_motor - left_motor) * max_thrust * arm_length
    angular_acceleration = torque / inertia

    angular_acceleration -= damping * angular_velocity

    angular_velocity += angular_acceleration * dt
    angle += angular_velocity * dt

    print(
        f"angle={math.degrees(angle):7.2f} deg | "
        f"gyro={math.degrees(angular_velocity):7.2f} deg/s | "
        f"L={left_motor:.3f} R={right_motor:.3f}"
    )

    t += dt

    times.append(t)
    angles.append(math.degrees(angle))
    gyros.append(math.degrees(angular_velocity))
    left_outputs.append(left_motor)
    right_outputs.append(right_motor)

    time.sleep(dt)

try:
    while True:
        simulation()
except KeyboardInterrupt:
    plt.figure()
    plt.plot(times, angles, label="angle deg")
    plt.plot(times, gyros, label="gyro deg/s")
    plt.legend()
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.plot(times, left_outputs, label="left motor")
    plt.plot(times, right_outputs, label="right motor")
    plt.legend()
    plt.grid(True)
    plt.show()