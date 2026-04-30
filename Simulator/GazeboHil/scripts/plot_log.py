import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("one_axis_hil_log.csv")

plt.figure()
plt.plot(df["time"], df["angle_deg"], label="angle deg")
plt.plot(df["time"], df["gyro_deg_s"], label="gyro deg/s")
plt.grid(True)
plt.legend()
plt.title("One-axis HIL attitude")

plt.figure()
plt.plot(df["time"], df["left_motor"], label="left motor")
plt.plot(df["time"], df["right_motor"], label="right motor")
plt.grid(True)
plt.legend()
plt.title("Motor outputs")

plt.figure()
plt.plot(df["time"], df["torque"], label="torque")
plt.grid(True)
plt.legend()
plt.title("Applied torque")

plt.show()