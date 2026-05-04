import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("imu_hil_log.csv")

t = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0

plt.figure()
plt.plot(t, df["rc_roll"], label="rc_roll")
plt.plot(t, df["t_roll"], label="target_roll_rate")
plt.plot(t, df["g_roll"], label="gyro_roll")
plt.legend()
plt.grid()
plt.title("Roll input / target / gyro")
plt.show()

plt.figure()
plt.plot(t, df["est_roll"], label="estimated_roll")
plt.plot(t, df["est_pitch"], label="estimated_pitch")
plt.legend()
plt.grid()
plt.title("Estimated attitude")
plt.show()

plt.figure()
plt.plot(t, df["c_roll"], label="control_roll")
plt.plot(t, df["c_pitch"], label="control_pitch")
plt.legend()
plt.grid()
plt.title("Controller output")
plt.show()

plt.figure()
plt.plot(t, df["m1"], label="m1")
plt.plot(t, df["m2"], label="m2")
plt.plot(t, df["m3"], label="m3")
plt.plot(t, df["m4"], label="m4")
plt.legend()
plt.grid()
plt.title("Motor outputs")
plt.show()