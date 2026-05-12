Milestone 2:
- Gazebo 2-axis HIL works
- Nucleo receives MAVLink HIL_SENSOR
- Nucleo receives MAVLink MANUAL_CONTROL from RadioMaster TX12 via plugin
- Nucleo outputs 4 virtual motors via SERVO_OUTPUT_RAW
- Gazebo object follows stick input and self-levels when sticks are centered

Milestone 3:
- Rate/Acro mode
- Possibility to change flight mode by input from RadioMaster TX12
- Acro Mode roll/pitch/yaw

Milestone 4:
- Optimized UART transport
- Possibility to fly on stand but with small drifting
- PIDs before tuning
-     pitchPID = rollPID
      {
          .kp = 0.0004f,
          .ki = 0.0001f,
          .kd = 0.0f,
          .integrator = 0.0f,
          .previousError = 0.0f,
          .integratorLimit = 50.0f
      };

Milestone 5:
- Architecture changing. Separate moduls for Sensors, Estimation, RC Command, Rate control, Mixer, MotorOutput
- Dynamic using in HIL and real drone, by changing one Definition <code>NOT_USE_HIL 0/1</code>