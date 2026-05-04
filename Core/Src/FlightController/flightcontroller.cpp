//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/flightcontroller.h"

#include <algorithm>

#include "main.h"
#include "FlightController/mathutils.h"
#include "FlightController/PID.h"

// Stable 1-axis HIL preset:
// - smooth convergence to 0 deg
// - no overshoot
// - tested with Python sim dt=0.01
// m_rollPID = {
//     .kp = 0.008f,
//     .ki = 0.0f,
//     .kd = 0.0045f,
//     .integrator = 0.0f,
//     .previousError = 0.0f,
//     .integratorLimit = 50.0f
// };
FlightController::FlightController()
{
    m_rollPID =
    {
        .kp = 0.0004f,
        .ki = 0.0001f,
        .kd = 0.0f,
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 50.0f
    };

    m_pitchPID =
    {
        .kp = 0.0004f,
        .ki = 0.0001f,
        .kd = 0.0f,
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 50.0f
    };

    m_yawPID =
    {
        .kp = 0.0001f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 100.0f
    };

    m_rcCommand =
    {
        .throttle = 0.0f,
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f,
        .armed = false,
        .acroMode = false,
        .valid = false
    };
}

void FlightController::Init(UART_HandleTypeDef& huart2)
{
    m_huart2 = &huart2;
}

void FlightController::Update(float dt)
{
    if (!m_simImu.valid)
    {
        return;
    }

    if (m_lastProcessedImuSequence == m_imuSequence)
    {
        return; // не рахувати PID повторно на старому IMU
    }

    m_lastProcessedImuSequence = m_imuSequence;

    MotorOutputs motors = {0};

    UpdateAttitudeEstimator(GetImuDtSec());

    if (!m_armed)
    {
        SendServoOutputRaw(motors);
        return;
    }

    if (!m_gyroBiasReady)
    {
        CalibrateGyroBias();
        return;
    }

    if (m_rcCommand.throttle <= m_idleThrottleThreshold)
    {
        ResetRatePidState();
        m_filteredGyroRollDegPerSec = 0.0f;
        m_filteredGyroPitchDegPerSec = 0.0f;
        m_filteredGyroYawDegPerSec = 0.0f;

        SendServoOutputRaw({m_idleArmedThrottle, m_idleArmedThrottle, m_idleArmedThrottle, m_idleArmedThrottle});
        return;
    }

    ControlOutput control = {0};

    switch (m_flightMode)
    {
        case FlightMode::FLIGHT_MODE_ACRO:
            control = UpdateAcroController(dt);
            break;
        case FlightMode::FLIGHT_MODE_ANGLE:
            control = UpdateAngleController(dt);
            break;
    }

    motors = MixQuadX(m_rcCommand.throttle, control);

    // SendAcroDebug(0, 0, 0,
    // 0, 0, 0,
    // {}, m_rcCommand.throttle, motors);

    SendServoOutputRaw(motors);

    FlightLogSample log{};

    log.timeMs = HAL_GetTick();
    log.imuSeq = m_imuSequence;
    log.dt = dt;
    log.imuDt = GetImuDtSec();

    log.rcThrottle = m_rcCommand.throttle;
    log.rcRoll = m_rcCommand.roll;
    log.rcPitch = m_rcCommand.pitch;
    log.rcYaw = m_rcCommand.yaw;

    log.estimatedRollDeg = m_estimatedRollDeg;
    log.estimatedPitchDeg = m_estimatedPitchDeg;

    log.controlRoll = control.roll;
    log.controlPitch = control.pitch;
    log.controlYaw = control.yaw;

    log.motorM1 = motors.m1;
    log.motorM2 = motors.m2;
    log.motorM3 = motors.m3;
    log.motorM4 = motors.m4;

    log.targetRollRateDegSec = m_lastControlDebug.targetRollRateDegSec;
    log.targetPitchRateDegSec = m_lastControlDebug.targetPitchRateDegSec;
    log.targetYawRateDegSec = m_lastControlDebug.targetYawRateDegSec;

    log.gyroRollDegSec = m_lastControlDebug.gyroRollDegSec;
    log.gyroPitchDegSec = m_lastControlDebug.gyroPitchDegSec;
    log.gyroYawDegSec = m_lastControlDebug.gyroYawDegSec;

    SendFlightLogCsv(log);
}

void FlightController::Heartbeat()
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        1,                      // system_id
        1,                      // component_id
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );

    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    HAL_UART_Transmit(m_huart2, buffer, len, 100);
}

void FlightController::MavlinkParseByte(uint8_t byte)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
    {
        MavlinkHandleMessage(&msg);
    }
}

void FlightController::MavlinkHandleMessage(const mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HIL_SENSOR:
            HandleHilSensor(msg);
            break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
        {
            mavlink_manual_control_t manual;
            mavlink_msg_manual_control_decode(msg, &manual);

            // x/y/r зазвичай -1000..1000, z 0..1000
            m_rcCommand.pitch = manual.x / 1000.0f;
            m_rcCommand.roll = manual.y / 1000.0f;
            m_rcCommand.throttle = manual.z / 1000.0f;
            m_rcCommand.yaw = manual.r / 1000.0f;

            m_rcCommand.roll = MathUtils::Clamp(m_rcCommand.roll, -1.0f, 1.0f);
            m_rcCommand.pitch = MathUtils::Clamp(m_rcCommand.pitch, -1.0f, 1.0f);
            m_rcCommand.throttle = MathUtils::Clamp(m_rcCommand.throttle, 0.0f, 1.0f);
            m_rcCommand.yaw = MathUtils::Clamp(m_rcCommand.yaw, -1.0f, 1.0f);

            constexpr uint8_t armedInputMask = 1u << 1;
            constexpr uint8_t flightModeInputMask = 1u << 2;
            m_rcCommand.armed = (manual.buttons & armedInputMask) != 0;
            m_rcCommand.acroMode = (manual.buttons & flightModeInputMask) != 0;
            m_armed = m_rcCommand.armed;

            if (m_rcCommand.acroMode)
            {
                m_flightMode = FlightMode::FLIGHT_MODE_ACRO;
            }
            else
            {
                m_flightMode = FlightMode::FLIGHT_MODE_ANGLE;
            }

            m_rcCommand.valid = true;
            break;
        }
        default:
            break;
    }
}

void FlightController::HandleHilSensor(const mavlink_message_t *msg)
{
    mavlink_hil_sensor_t sensor;
    mavlink_msg_hil_sensor_decode(msg, &sensor);

    m_simImu.accel.x = sensor.xacc;
    m_simImu.accel.y = sensor.yacc;
    m_simImu.accel.z = sensor.zacc;

    m_simImu.gyro.x = sensor.xgyro;
    m_simImu.gyro.y = sensor.ygyro;
    m_simImu.gyro.z = sensor.zgyro;

    m_lastImuTimeUsec = sensor.time_usec;
    ++m_imuSequence;

    m_simImu.valid = true;
}

void FlightController::SendServoOutputRaw(const MotorOutputs motor_outputs)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    const uint16_t leftPwmFront = static_cast<uint16_t>(1000.0f + motor_outputs.m1 * 1000.0f);
    const uint16_t rightPwmFront = static_cast<uint16_t>(1000.0f + motor_outputs.m2 * 1000.0f);
    const uint16_t rightPwmBack = static_cast<uint16_t>(1000.0f + motor_outputs.m3 * 1000.0f);
    const uint16_t leftPwmBack = static_cast<uint16_t>(1000.0f + motor_outputs.m4 * 1000.0f);

    mavlink_msg_servo_output_raw_pack(
        1,
        1,
        &msg,
        HAL_GetTick() * 1000ULL,
        0,
        leftPwmFront,
        rightPwmFront,
        rightPwmBack,
        leftPwmBack,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    HAL_UART_Transmit(m_huart2, buffer, len, 100);
}

float FlightController::ApplyDeadband(float input, float deadband)
{
    if (std::abs(input) < deadband)
        return 0.0f;

    return input;
}

ControlOutput FlightController::UpdateAngleController(float dt)
{
    ControlOutput out{};

    constexpr float maxRollAngleDeg = 25.0f;
    constexpr float maxPitchAngleDeg = 25.0f;

    constexpr float RADIAN_ANGLE_MULTIPLIER = 57.2957795f;

    constexpr float angleP = 4.0f; // deg error -> deg/s target

    const float targetRollAngleDeg = m_rcCommand.roll * maxRollAngleDeg;
    const float targetPitchAngleDeg = m_rcCommand.pitch * maxPitchAngleDeg;

    const float rollAngleError = targetRollAngleDeg - m_estimatedRollDeg;
    const float pitchAngleError = targetPitchAngleDeg - m_estimatedPitchDeg;

    float targetRollRateDegSec = rollAngleError * angleP;
    float targetPitchRateDegSec = pitchAngleError * angleP;

    targetRollRateDegSec = MathUtils::Clamp(targetRollRateDegSec, -120.0f, 120.0f);
    targetPitchRateDegSec = MathUtils::Clamp(targetPitchRateDegSec, -120.0f, 120.0f);

    const float gyroRollDegSec = m_simImu.gyro.x * RADIAN_ANGLE_MULTIPLIER;
    const float gyroPitchDegSec = m_simImu.gyro.y * RADIAN_ANGLE_MULTIPLIER;

    out.roll = PID_Controller::Update(
        &m_rollPID,
        targetRollRateDegSec,
        gyroRollDegSec,
        dt
    );

    out.pitch = PID_Controller::Update(
        &m_pitchPID,
        targetPitchRateDegSec,
        gyroPitchDegSec,
        dt
    );

    out.yaw = 0.0f;

    out.roll = MathUtils::Clamp(out.roll, -0.03f, 0.03f);
    out.pitch = MathUtils::Clamp(out.pitch, -0.03f, 0.03f);

    return out;
}

float FlightController::FilterGyroRollForDebug(float gyroRollDegSec)
{
    constexpr float maxReasonableStepDegSec = 40.0f;
    constexpr float maxAbsGyroDegSec = 120.0f;

    if (fabsf(gyroRollDegSec) > maxAbsGyroDegSec)
    {
        return m_lastGoodGyroRollDegSec;
    }

    if (fabsf(gyroRollDegSec - m_lastGoodGyroRollDegSec) > maxReasonableStepDegSec)
    {
        return m_lastGoodGyroRollDegSec;
    }

    m_lastGoodGyroRollDegSec = gyroRollDegSec;
    return gyroRollDegSec;
}

void FlightController::SendFlightLogCsv(const FlightLogSample &sample)
{
    const uint32_t nowMs = HAL_GetTick();

    constexpr uint32_t logPeriodMs = 20; // 50 Hz
    if (nowMs - m_lastDebugMs < logPeriodMs)
    {
        return;
    }

    m_lastDebugMs = nowMs;

    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    auto sendNamed = [&](const char* name, float value)
    {
        mavlink_msg_named_value_float_pack(
            1,
            1,
            &msg,
            nowMs,
            name,
            value
        );

        const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        HAL_UART_Transmit(m_huart2, buffer, len, 100);
    };

    sendNamed("+++++", 1); //start log
    sendNamed("time_ms", static_cast<float>(sample.timeMs));
    sendNamed("imu_seq", static_cast<float>(sample.imuSeq));
    sendNamed("dt", sample.dt);
    sendNamed("imu_dt", sample.imuDt);
    sendNamed("rc_thr", sample.rcThrottle);
    sendNamed("rc_roll", sample.rcRoll);
    sendNamed("rc_pitch", sample.rcPitch);
    sendNamed("rc_yaw", sample.rcYaw);
    sendNamed("t_roll", sample.targetRollRateDegSec);
    sendNamed("t_pitch", sample.targetPitchRateDegSec);
    sendNamed("t_yaw", sample.targetYawRateDegSec);
    sendNamed("g_roll", sample.gyroRollDegSec);
    sendNamed("g_pitch", sample.gyroPitchDegSec);
    sendNamed("g_yaw", sample.gyroYawDegSec);
    sendNamed("est_roll", sample.estimatedRollDeg);
    sendNamed("est_pitch", sample.estimatedPitchDeg);
    sendNamed("c_roll", sample.controlRoll);
    sendNamed("c_pitch", sample.controlPitch);
    sendNamed("c_yaw", sample.controlYaw);
    sendNamed("m1", sample.motorM1);
    sendNamed("m2", sample.motorM2);
    sendNamed("m3", sample.motorM3);
    sendNamed("m4", sample.motorM4);
    sendNamed("-----", 0); //end log

    // if (len > 0)
    // {
        // HAL_UART_Transmit(m_huart2, reinterpret_cast<uint8_t*>(buffer), len, 10);
    // }
}

float FlightController::GetImuDtSec()
{
    if (m_previousImuTimeUsec == 0)
    {
        m_previousImuTimeUsec = m_lastImuTimeUsec;
        return 0.01f;
    }

    const uint64_t diffUsec = m_lastImuTimeUsec - m_previousImuTimeUsec;
    m_previousImuTimeUsec = m_lastImuTimeUsec;

    const float dt = static_cast<float>(diffUsec) * 1e-6f;
    return MathUtils::Clamp(dt, 0.001f, 0.05f);
}

ControlOutput FlightController::UpdateAcroController(float dt)
{
    ControlOutput out = {0};

    const float maxRollRateDegSec = 180.0f;
    const float maxPitchRateDegSec = 180.0f;
    const float maxYawRateDegSec = 120.0f;

    constexpr float RADIAN_ANGLE_MULTIPLIER = 57.2957795f;

    float gyroX = ApplyDeadband(m_simImu.gyro.x - m_gyroBias.x, 0.02);
    float gyroY = ApplyDeadband(m_simImu.gyro.y - m_gyroBias.y, 0.02);
    float gyroZ = ApplyDeadband(m_simImu.gyro.z - m_gyroBias.z, 0.02);
    float gyroRollDegPerSec = gyroX * RADIAN_ANGLE_MULTIPLIER;
    float gyroPitchDegPerSec = gyroY * RADIAN_ANGLE_MULTIPLIER;
    float gyroYawDegPerSec = gyroZ * RADIAN_ANGLE_MULTIPLIER;

    float targetRollRateDegSec = m_rcCommand.roll * maxRollRateDegSec;
    float targetPitchRateDegSec = m_rcCommand.pitch * maxPitchRateDegSec;
    float targetYawRateDegSec = m_rcCommand.yaw * maxYawRateDegSec;

    // float filteredGyroRollDegPerSec = FilterGyroRollForDebug(gyroRollDegPerSec);
    //
    // constexpr float alpha = 0.85f;
    //
    // m_filteredGyroRollDegPerSec =
    //     alpha * m_filteredGyroRollDegPerSec +
    //     (1.0f - alpha) * filteredGyroRollDegPerSec;

    out.roll = PID_Controller::Update(
        &m_rollPID,
        targetRollRateDegSec,
        gyroRollDegPerSec,
        dt
    );

    out.pitch = PID_Controller::Update(
        &m_pitchPID,
        targetPitchRateDegSec,
        gyroPitchDegPerSec,
        dt
    );

    out.yaw = PID_Controller::Update(
        &m_yawPID,
        targetYawRateDegSec,
        gyroYawDegPerSec,
        dt
    );


    // out.roll = MathUtils::Clamp(out.roll, -0.25f, 0.25f);
    // out.pitch = MathUtils::Clamp(out.pitch, -0.25f, 0.25f);
    // out.yaw = MathUtils::Clamp(out.yaw, -0.20f, 0.20f);

    // float throttleScale = MathUtils::Clamp(m_rcCommand.throttle / 0.6f, 0.0f, 1.0f);
    // out.roll *= throttleScale;

    out.roll = MathUtils::Clamp(out.roll, -0.05f, 0.05f);
    out.pitch = MathUtils::Clamp(out.pitch, -0.05f, 0.05f);
    out.yaw = MathUtils::Clamp(out.yaw, -0.20f, 0.20f);

    // out.roll = out.roll;
    // out.pitch = 0.0;
    out.yaw = 0.0;

    // SendAcroDebug(targetRollRateDegSec, targetPitchRateDegSec, targetYawRateDegSec,
    //                 gyroRollDegPerSec, gyroPitchDegPerSec, gyroYawDegPerSec,
    //                 out, m_rcCommand.throttle);

    m_lastControlDebug.targetRollRateDegSec = targetRollRateDegSec;
    m_lastControlDebug.targetPitchRateDegSec = targetPitchRateDegSec;
    m_lastControlDebug.targetYawRateDegSec = targetYawRateDegSec;

    m_lastControlDebug.gyroRollDegSec = gyroRollDegPerSec;
    m_lastControlDebug.gyroPitchDegSec = gyroPitchDegPerSec;
    m_lastControlDebug.gyroYawDegSec = gyroYawDegPerSec;

    return out;
}

void FlightController::UpdateAttitudeEstimator(float dt)
{
    constexpr float radToDeg = 57.2957795f;

    const float gyroRollDegPerSec = m_simImu.gyro.x * radToDeg;
    const float gyroPitchDegPerSec = m_simImu.gyro.y * radToDeg;

    const float accelRollDeg =
        atan2f(m_simImu.accel.y, m_simImu.accel.z) * radToDeg;

    const float accelPitchDeg =
        atan2f(
            -m_simImu.accel.x,
            sqrtf(m_simImu.accel.y * m_simImu.accel.y +
                  m_simImu.accel.z * m_simImu.accel.z)
        ) * radToDeg;

    if (!m_estimatorInitialized)
    {
        m_estimatedRollDeg = accelRollDeg;
        m_estimatedPitchDeg = accelPitchDeg;
        m_estimatorInitialized = true;
        return;
    }

    constexpr float alpha = 0.98f;

    m_estimatedRollDeg =
        alpha * (m_estimatedRollDeg + gyroRollDegPerSec * dt)
      + (1.0f - alpha) * accelRollDeg;

    m_estimatedPitchDeg =
        alpha * (m_estimatedPitchDeg + gyroPitchDegPerSec * dt)
      + (1.0f - alpha) * accelPitchDeg;
}

// void FlightController::SendAcroDebug(float targetRollRateDegSec, float targetPitchRateDegSec, float targetYawRateDegSec,
//                                      float gyroRollDegPerSec, float gyroPitchDegPerSec, float gyroYawDegPerSec,
//                                      // float accelRollDeg, float accelPitchDeg, float throttleAuthority,
//                                      ControlOutput control_output, float throttle, MotorOutputs motors)
// {
//     const uint32_t nowMs = HAL_GetTick();
//
//     if (nowMs - m_lastDebugMs < 100)
//     {
//         return;
//     }
//
//     m_lastDebugMs = nowMs;
//
//     mavlink_message_t msg;
//     uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//
//     auto sendNamed = [&](const char* name, float value)
//     {
//         mavlink_msg_named_value_float_pack(
//             1,
//             1,
//             &msg,
//             nowMs,
//             name,
//             value
//         );
//
//         const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
//         HAL_UART_Transmit(m_huart2, buffer, len, 100);
//     };
//
//     // sendNamed("=======", 1);
//     // sendNamed("tick", static_cast<float>(m_lastProcessedImuSequence));
//     // sendNamed("t_roll", targetRollRateDegSec);
//     // sendNamed("raw_g_roll", gyroRollDegPerSec);
//     // sendNamed("filtered_g_roll", m_filteredGyroRollDegPerSec);
//     // sendNamed("t_pitch", targetPitchRateDegSec);
//     // sendNamed("g_pitch", gyroPitchDegPerSec);
//     // sendNamed("t_yaw", targetYawRateDegSec);
//     // sendNamed("g_yaw", gyroYawDegPerSec);
//     // sendNamed("a_roll", accelRollDeg);
//     // sendNamed("a_pitch", accelPitchDeg);
//     // sendNamed("auth", throttleAuthority);
//     // sendNamed("throttle", throttle);
//     // sendNamed("c_roll", control_output.roll);
//     // sendNamed("motor.m1", motors.m1);
//     // sendNamed("motor.m2", motors.m2);
//     // sendNamed("motor.m3", motors.m3);
//     // sendNamed("motor.m4", motors.m4);
//     // sendNamed("c_pitch", control_output.pitch);
//     // sendNamed("c_yaw", control_output.yaw);
//     // sendNamed("_______", 0);
//
// }

void FlightController::ResetRatePidState()
{
    m_rollPID.integrator = 0.0f;
    m_rollPID.previousError = 0.0f;

    m_pitchPID.integrator = 0.0f;
    m_pitchPID.previousError = 0.0f;

    m_yawPID.integrator = 0.0f;
    m_yawPID.previousError = 0.0f;
}

void FlightController::CalibrateGyroBias()
{
    constexpr int sampleCount = 500;
    static int currentSampleCount = 0;

    static Vector3 sum{};

    if (currentSampleCount < sampleCount)
    {
        sum.x += m_simImu.gyro.x;
        sum.y += m_simImu.gyro.y;
        sum.z += m_simImu.gyro.z;

        currentSampleCount++;
        return;
    }

    m_gyroBias.x = sum.x / sampleCount;
    m_gyroBias.y = sum.y / sampleCount;
    m_gyroBias.z = sum.z / sampleCount;

    m_gyroBiasReady = true;
}

MotorOutputs FlightController::MixQuadX(const float throttle, const ControlOutput& controlOutput)
{
    MotorOutputs motorOutputs{};

    /*
    Motor layout:
          front

      M3       M1
         \   /
          \ /
          / \
         /   \
      M2       M4

          back
    */

    constexpr float correctionFullAtThrottle = 0.35f;

    if (throttle <= m_idleThrottleThreshold)
    {
        return motorOutputs;
    }

    const float correctionScale = MathUtils::Clamp01(
        (throttle - m_idleThrottleThreshold) / (correctionFullAtThrottle - m_idleThrottleThreshold));

    const float roll = controlOutput.roll * correctionScale;
    const float pitch = controlOutput.pitch * correctionScale;
    const float yaw = controlOutput.yaw * correctionScale;

    motorOutputs.m1 = throttle + roll + pitch - yaw;
    motorOutputs.m2 = throttle - roll - pitch - yaw;
    motorOutputs.m3 = throttle - roll + pitch + yaw;
    motorOutputs.m4 = throttle + roll - pitch + yaw;

    // motorOutputs.m1 = throttle + pitch;
    // motorOutputs.m2 = throttle - pitch;
    // motorOutputs.m3 = throttle + pitch;
    // motorOutputs.m4 = throttle - pitch;

    motorOutputs = DesaturateMotors(motorOutputs);
    // motorOutputs.m1 = MathUtils::Clamp01(motorOutputs.m1);
    // motorOutputs.m2 = MathUtils::Clamp01(motorOutputs.m2);
    // motorOutputs.m3 = MathUtils::Clamp01(motorOutputs.m3);
    // motorOutputs.m4 = MathUtils::Clamp01(motorOutputs.m4);

    return motorOutputs;
}

MotorOutputs FlightController::DesaturateMotors(MotorOutputs motorOutputs)
{
    float maxMotor = std::max(
        std::max(motorOutputs.m1, motorOutputs.m2),
        std::max(motorOutputs.m3, motorOutputs.m4)
    );

    float minMotor = std::min(
        std::min(motorOutputs.m1, motorOutputs.m2),
        std::min(motorOutputs.m3, motorOutputs.m4)
    );

    if (maxMotor > 1.0f)
    {
        float excess = maxMotor - 1.0f;

        motorOutputs.m1 -= excess;
        motorOutputs.m2 -= excess;
        motorOutputs.m3 -= excess;
        motorOutputs.m4 -= excess;
    }

    if (minMotor < 0.0f)
    {
        float deficit = -minMotor;

        motorOutputs.m1 += deficit;
        motorOutputs.m2 += deficit;
        motorOutputs.m3 += deficit;
        motorOutputs.m4 += deficit;
    }

    motorOutputs.m1 = MathUtils::Clamp01(motorOutputs.m1);
    motorOutputs.m2 = MathUtils::Clamp01(motorOutputs.m2);
    motorOutputs.m3 = MathUtils::Clamp01(motorOutputs.m3);
    motorOutputs.m4 = MathUtils::Clamp01(motorOutputs.m4);

    return motorOutputs;
}
