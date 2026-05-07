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
        .kp = 0.8f,
        .ki = 0.0f, //0.05
        .kd = 0.0f, //0.02
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 100.0f
    };

    m_pitchPID =
    {
        .kp = 0.8f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 100.0f
    };

    m_yawPID =
    {
        .kp = 0.6f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 50.0f
    };

    m_rcCommand =
    {
        .throttle = 0,
        .roll = 0,
        .pitch = 0,
        .yaw = 0,
        .armed = false,
        .acroMode = false,
        .valid = false
    };


}

FlightController::~FlightController()
{
    if (m_logger)
    {
        delete m_logger;
    }
}

void FlightController::Init(UART_HandleTypeDef& huart1, UART_HandleTypeDef& huart2)
{
    m_huart1 = &huart1;
    m_huart2 = &huart2;

    m_logger = new Logger(huart2);

    m_crsfTelemetry.Init(huart1);
}

void FlightController::UpdateFormNewImuSample()
{
    if (!m_simImu.valid)
    {
        return;
    }

    if (m_lastProcessedImuSequence == m_imuSequence)
    {
        return;
    }

    m_lastProcessedImuSequence = m_imuSequence;

    MotorOutputs motors = {0};

    const float imuDt = GetImuDtSec();

    UpdateAttitudeEstimator(imuDt);

    if (!m_gyroBiasReady)
    {
        CalibrateGyroBias();
        return;
    }

    if (!m_levelOffsetReady)
    {
        CalibrateLevelOffset();
        return;
    }

    if (!m_armed)
    {
        SendServoOutputRaw(motors);
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
            control = UpdateAcroController(imuDt);
            break;
        case FlightMode::FLIGHT_MODE_ANGLE:
            control = UpdateAngleController(imuDt);
            break;
    }

    motors = MixQuadX(m_rcCommand.throttle, control);

    // motors.m1 = m_rcCommand.throttle;
    // motors.m2 = m_rcCommand.throttle;
    // motors.m3 = m_rcCommand.throttle;
    // motors.m4 = m_rcCommand.throttle;

    SendServoOutputRaw(motors);

    ++m_controlSequence;

    if (m_logger)
    {
        const uint32_t nowMs = HAL_GetTick();
        m_logger->GetLogSample().halDt = static_cast<float>(nowMs - m_previousHalLogMs) * 0.001f;
        m_previousHalLogMs = nowMs;

        m_logger->GetLogSample().flightMode = m_flightMode == FlightMode::FLIGHT_MODE_ACRO ? 1 : 0;
        m_logger->GetLogSample().armed = m_armed ? 1 : 0;

        m_logger->GetLogSample().controlSeq = m_controlSequence;
        m_logger->GetLogSample().logSeq = ++m_logSequence;
        m_logger->GetLogSample().timeMs = HAL_GetTick();
        m_logger->GetLogSample().imuSeq = m_imuSequence;
        // log.dt = dt;
        m_logger->GetLogSample().imuDt = imuDt;

        m_logger->GetLogSample().rcThrottle = m_rcCommand.throttle;
        m_logger->GetLogSample().rcRoll = m_rcCommand.roll;
        m_logger->GetLogSample().rcPitch = m_rcCommand.pitch;
        m_logger->GetLogSample().rcYaw = m_rcCommand.yaw;

        m_logger->GetLogSample().estimatedRollDeg = m_estimatedRollDeg;
        m_logger->GetLogSample().estimatedPitchDeg = m_estimatedPitchDeg;

        m_logger->GetLogSample().controlRoll = control.roll;
        m_logger->GetLogSample().controlPitch = control.pitch;
        m_logger->GetLogSample().controlYaw = control.yaw;

        m_logger->GetLogSample().motorM1 = motors.m1;
        m_logger->GetLogSample().motorM2 = motors.m2;
        m_logger->GetLogSample().motorM3 = motors.m3;
        m_logger->GetLogSample().motorM4 = motors.m4;

        m_logger->GetLogSample().targetRollRateDegSec = m_lastControlDebug.targetRollRateDegSec;
        m_logger->GetLogSample().targetPitchRateDegSec = m_lastControlDebug.targetPitchRateDegSec;
        m_logger->GetLogSample().targetYawRateDegSec = m_lastControlDebug.targetYawRateDegSec;

        m_logger->GetLogSample().gyroRollDegSec = m_lastControlDebug.gyroRollDegSec;
        m_logger->GetLogSample().gyroPitchDegSec = m_lastControlDebug.gyroPitchDegSec;
        m_logger->GetLogSample().gyroYawDegSec = m_lastControlDebug.gyroYawDegSec;

        m_logger->SendFlightLogCsv();
    }
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

void FlightController::Update()
{
    constexpr uint32_t UPDATE_INTERVAL_MS = 1000;
    const uint32_t nowMs = HAL_GetTick();

    static uint32_t previousMs = 0;

    if (nowMs - previousMs < UPDATE_INTERVAL_MS)
    {
        return;
    }


    if (m_armed)
    {
        const char* flightMode = m_flightMode == FlightMode::FLIGHT_MODE_ACRO ? "ARM ACRO" : "ARM ANGLE";
        m_crsfTelemetry.SendFlightMode(flightMode);
    }
    else
    {
        m_crsfTelemetry.SendFlightMode("DISARM");
    }

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

void FlightController::ParseRcCommandByte(uint8_t byte)
{
    if (m_crsfReceiver.ProcessByte(byte))
    {
        HandleRcCommand();
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
            // HandleRcCommand(msg);
            break;
        default:
            break;
    }
}

void FlightController::HandleHilSensor(const mavlink_message_t* msg)
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

    UpdateFormNewImuSample();
}

void FlightController::HandleRcCommand(const mavlink_message_t* msg)
{
    mavlink_manual_control_t manual;
    mavlink_msg_manual_control_decode(msg, &manual);

    // x/y/r зазвичай -1000..1000, z 0..1000
    m_rcCommand.pitch = MathUtils::ApplyDeadband(manual.x, 25);
    m_rcCommand.roll = MathUtils::ApplyDeadband(manual.y, 25);
    m_rcCommand.throttle = MathUtils::ApplyDeadband(manual.z, 25u);
    m_rcCommand.yaw = MathUtils::ApplyDeadband(manual.r, 25);

    m_rcCommand.roll = MathUtils::Clamp(m_rcCommand.roll, -1000, 1000);
    m_rcCommand.pitch = MathUtils::Clamp(m_rcCommand.pitch, -1000, 1000);
    m_rcCommand.throttle = MathUtils::Clamp(m_rcCommand.throttle, 0, 1000);
    m_rcCommand.yaw = MathUtils::Clamp(m_rcCommand.yaw, -1000, 1000);

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
}

void FlightController::HandleRcCommand()
{
    const CrsfReceiver::Channels& channels = m_crsfReceiver.GetChannels();

    if (!channels.valid)
    {
        return;
    }

    float pitch = m_crsfReceiver.NormalizeStick(channels.us[0]);
    float roll = m_crsfReceiver.NormalizeStick(channels.us[1]);;
    float throttle = m_crsfReceiver.NormalizeThrottle(channels.us[2]);;
    float yaw = m_crsfReceiver.NormalizeStick(channels.us[3]);;

    m_rcCommand.pitch = MathUtils::ApplyDeadband(pitch, 25);
    m_rcCommand.roll = MathUtils::ApplyDeadband(roll, 25);
    m_rcCommand.throttle = MathUtils::ApplyDeadband(throttle, 25u);
    m_rcCommand.yaw = MathUtils::ApplyDeadband(yaw, 25);

    m_rcCommand.roll = MathUtils::Clamp(m_rcCommand.roll, -1000, 1000);
    m_rcCommand.pitch = MathUtils::Clamp(m_rcCommand.pitch, -1000, 1000);
    m_rcCommand.throttle = MathUtils::Clamp(m_rcCommand.throttle, 0, 1000);
    m_rcCommand.yaw = MathUtils::Clamp(m_rcCommand.yaw, -1000, 1000);

    m_rcCommand.armed = channels.us[4] > 1500;
    m_rcCommand.acroMode = channels.us[5] > 1500;

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
}

void FlightController::SendByteToRc(uint8_t byte)
{

}

void FlightController::SendServoOutputRaw(const MotorOutputs motor_outputs)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    const uint16_t leftPwmFront = (1000 + motor_outputs.m1);
    const uint16_t rightPwmFront = (1000 + motor_outputs.m2);
    const uint16_t rightPwmBack = (1000 + motor_outputs.m3);
    const uint16_t leftPwmBack = (1000 + motor_outputs.m4);

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

ControlOutput FlightController::UpdateAngleController(float dt)
{
    ControlOutput out{};

    constexpr float maxRollAngleDeg = 25.0f;
    constexpr float maxPitchAngleDeg = 25.0f;
    constexpr float angleP = 4.0f;
    constexpr float maxLevelRateDegSec = 120.0f;
    constexpr float radToDeg = 57.2957795f;
    constexpr float manualRollTrimDeg = 0.0f;
    constexpr float manualPitchTrimDeg = 0.0f;
    constexpr float angleErrorDeadbandDeg = 0.15f;

    const float rollStick = static_cast<float>(m_rcCommand.roll) / 1000.0f;
    const float pitchStick = static_cast<float>(m_rcCommand.pitch) / 1000.0f;
    const float yawStick = static_cast<float>(m_rcCommand.yaw) / 1000.0f;

    const float targetRollAngleDeg = rollStick * maxRollAngleDeg + manualRollTrimDeg;
    const float targetPitchAngleDeg = pitchStick * maxPitchAngleDeg + manualPitchTrimDeg;

    const float correctedRollDeg = m_estimatedRollDeg - m_levelRollOffsetDeg;
    const float correctedPitchDeg = m_estimatedPitchDeg - m_levelPitchOffsetDeg;

    float rollAngleError = targetRollAngleDeg - correctedRollDeg;
    float pitchAngleError = targetPitchAngleDeg - correctedPitchDeg;

    if (fabsf(rollAngleError) < angleErrorDeadbandDeg)
    {
        rollAngleError = 0.0f;
        m_rollPID.previousError = 0.0f;
        m_rollPID.integrator = 0.0f;
    }

    if (fabsf(pitchAngleError) < angleErrorDeadbandDeg)
    {
        pitchAngleError = 0.0f;
        m_pitchPID.previousError = 0.0f;
        m_pitchPID.integrator = 0.0f;
    }

    float targetRollRateDegSec = rollAngleError * angleP;
    float targetPitchRateDegSec = pitchAngleError * angleP;

    targetRollRateDegSec = MathUtils::Clamp(targetRollRateDegSec, -maxLevelRateDegSec, maxLevelRateDegSec);
    targetPitchRateDegSec = MathUtils::Clamp(targetPitchRateDegSec, -maxLevelRateDegSec, maxLevelRateDegSec);

    float gyroRollDegSec = 0.0;
    float gyroPitchDegSec = 0.0;
    float gyroYawDegSec = 0.0;

    if (m_gyroBiasReady)
    {
        gyroRollDegSec = (m_simImu.gyro.x - m_gyroBias.x) * radToDeg;
        gyroPitchDegSec = (m_simImu.gyro.y - m_gyroBias.y) * radToDeg;
        gyroYawDegSec = (m_simImu.gyro.z - m_gyroBias.z) * radToDeg;
    }
    else
    {
        gyroRollDegSec = m_simImu.gyro.x * radToDeg;
        gyroPitchDegSec = m_simImu.gyro.y * radToDeg;
        gyroYawDegSec = m_simImu.gyro.z * radToDeg;
    }

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

    constexpr float maxYawRateDegSec = 90.0f;
    const float targetYawRateDegSec = yawStick * maxYawRateDegSec;

    out.yaw = PID_Controller::Update(
        &m_yawPID,
        targetYawRateDegSec,
        gyroYawDegSec,
        dt
    );

    out.roll = MathUtils::Clamp(out.roll, -20, 20);
    out.pitch = MathUtils::Clamp(out.pitch, -20, 20);
    out.yaw = MathUtils::Clamp(out.yaw, -30, 30);

    constexpr float controlDeadband = 1.5f;

    if (fabsf(out.roll) < controlDeadband)
    {
        out.roll = 0.0f;
    }

    if (fabsf(out.pitch) < controlDeadband)
    {
        out.pitch = 0.0f;
    }

    if (fabsf(out.yaw) < controlDeadband)
    {
        out.yaw = 0.0f;
    }

    m_lastControlDebug.targetRollRateDegSec = targetRollRateDegSec;
    m_lastControlDebug.targetPitchRateDegSec = targetPitchRateDegSec;
    m_lastControlDebug.targetYawRateDegSec = targetYawRateDegSec;

    m_lastControlDebug.gyroRollDegSec = gyroRollDegSec;
    m_lastControlDebug.gyroPitchDegSec = gyroPitchDegSec;
    m_lastControlDebug.gyroYawDegSec = gyroYawDegSec;

    if (m_logger)
    {
        m_logger->GetLogSample().correctedRoll = correctedRollDeg;
        m_logger->GetLogSample().correctedPitch = correctedPitchDeg;
        m_logger->GetLogSample().angleErrorRoll = rollAngleError;
        m_logger->GetLogSample().angleErrorPitch = pitchAngleError;
    }

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

    constexpr float maxRollRateDegSec = 180.0f;
    constexpr float maxPitchRateDegSec = 180.0f;
    constexpr float maxYawRateDegSec = 90.0f;

    constexpr float radToDeg = 57.2957795f;

    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;

    if (m_gyroBiasReady)
    {
        gyroX = m_simImu.gyro.x - m_gyroBias.x;
        gyroY = m_simImu.gyro.y - m_gyroBias.y;
        gyroZ = m_simImu.gyro.z - m_gyroBias.z;
    }
    else
    {
        gyroX = m_simImu.gyro.x;
        gyroY = m_simImu.gyro.y;
        gyroZ = m_simImu.gyro.z;
    }

    float gyroRollDegPerSec = gyroX * radToDeg;
    float gyroPitchDegPerSec = gyroY * radToDeg;
    float gyroYawDegPerSec = gyroZ * radToDeg;

    constexpr float gyroRateDeadbandDegSec = 1.5f;

    if (fabsf(gyroRollDegPerSec) < gyroRateDeadbandDegSec)
    {
        gyroRollDegPerSec = 0.0f;
    }

    if (fabsf(gyroPitchDegPerSec) < gyroRateDeadbandDegSec)
    {
        gyroPitchDegPerSec = 0.0f;
    }

    if (fabsf(gyroYawDegPerSec) < gyroRateDeadbandDegSec)
    {
        gyroYawDegPerSec = 0.0f;
    }

    const float targetRollRateDegSec =
        static_cast<float>(m_rcCommand.roll) / 1000.0f * maxRollRateDegSec;

    const float targetPitchRateDegSec =
        static_cast<float>(m_rcCommand.pitch) / 1000.0f * maxPitchRateDegSec;

    const float targetYawRateDegSec =
        static_cast<float>(m_rcCommand.yaw) / 1000.0f * maxYawRateDegSec;

    constexpr float targetRateDeadbandDegSec = 0.5f;

    const bool rollIdle =
        fabsf(targetRollRateDegSec) < targetRateDeadbandDegSec &&
        fabsf(gyroRollDegPerSec) < gyroRateDeadbandDegSec;

    const bool pitchIdle =
        fabsf(targetPitchRateDegSec) < targetRateDeadbandDegSec &&
        fabsf(gyroPitchDegPerSec) < gyroRateDeadbandDegSec;

    const bool yawIdle =
        fabsf(targetYawRateDegSec) < targetRateDeadbandDegSec &&
        fabsf(gyroYawDegPerSec) < gyroRateDeadbandDegSec;

    if (rollIdle)
    {
        m_rollPID.previousError = 0.0f;
        m_rollPID.integrator = 0.0f;
        out.roll = 0.0f;
    }
    else
    {
        out.roll = PID_Controller::Update(
            &m_rollPID,
            targetRollRateDegSec,
            gyroRollDegPerSec,
            dt
        );
    }

    if (pitchIdle)
    {
        m_pitchPID.previousError = 0.0f;
        m_pitchPID.integrator = 0.0f;
        out.pitch = 0.0f;
    }
    else
    {
        out.pitch = PID_Controller::Update(
            &m_pitchPID,
            targetPitchRateDegSec,
            gyroPitchDegPerSec,
            dt
        );
    }

    if (yawIdle)
    {
        m_yawPID.previousError = 0.0f;
        m_yawPID.integrator = 0.0f;
        out.yaw = 0.0f;
    }
    else
    {
        out.yaw = PID_Controller::Update(
            &m_yawPID,
            targetYawRateDegSec,
            gyroYawDegPerSec,
            dt
        );
    }

    out.roll = MathUtils::Clamp(out.roll, -20.0f, 20.0f);
    out.pitch = MathUtils::Clamp(out.pitch, -20.0f, 20.0f);
    out.yaw = MathUtils::Clamp(out.yaw, -30.0f, 30.0f);

    constexpr float acroControlDeadband = 1.5f;

    if (fabsf(out.roll) < acroControlDeadband)
    {
        out.roll = 0.0f;
    }

    if (fabsf(out.pitch) < acroControlDeadband)
    {
        out.pitch = 0.0f;
    }

    if (fabsf(out.yaw) < acroControlDeadband)
    {
        out.yaw = 0.0f;
    }

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

    float gyroRollDegPerSec = 0.0;
    float gyroPitchDegPerSec = 0.0;

    if (m_gyroBiasReady)
    {
        gyroRollDegPerSec = (m_simImu.gyro.x - m_gyroBias.x) * radToDeg;
        gyroPitchDegPerSec = (m_simImu.gyro.y - m_gyroBias.y) * radToDeg;
    }
    // else
    // {
    //     gyroRollDegPerSec = m_simImu.gyro.x * radToDeg;
    //     gyroPitchDegPerSec = m_simImu.gyro.y * radToDeg;
    // }

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

    if (m_logger)
    {
        m_logger->GetLogSample().accelRoll = accelRollDeg;
        m_logger->GetLogSample().accelPitch = accelPitchDeg;
    }
}

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

void FlightController::CalibrateLevelOffset()
{
    constexpr int sampleCount = 100;
    static int currentSampleCount = 0;

    static float sumLevelRollOffsetDeg = 0.0f;
    static float sumLevelPitchOffsetDeg = 0.0f;

    if (currentSampleCount < sampleCount)
    {
        sumLevelRollOffsetDeg += m_estimatedRollDeg;
        sumLevelPitchOffsetDeg += m_estimatedPitchDeg;

        currentSampleCount++;
        return;
    }

    m_levelRollOffsetDeg = sumLevelRollOffsetDeg / sampleCount;
    m_levelPitchOffsetDeg = sumLevelPitchOffsetDeg / sampleCount;
    m_levelOffsetReady = true;
}

MotorOutputs FlightController::MixQuadX(const uint16_t throttle, const ControlOutput& controlOutput)
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

    constexpr uint16_t correctionFullAtThrottle = 350;

    if (throttle <= m_idleThrottleThreshold)
    {
        return motorOutputs;
    }


    const int32_t correctionScale =
        MathUtils::Clamp(
            static_cast<int16_t>(
                ((static_cast<int32_t>(throttle) - m_idleThrottleThreshold) * 1000) /
                (correctionFullAtThrottle - m_idleThrottleThreshold)
            ),
            static_cast<int16_t>(0),
            static_cast<int16_t>(1000)
        );

    const int16_t roll  = static_cast<int16_t>((static_cast<int32_t>(controlOutput.roll)  * correctionScale) / 1000);
    const int16_t pitch = static_cast<int16_t>((static_cast<int32_t>(controlOutput.pitch) * correctionScale) / 1000);
    const int16_t yaw   = static_cast<int16_t>((static_cast<int32_t>(controlOutput.yaw)   * correctionScale) / 1000);

    motorOutputs.m1 = throttle + roll + pitch - yaw;
    motorOutputs.m2 = throttle - roll - pitch - yaw;
    motorOutputs.m3 = throttle - roll + pitch + yaw;
    motorOutputs.m4 = throttle + roll - pitch + yaw;

    motorOutputs = DesaturateMotors(motorOutputs);

    return motorOutputs;
}

MotorOutputs FlightController::DesaturateMotors(MotorOutputs motorOutputs)
{
    int16_t maxMotor = std::max(
        std::max(motorOutputs.m1, motorOutputs.m2),
        std::max(motorOutputs.m3, motorOutputs.m4)
    );

    int16_t minMotor = std::min(
        std::min(motorOutputs.m1, motorOutputs.m2),
        std::min(motorOutputs.m3, motorOutputs.m4)
    );

    if (maxMotor > 1000)
    {
        int16_t excess = maxMotor - 1000;

        motorOutputs.m1 -= excess;
        motorOutputs.m2 -= excess;
        motorOutputs.m3 -= excess;
        motorOutputs.m4 -= excess;
    }

    if (minMotor < 0)
    {
        int16_t deficit = -minMotor;

        motorOutputs.m1 += deficit;
        motorOutputs.m2 += deficit;
        motorOutputs.m3 += deficit;
        motorOutputs.m4 += deficit;
    }

    motorOutputs.m1 = MathUtils::Clamp(motorOutputs.m1, 0, 1000);
    motorOutputs.m2 = MathUtils::Clamp(motorOutputs.m2, 0, 1000);
    motorOutputs.m3 = MathUtils::Clamp(motorOutputs.m3, 0, 1000);
    motorOutputs.m4 = MathUtils::Clamp(motorOutputs.m4, 0, 1000);

    return motorOutputs;
}
