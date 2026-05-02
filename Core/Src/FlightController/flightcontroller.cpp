//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/flightcontroller.h"

#include "stm32f4xx_hal_uart.h"
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
        .kp = 0.008f,
        .ki = 0.0f,
        .kd = 0.0045f,
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 50.0f
    };

    m_pitchPID =
    {
        .kp = 0.008f,
        .ki = 0.0f,
        .kd = 0.0045f,
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 50.0f
    };

    m_rcCommand =
    {
        .throttle = 0.5f,
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f,
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

    float maxAngleDeg = 20.0f;
    float throttle = m_rcCommand.throttle;
    float targetRollDeg = m_rcCommand.roll * maxAngleDeg;
    float targetPitchDeg = m_rcCommand.pitch * maxAngleDeg;

    if (!m_rcCommand.valid)
    {
        throttle = 0.5f;
        targetRollDeg = 0.0f;
        targetPitchDeg = 0.0f;
    }

    float gyroRollDegPerSec = m_simImu.gyro.x * 57.2957795f;
    float accelRollDeg = atan2f(m_simImu.accel.y, m_simImu.accel.z) * 57.2957795f;

    float gyroPitchDegPerSec = m_simImu.gyro.y * 57.2957795f;
    float accelPitchDeg = atan2f(
                                -m_simImu.accel.x,
                                sqrtf(m_simImu.accel.y * m_simImu.accel.y + m_simImu.accel.z * m_simImu.accel.z)
                            ) * 57.2957795f;

    if (!m_estimatorInitialized)
    {
        m_estimatedRollDeg = accelRollDeg;
        m_estimatedPitchDeg = accelPitchDeg;
        m_estimatorInitialized = true;
    }
    else
    {
        m_estimatedPitchDeg =
            0.98f * (m_estimatedPitchDeg + gyroPitchDegPerSec * dt)
          + 0.02f * accelPitchDeg;

        m_estimatedRollDeg =
            0.98f * (m_estimatedRollDeg + gyroRollDegPerSec * dt)
          + 0.02f * accelRollDeg;
    }

    float rollCorrection = PID_Controller::UpdateAngleWithGyroD(&m_rollPID, targetRollDeg, m_estimatedRollDeg, gyroRollDegPerSec, dt);
    rollCorrection = MathUtils::Clamp(rollCorrection, -0.25f, 0.25f);

    float pitchCorrection = PID_Controller::UpdateAngleWithGyroD(&m_pitchPID, targetPitchDeg, m_estimatedPitchDeg, gyroPitchDegPerSec, dt);
    pitchCorrection = MathUtils::Clamp(pitchCorrection, -0.25f, 0.25f);

    MotorOutputs motor_outputs{};
    motor_outputs.m1 = throttle + rollCorrection + pitchCorrection;
    motor_outputs.m2 = throttle - rollCorrection + pitchCorrection;
    motor_outputs.m3 = throttle - rollCorrection - pitchCorrection;
    motor_outputs.m4 = throttle + rollCorrection - pitchCorrection;

    motor_outputs.m1 = MathUtils::Clamp(motor_outputs.m1, 0.0f, 1.0f);
    motor_outputs.m2 = MathUtils::Clamp(motor_outputs.m2, 0.0f, 1.0f);
    motor_outputs.m3 = MathUtils::Clamp(motor_outputs.m3, 0.0f, 1.0f);
    motor_outputs.m4 = MathUtils::Clamp(motor_outputs.m4, 0.0f, 1.0f);

    SendServoOutputRaw(motor_outputs);
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
