//
// Created by Dmytro Hrachov on 29.04.2026.
//
#include "FlightController/flight_controller.h"

#include <math.h>
#include <stdio.h>
#include "main.h"

#include "mavlink/common/mavlink.h"

#include "FlightController/helpers.h"
#include "FlightController/pid.h"

extern UART_HandleTypeDef huart2;

void TestMavlinkCompile(void)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        1,
        1,
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0,
        0,
        MAV_STATE_STANDBY
    );

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    (void)len;
}

void update_flight_controller(void)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(3000);
}

// Stable 1-axis HIL preset:
// - smooth convergence to 0 deg
// - no overshoot
// - tested with Python sim dt=0.01
// static PID rollPid = {
//     .kp = 0.008f,
//     .ki = 0.0f,
//     .kd = 0.0045f,
//     .integrator = 0.0f,
//     .previousError = 0.0f,
//     .integratorLimit = 50.0f
// };

void flight_Update(float dt)
{
    static PID rollPid = {
        .kp = 0.008f,
        .ki = 0.0f,
        .kd = 0.0045f,
        .integrator = 0.0f,
        .previousError = 0.0f,
        .integratorLimit = 50.0f
    };

    if (!g_simImu.valid)
        return;

    static bool estimatorInitialized = false;
    static float estimatedRollDeg = 0.0f;

    float targetRollDeg = 0.0f;

    float gyroRollDegPerSec = g_simImu.gyro.x * 57.2957795f;

    float accelRollDeg = atan2f(g_simImu.accel.y, g_simImu.accel.z) * 57.2957795f;

    if (!estimatorInitialized)
    {
        estimatedRollDeg = accelRollDeg;
        estimatorInitialized = true;
    }
    else
    {
        estimatedRollDeg =
            0.98f * (estimatedRollDeg + gyroRollDegPerSec * dt)
          + 0.02f * accelRollDeg;
    }

    float correction = PID_UpdateAngleWithGyroD(&rollPid, targetRollDeg, estimatedRollDeg, gyroRollDegPerSec, dt);
    // float correction = PID_Update(&rollPid, targetRollDeg, estimatedRollDeg, dt);

    correction = clamp(correction, -0.25f, 0.25f);

    float throttle = 0.5f;

    float leftMotor = throttle - correction;
    float rightMotor = throttle + correction;

    leftMotor = clamp(leftMotor, 0.0f, 1.0f);
    rightMotor = clamp(rightMotor, 0.0f, 1.0f);

    SendServoOutputRaw(leftMotor, rightMotor);
}

static uint32_t lastHeartbeatMs = 0;

void heartbeat(void)
{
    // const uint32_t now = HAL_GetTick();

    // if (now - lastHeartbeatMs >= 1000)
    // {
        // lastHeartbeatMs = now;

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

        HAL_UART_Transmit(&huart2, buffer, len, 100);
    // }
}

void MavlinkParseByte(uint8_t byte)
{
    static mavlink_message_t msg;
    static mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
    {
        MavlinkHandleMessage(&msg);
    }
}

void MavlinkHandleMessage(const mavlink_message_t* msg)
{
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HIL_SENSOR:
            HandleHilSensor(msg);
            break;

        default:
            break;
    }
}

void HandleHilSensor(const mavlink_message_t* msg)
{
    mavlink_hil_sensor_t sensor;
    mavlink_msg_hil_sensor_decode(msg, &sensor);

    g_simImu.accel.x = sensor.xacc;
    g_simImu.accel.y = sensor.yacc;
    g_simImu.accel.z = sensor.zacc;

    g_simImu.gyro.x = sensor.xgyro;
    g_simImu.gyro.y = sensor.ygyro;
    g_simImu.gyro.z = sensor.zgyro;

    g_simImu.valid = true;
}

void SendServoOutputRaw(float leftMotor, float rightMotor)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    uint16_t leftPwm = (uint16_t)(1000.0f + leftMotor * 1000.0f);
    uint16_t rightPwm = (uint16_t)(1000.0f + rightMotor * 1000.0f);

    mavlink_msg_servo_output_raw_pack(
        1,
        1,
        &msg,
        HAL_GetTick() * 1000ULL,
        0,
        leftPwm,
        rightPwm,
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
        0,
        0,
        0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    HAL_UART_Transmit(&huart2, buffer, len, 100);
}