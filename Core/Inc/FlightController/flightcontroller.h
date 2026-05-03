//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <cstdint>

#include "main.h"

#include "mavlink/common/mavlink.h"
#include "structs.h"
#include "PID.h"

enum class FlightMode
{
    FLIGHT_MODE_ANGLE = 0,
    FLIGHT_MODE_ACRO = 1,
};

class FlightController
{
public:
    FlightController();
    ~FlightController() = default;

    void Init(UART_HandleTypeDef& huart2);
    void Update(float dt);
    void Heartbeat();
    void MavlinkParseByte(uint8_t byte);

private:
    void MavlinkHandleMessage(const mavlink_message_t* msg);
    void HandleHilSensor(const mavlink_message_t* msg);
    void SendServoOutputRaw(MotorOutputs motor_outputs);
    ControlOutput UpdateAngleController(float dt);
    ControlOutput UpdateAcroController(float dt);
    MotorOutputs MixQuadX(const float throttle, const ControlOutput& control_output);

private:
    UART_HandleTypeDef* m_huart2 = nullptr;

    RcCommand m_rcCommand = {};
    SimImuSample m_simImu = {};

    PID m_rollPID = {};
    PID m_pitchPID = {};
    PID m_yawPID = {};

    FlightMode m_flightMode = FlightMode::FLIGHT_MODE_ANGLE;

    bool m_armed = false;

    bool m_estimatorInitialized = false;
    float m_estimatedRollDeg = 0.0f;
    float m_estimatedPitchDeg = 0.0f;
};
