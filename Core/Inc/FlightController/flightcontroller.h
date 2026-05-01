//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <cstdint>

#include "main.h"

#include "mavlink/common/mavlink.h"
#include "structs.h"
#include "PID.h"

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

private:
    UART_HandleTypeDef* m_huart2 = nullptr;

    SimImuSample m_simImu = {};
    PID m_rollPID = {};
    PID m_pitchPID = {};

    bool m_estimatorInitialized = false;
};
