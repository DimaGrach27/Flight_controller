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
    void UpdateFormNewImuSample();
    void Heartbeat();
    void MavlinkParseByte(uint8_t byte);

private:
    void MavlinkHandleMessage(const mavlink_message_t* msg);
    void HandleHilSensor(const mavlink_message_t* msg);
    void SendServoOutputRaw(MotorOutputs motor_outputs);

    float ApplyDeadband(float input, float deadband);

    ControlOutput UpdateAngleController(float dt);
    ControlOutput UpdateAcroController(float dt);
    void UpdateAttitudeEstimator(float dt);
    MotorOutputs MixQuadX(const float throttle, const ControlOutput& control_output);
    MotorOutputs DesaturateMotors(MotorOutputs motor_outputs);

    // void SendAcroDebug(float targetRollRateDegSec, float targetPitchRateDegSec, float targetYawRateDegSec,
    //                    float gyroRollDegPerSec, float gyroPitchDegPerSec, float gyroYawDegPerSec,
    //                    // float accelRollDeg, float accelPitchDeg, float throttleAuthority,
    //                    ControlOutput control_output, float throttle, MotorOutputs motors);

    void ResetRatePidState();
    void CalibrateGyroBias();

    float FilterGyroRollForDebug(float gyroRollDegSec);

    void SendFlightLogCsv(const FlightLogSample& sample);

    float GetImuDtSec();

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

    uint32_t m_lastDebugMs = 0;

    float m_lastGoodGyroRollDegSec = 0.0f;

    float m_filteredGyroRollDegPerSec = 0.0f;
    float m_filteredGyroPitchDegPerSec = 0.0f;
    float m_filteredGyroYawDegPerSec = 0.0f;
    bool m_gyroFilterInitialized = false;

    uint32_t m_imuSequence = 0;
    uint32_t m_lastProcessedImuSequence = 0;
    uint64_t m_lastImuTimeUsec = 0;

    const float m_idleArmedThrottle = 0.08f;
    const float m_idleThrottleThreshold = 0.05f;

    Vector3 m_gyroBias = {};
    bool m_gyroBiasReady = false;

    ControlDebug m_lastControlDebug{};

    uint64_t m_previousImuTimeUsec = 0;
    uint32_t m_controlSequence = 0;
    uint32_t m_logSequence = 0;
    uint32_t m_previousHalLogMs = 0;
};
