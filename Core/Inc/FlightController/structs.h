//
// Created by Dmytro Hrachov on 01.05.2026.
//
#pragma once

#include <cstdint>
struct MotorOutputs
{
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
};

struct ControlOutput
{
    float roll;
    float pitch;
    float yaw;
};

struct PidGains
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

//DEBUG

struct FlightLogSample
{
    float flightMode = 0; //1 = acro; 0 = angle
    float armed = 0; //1 = yes; 0 = no
    uint32_t timeMs = 0;
    uint32_t imuSeq = 0;
    uint32_t controlSeq = 0;
    uint32_t logSeq = 0;

    float dt = 0.0f;
    float imuDt = 0.0f;
    float halDt = 0.0f;

    float rcThrottle = 0.0f;
    float rcRoll = 0.0f;
    float rcPitch = 0.0f;
    float rcYaw = 0.0f;

    float targetRollRateRadSec = 0.0f;
    float targetPitchRateRadSec = 0.0f;
    float targetYawRateRadSec = 0.0f;

    float gyroRollRadSec = 0.0f;
    float gyroPitchRadSec = 0.0f;
    float gyroYawRadSec = 0.0f;

    float gyroMagnitude = 0.0f;

    float accelRoll = 0.0f;
    float accelPitch = 0.0f;
    float accelYaw = 0.0f;

    float correctedRoll = 0.0f;
    float correctedPitch = 0.0f;
    float correctedYaw = 0.0f;

    float angleErrorRoll = 0.0f;
    float angleErrorPitch = 0.0f;
    float angleErrorYaw = 0.0f;

    float estimatedRollRad = 0.0f;
    float estimatedPitchRad = 0.0f;
    float estimatedYawRad = 0.0f;

    float gyroBiasX = 0.0f;
    float gyroBiasY = 0.0f;
    float gyroBiasZ = 0.0f;

    float accelMagnitude = 0.0f;
    float accelWeight = 0.0f;
    float ahrsValid = 0.0f;

    float controlRoll = 0.0f;
    float controlPitch = 0.0f;
    float controlYaw = 0.0f;

    float motorM1 = 0.0f;
    float motorM2 = 0.0f;
    float motorM3 = 0.0f;
    float motorM4 = 0.0f;

    float PID_P_roll = 0.0f;
    float PID_I_roll = 0.0f;
    float PID_D_roll = 0.0f;

    float PID_P_pitch = 0.0f;
    float PID_I_pitch = 0.0f;
    float PID_D_pitch = 0.0f;

    float PID_P_yaw = 0.0f;
    float PID_I_yaw = 0.0f;
    float PID_D_yaw = 0.0f;
};

struct ControlDebug
{
    float targetRollRateDegSec = 0.0f;
    float targetPitchRateDegSec = 0.0f;
    float targetYawRateDegSec = 0.0f;

    float gyroRollDegSec = 0.0f;
    float gyroPitchDegSec = 0.0f;
    float gyroYawDegSec = 0.0f;
};