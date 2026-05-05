//
// Created by Dmytro Hrachov on 01.05.2026.
//
#pragma once

struct Vector3
{
    float x;
    float y;
    float z;
};

struct SimImuSample
{
    Vector3 gyro;
    Vector3 accel;
    bool valid;
};

struct MotorOutputs
{
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
};

struct RcCommand
{
    uint16_t throttle; // 0..1000
    int16_t roll;     // -1000..1000
    int16_t pitch;    // -1000..1000
    int16_t yaw;      // -1000..1000
    bool armed;
    bool acroMode;
    bool valid;
};

struct ControlOutput
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
};


//DEBUG

struct FlightLogSample
{
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

    float targetRollRateDegSec = 0.0f;
    float targetPitchRateDegSec = 0.0f;
    float targetYawRateDegSec = 0.0f;

    float gyroRollDegSec = 0.0f;
    float gyroPitchDegSec = 0.0f;
    float gyroYawDegSec = 0.0f;

    float estimatedRollDeg = 0.0f;
    float estimatedPitchDeg = 0.0f;

    float controlRoll = 0.0f;
    float controlPitch = 0.0f;
    float controlYaw = 0.0f;

    float motorM1 = 0.0f;
    float motorM2 = 0.0f;
    float motorM3 = 0.0f;
    float motorM4 = 0.0f;
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