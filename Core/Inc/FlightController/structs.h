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
    float m1;
    float m2;
    float m3;
    float m4;
};

struct RcCommand
{
    float throttle; // 0..1
    float roll;     // -1..1
    float pitch;    // -1..1
    float yaw;      // -1..1
    bool armed;
    bool acroMode;
    bool valid;
};

struct ControlOutput
{
    float roll;
    float pitch;
    float yaw;
};


//DEBUG

struct FlightLogSample
{
    uint32_t timeMs = 0;
    uint32_t imuSeq = 0;

    float dt = 0.0f;
    float imuDt = 0.0f;

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