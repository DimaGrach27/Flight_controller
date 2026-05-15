//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include <cstdint>

#include "structs.h"

struct ImuRawData
{
    int16_t rawGyroX = 0;
    int16_t rawGyroY = 0;
    int16_t rawGyroZ = 0;

    int16_t rawAccelX = 0;
    int16_t rawAccelY = 0;
    int16_t rawAccelZ = 0;

    int16_t temperature = 0;

    uint32_t timestampUs = 0;
    bool valid = false;
};

struct ImuSample
{
    Vector3 accel_mps2 = {};
    Vector3 gyro_rads = {};

    float temperature_C = 0.0f;

    uint32_t timestampUs = 0;
    bool valid = false;
};

struct BatteryData
{
    float voltage_V = 0.0f;
    float current_A = 0.0f;
    float percentage = 0.0f;

    bool lowVoltage = false;
    bool criticalVoltage = false;

    uint32_t timestampUs = 0;
    bool valid = false;
};

struct RcCommand
{
    float throttle = 0.0f; // 0..1

    float roll = 0.0f;    // -1..1
    float pitch = 0.0f;   // -1..1
    float yaw = 0.0f;     // -1..1

    bool armSwitch = false;
    bool angleModeSwitch = false;
    bool acroModeSwitch = false;
    bool failsafe = false;

    uint32_t timestampUs = 0;
    bool valid = false;
};

struct VehicleState
{
    float rollRad = 0.0f;
    float pitchRad = 0.0f;
    float yawRad = 0.0f;

    float rollRateRadS = 0.0f;
    float pitchRateRadS = 0.0f;
    float yawRateRadS = 0.0f;

    float altitudeM = 0.0f;
    float verticalVelocityMS = 0.0f;

    float imuDt = 0.0f;

    uint32_t timestampUs = 0;
    bool valid = false;
};

