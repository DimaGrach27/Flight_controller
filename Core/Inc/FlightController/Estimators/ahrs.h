//
// Created by Dmytro Hrachov on 15.05.2026.
//
#pragma once

#include <cstdint>
#include "FlightController/Math/quaternion.h"
#include "FlightController/datastructs.h"

class Ahrs
{
public:
    struct Config
    {
        float kp = 0.8f;
        float ki = 0.00f;

        float gravityMagnitude = 9.80665f;
        float accelMagnitudeTolerance = 0.15f;

        float maxLimitedError = 0.15f;

        float minDt = 0.0001f;
        float maxDt = 0.02f;
    };

    Ahrs();

    void Init();
    void Reset();

    bool Update(const ImuSample& imuSample);

    const Quaternion& GetQuaternion() const;
    EulerAngles GetEuler() const;
    Vector3f GetGyroBiasRadS() const;
    float GetAccelWeight() const;

    float GetLastDt() const;
    bool IsValid() const;

private:
    float ComputeDt(uint32_t timestampUs);
    bool IsAccelUsable(const Vector3f& accel) const;
    float ComputeAccelWeight(const Vector3f& accel) const;

    Vector3f ComputeGravityErrorBody(const Vector3f& accelBodyNormalized) const;
    void IntegrateGyro(const Vector3f& gyroRadS, float dt);

private:
    Config m_config{};

    Quaternion m_q{};
    Vector3f m_gyroBiasRadS{};

    uint32_t m_lastTimestampUs = 0;
    bool m_hasTimestamp = false;

    float m_accelWeight = 0.0f;

    float m_lastDt = 0.0f;
    bool m_valid = false;
    bool m_initialized = false;
};