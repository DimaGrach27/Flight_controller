//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Estimators/stateestimator.h"

#include <cmath>

namespace
{
    constexpr float MinDtSeconds = 0.000001f;
    constexpr float MaxDtSeconds = 0.05f;
}

StateEstimator::StateEstimator()
{
}

void StateEstimator::Init()
{
    Reset();
}

bool StateEstimator::UpdateImu(const ImuSample& imuSample)
{
    if (!imuSample.valid)
    {
        m_state.valid = false;
        return false;
    }

    if (!m_initialized)
    {
        InitializeFromAccel(imuSample);

        m_state.rollRateRadS = imuSample.gyro_rads.x;
        m_state.pitchRateRadS = imuSample.gyro_rads.y;
        m_state.yawRateRadS = imuSample.gyro_rads.z;

        m_state.timestampUs = imuSample.timestampUs;
        m_state.valid = true;

        m_lastUpdateUs = imuSample.timestampUs;
        m_hasLastUpdate = true;
        m_initialized = true;

        return true;
    }

    const float dt = ComputeDtSeconds(imuSample.timestampUs);

    if (dt <= 0.0f)
    {
        m_state.valid = false;
        return false;
    }

    const float accelRollRad = ComputeAccelRollRad(imuSample);
    const float accelPitchRad = ComputeAccelPitchRad(imuSample);

    const float gyroRollRad = m_state.rollRad + imuSample.gyro_rads.x * dt;
    const float gyroPitchRad = m_state.pitchRad + imuSample.gyro_rads.y * dt;
    const float gyroYawRad = m_state.yawRad + imuSample.gyro_rads.z * dt;

    m_state.rollRad =
        m_alpha * gyroRollRad +
        (1.0f - m_alpha) * accelRollRad;

    m_state.pitchRad =
        m_alpha * gyroPitchRad +
        (1.0f - m_alpha) * accelPitchRad;

    /*
        Yaw з accel не виправляється.

        Без магнітометра або зовнішнього reference yaw буде дрейфувати.
        Для першої версії це нормально.
    */
    m_state.yawRad = gyroYawRad;

    m_state.rollRateRadS = imuSample.gyro_rads.x;
    m_state.pitchRateRadS = imuSample.gyro_rads.y;
    m_state.yawRateRadS = imuSample.gyro_rads.z;

    m_state.timestampUs = imuSample.timestampUs;
    m_state.valid = true;

    m_lastUpdateUs = imuSample.timestampUs;
    m_hasLastUpdate = true;

    return true;
}

const VehicleState& StateEstimator::GetState() const
{
    return m_state;
}

void StateEstimator::Reset()
{
    m_state = {};

    m_lastUpdateUs = 0;

    m_initialized = false;
    m_hasLastUpdate = false;
}

float StateEstimator::ComputeDtSeconds(uint32_t timestampUs)
{
    if (!m_hasLastUpdate)
    {
        m_lastUpdateUs = timestampUs;
        m_hasLastUpdate = true;
        return 0.0f;
    }

    const uint32_t dtUs = timestampUs - m_lastUpdateUs;
    const float dtSeconds = static_cast<float>(dtUs) / 1000000.0f;

    if (dtSeconds < MinDtSeconds)
    {
        return 0.0f;
    }

    if (dtSeconds > MaxDtSeconds)
    {
        return 0.0f;
    }

    return dtSeconds;
}

void StateEstimator::InitializeFromAccel(const ImuSample& imuSample)
{
    m_state.rollRad = ComputeAccelRollRad(imuSample);
    m_state.pitchRad = ComputeAccelPitchRad(imuSample);
    m_state.yawRad = 0.0f;
}

float StateEstimator::ComputeAccelRollRad(const ImuSample& imuSample) const
{
    return std::atan2(
        imuSample.accel_mps2.y,
        imuSample.accel_mps2.z
    );
}

float StateEstimator::ComputeAccelPitchRad(const ImuSample& imuSample) const
{
    return std::atan2(
        -imuSample.accel_mps2.x,
        std::sqrt(
            imuSample.accel_mps2.y * imuSample.accel_mps2.y +
            imuSample.accel_mps2.z * imuSample.accel_mps2.z
        )
    );
}