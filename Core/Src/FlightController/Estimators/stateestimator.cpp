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
    : m_state()
    , m_ahrs()
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

    const bool updated = m_ahrs.Update(imuSample);

    if (!updated)
    {
        m_state.valid = false;
        return false;
    }

    const EulerAngles euler = m_ahrs.GetEuler();

    m_state.rollRad = euler.rollRad;
    m_state.pitchRad = euler.pitchRad;
    m_state.yawRad = euler.yawRad;

    m_state.gyroBias = m_ahrs.GetGyroBiasRadS();

    /*
        Для rate controller краще поки залишити raw gyro.
        AHRS attitude фільтрує roll/pitch/yaw,
        але rate loop має працювати по максимально швидкому gyro.
    */
    m_state.rollRateRadS = imuSample.gyro_rads.x;
    m_state.pitchRateRadS = imuSample.gyro_rads.y;
    m_state.yawRateRadS = imuSample.gyro_rads.z;

    m_state.imuDt = m_ahrs.GetLastDt();
    m_state.timestampUs = imuSample.timestampUs;
    m_state.valid = true;
    m_state.ahrsValid = m_ahrs.IsValid();

    return true;
}

const VehicleState& StateEstimator::GetState() const
{
    return m_state;
}

void StateEstimator::Reset()
{
    m_state = {};
    m_ahrs.Reset();
}