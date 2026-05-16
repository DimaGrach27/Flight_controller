//
// Created by Dmytro Hrachov on 15.05.2026.
//
#include "FlightController/Estimators/ahrs.h"

#include "FlightController/Utils/mathutils.h"
#include "FlightController/Math/vector3f.h"

namespace
{
    constexpr float Epsilon = 1.0e-6f;
}

Ahrs::Ahrs()
{

}

void Ahrs::Init()
{
    Reset();
}

void Ahrs::Reset()
{
    m_q = Quaternion{};
    m_gyroBiasRadS = {};
    m_lastTimestampUs = 0;
    m_hasTimestamp = false;
    m_lastDt = 0.0f;
    m_valid = false;
    m_initialized = false;
}

bool Ahrs::Update(const ImuSample& imuSample)
{
    if (!imuSample.valid)
    {
        m_valid = false;
        return false;
    }

    const float dt = ComputeDt(imuSample.timestampUs);
    if (dt <= 0.0f)
    {
        return false;
    }

    Vector3f correctedGyro = imuSample.gyro_rads - m_gyroBiasRadS;

    if (IsAccelUsable(imuSample.accel_mps2))
    {
        const Vector3f accelNormalized = imuSample.accel_mps2.Normalized();
        const Vector3f errorBody = ComputeGravityErrorBody(accelNormalized);

        correctedGyro = correctedGyro + errorBody * m_config.kp;

        if (m_config.ki > 0.0f)
        {
            m_gyroBiasRadS = m_gyroBiasRadS - errorBody * m_config.ki * dt;
        }
    }

    IntegrateGyro(correctedGyro, dt);

    m_lastDt = dt;
    m_valid = true;

    return true;
}

const Quaternion& Ahrs::GetQuaternion() const
{
    return m_q;
}

EulerAngles Ahrs::GetEuler() const
{
    return m_q.ToEuler();
}

Vector3f Ahrs::GetGyroBiasRadS() const
{
    return m_gyroBiasRadS;
}

float Ahrs::GetLastDt() const
{
    return m_lastDt;
}

bool Ahrs::IsValid() const
{
    return m_valid;
}

float Ahrs::ComputeDt(uint32_t timestampUs)
{
    if (!m_hasTimestamp)
    {
        m_lastTimestampUs = timestampUs;
        m_hasTimestamp = true;
        return 0.0f;
    }

    const uint32_t deltaUs = timestampUs - m_lastTimestampUs;
    m_lastTimestampUs = timestampUs;

    const float dt = static_cast<float>(deltaUs) / 1000000.0f;

    if (dt < m_config.minDt)
    {
        return 0.0f;
    }

    if (dt > m_config.maxDt)
    {
        return 0.0f;
    }

    return MathUtils::Clamp(dt, m_config.minDt, m_config.maxDt);
}

bool Ahrs::IsAccelUsable(const Vector3f& accel) const
{
    const float accelMagnitude = accel.Length();

    if (accelMagnitude < Epsilon)
    {
        return false;
    }

    const float minAccel =
        m_config.gravityMagnitude * (1.0f - m_config.accelMagnitudeTolerance);

    const float maxAccel =
        m_config.gravityMagnitude * (1.0f + m_config.accelMagnitudeTolerance);

    return accelMagnitude >= minAccel && accelMagnitude <= maxAccel;
}

Vector3f Ahrs::ComputeGravityErrorBody(const Vector3f& accelBodyNormalized) const
{
    /*
        ВАЖЛИВО:
        Це залежить від твоєї системи осей.

        Тут припускаємо:
        - accel_mps2 на столі має приблизно +Z = +9.81
        - world gravity direction = +Z
    */
    const Vector3f gravityWorld = {0.0f, 0.0f, 1.0f};

    const Vector3f estimatedGravityBody = m_q.RotateWorldToBody(gravityWorld);

    return Vector3f::Cross(accelBodyNormalized, estimatedGravityBody);
}

void Ahrs::IntegrateGyro(const Vector3f& gyroRadS, float dt)
{
    const Quaternion omega(
        0.0f,
        gyroRadS.x,
        gyroRadS.y,
        gyroRadS.z
    );

    const Quaternion qDot = m_q * omega;

    m_q.w += 0.5f * qDot.w * dt;
    m_q.x += 0.5f * qDot.x * dt;
    m_q.y += 0.5f * qDot.y * dt;
    m_q.z += 0.5f * qDot.z * dt;

    m_q.Normalize();
}