//
// Created by Dmytro Hrachov on 15.05.2026.
//
#include "FlightController/Estimators/ahrs.h"

#include "FlightController/Utils/mathutils.h"
#include "FlightController/Math/vector3f.h"

#include <cmath>

namespace
{
    constexpr float Epsilon = 1.0e-6f;

    static Vector3f LimitVector(const Vector3f& value, float maxLength)
    {
        const float length = value.Length();

        if (length <= maxLength || length < 1.0e-6f)
        {
            return value;
        }

        return value * (maxLength / length);
    }
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
    m_accelWeight = 0.0f;
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

    if (!m_initialized)
    {
        if (!InitFromAccel(imuSample.accel_mps2))
        {
            m_valid = false;
            return false;
        }

        m_initialized = true;
        m_valid = true;
        return true;
    }

    Vector3f correctedGyro = imuSample.gyro_rads - m_gyroBiasRadS;
    m_accelWeight = ComputeAccelWeight(imuSample.accel_mps2);

    if (m_accelWeight > 0.0f)
    {
        const Vector3f accelNormalized = imuSample.accel_mps2.Normalized();
        const Vector3f errorBody = ComputeGravityErrorBody(accelNormalized);
        const Vector3f limitedErrorBody = LimitVector(errorBody, m_config.maxLimitedError);

        correctedGyro = correctedGyro + limitedErrorBody * (m_config.kp * m_accelWeight);

        if (m_config.ki > 0.0f)
        {
            m_gyroBiasRadS = m_gyroBiasRadS - limitedErrorBody * (m_config.ki * m_accelWeight * dt);
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

float Ahrs::GetAccelWeight() const
{
    return m_accelWeight;
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

float Ahrs::ComputeAccelWeight(const Vector3f &accel) const
{
    const float accelMagnitude = accel.Length();

    if (accelMagnitude < Epsilon)
    {
        return 0.0f;
    }

    const float gravity = m_config.gravityMagnitude;
    const float error = std::fabs(accelMagnitude - gravity) / gravity;

    if (error >= m_config.accelMagnitudeTolerance)
    {
        return 0.0f;
    }

    const float magnitudeWeight = 1.0f - error / m_config.accelMagnitudeTolerance;

    const Vector3f accelNormalized = accel.Normalized();

    const Vector3f gravityWorld = {0.0f, 0.0f, 1.0f};
    const Vector3f estimatedGravityBody = m_q.RotateWorldToBody(gravityWorld);

    float dot = Vector3f::Dot(accelNormalized, estimatedGravityBody);
    dot = MathUtils::Clamp(dot, -1.0f, 1.0f);

    const float angleErrorRad = std::acos(dot);

    constexpr float MaxAccelAngleErrorRad = 0.45f; // ~26 deg

    if (angleErrorRad >= MaxAccelAngleErrorRad)
    {
        return 0.0f;
    }

    const float angleWeight = 1.0f - angleErrorRad / MaxAccelAngleErrorRad;

    return magnitudeWeight * angleWeight;
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

bool Ahrs::InitFromAccel(const Vector3f& accel)
{
    const float accelLength = accel.Length();

    if (accelLength < Epsilon)
    {
        return false;
    }

    constexpr float GravityForce = 9.8f;
    if (std::abs(GravityForce - accelLength) <= 0.2f)
    {
        return false;
    }

    const Vector3f accelNormalized = accel.Normalized();

    const float rollRad = std::atan2(
        accelNormalized.y,
        accelNormalized.z
    );

    const float pitchRad = std::atan2(
        -accelNormalized.x,
        std::sqrt(
            accelNormalized.y * accelNormalized.y +
            accelNormalized.z * accelNormalized.z
        )
    );

    constexpr float yawRad = 0.0f;

    m_q = Quaternion::FromEuler({rollRad, pitchRad, yawRad});
    m_q.Normalize();

    m_accelWeight = 1.0f;
    m_valid = true;
    m_initialized = true;

    return true;
}
