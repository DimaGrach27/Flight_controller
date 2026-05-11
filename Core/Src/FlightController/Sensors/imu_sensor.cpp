//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Sensors/imu_sensor.h"

namespace
{
    constexpr float MaxReasonableAccel_mps2 = 200.0f;
    constexpr float MaxReasonableGyro_rads = 40.0f;
}

Imu_Sensor::Imu_Sensor(Imu_Driver& driver)
    : m_driver(driver)
{

}

Imu_Sensor::~Imu_Sensor()
{

}

bool Imu_Sensor::Init()
{
    m_data = {};

    if (!m_driver.Init())
    {
        m_initialized = false;
        return false;
    }

    m_initialized = true;
    return true;
}

bool Imu_Sensor::Update(uint32_t nowUs)
{
    if (!m_initialized)
    {
        m_data.valid = false;
        return false;
    }

    ImuData sample{};

    if (!m_driver.Read(sample, nowUs))
    {
        m_data.valid = false;
        return false;
    }

    if (!ValidateSample(sample))
    {
        m_data.valid = false;
        return false;
    }

    m_data = ApplyCalibration(sample);
    m_data.valid = true;

    return true;
}

const ImuData & Imu_Sensor::GetData() const
{
    return m_data;
}

void Imu_Sensor::SetGyroOffset(float x_rads, float y_rads, float z_rads)
{
    m_gyroOffsetX_rads = x_rads;
    m_gyroOffsetY_rads = y_rads;
    m_gyroOffsetZ_rads = z_rads;
}

void Imu_Sensor::SetAccelOffset(float x_mps2, float y_mps2, float z_mps2)
{
    m_accelOffsetX_mps2 = x_mps2;
    m_accelOffsetY_mps2 = y_mps2;
    m_accelOffsetZ_mps2 = z_mps2;
}

ImuData Imu_Sensor::ApplyCalibration(const ImuData &sample) const
{
    ImuData calibrated = sample;

    calibrated.accel_mps2.x -= m_accelOffsetX_mps2;
    calibrated.accel_mps2.y -= m_accelOffsetY_mps2;
    calibrated.accel_mps2.z -= m_accelOffsetZ_mps2;

    calibrated.gyro_rads.x -= m_gyroOffsetX_rads;
    calibrated.gyro_rads.y -= m_gyroOffsetY_rads;
    calibrated.gyro_rads.z -= m_gyroOffsetZ_rads;

    calibrated.valid = sample.valid;

    return calibrated;
}

bool Imu_Sensor::ValidateSample(const ImuData& sample) const
{
    if (!sample.valid)
    {
        return false;
    }

    if (sample.accel_mps2.x > MaxReasonableAccel_mps2 ||
        sample.accel_mps2.x < -MaxReasonableAccel_mps2)
    {
        return false;
    }

    if (sample.accel_mps2.y > MaxReasonableAccel_mps2 ||
        sample.accel_mps2.y < -MaxReasonableAccel_mps2)
    {
        return false;
    }

    if (sample.accel_mps2.z > MaxReasonableAccel_mps2 ||
        sample.accel_mps2.z < -MaxReasonableAccel_mps2)
    {
        return false;
    }

    if (sample.gyro_rads.x > MaxReasonableGyro_rads ||
        sample.gyro_rads.x < -MaxReasonableGyro_rads)
    {
        return false;
    }

    if (sample.gyro_rads.y > MaxReasonableGyro_rads ||
        sample.gyro_rads.y < -MaxReasonableGyro_rads)
    {
        return false;
    }

    if (sample.gyro_rads.z > MaxReasonableGyro_rads ||
        sample.gyro_rads.z < -MaxReasonableGyro_rads)
    {
        return false;
    }

    return true;
}