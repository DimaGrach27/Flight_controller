//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "../../../../Inc/FlightController/Sensors/IMU/imu_sensor.h"

#include "FlightController/Sensors/IMU/imuaxismapper.h"

namespace
{
    constexpr float MaxReasonableAccel_mps2 = 200.0f;
    constexpr float MaxReasonableGyro_rads = 40.0f;

    constexpr float GyroLowPassCutoffHz = 60.0f;
    constexpr float AccelLowPassCutoffHz = 30.0f;

    constexpr float MinDtSeconds = 0.000001f;
    constexpr float MaxDtSeconds = 0.05f;
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

    m_calibrator.Init();

    m_gyroFilter.Init(GyroLowPassCutoffHz);
    m_accelFilter.Init(AccelLowPassCutoffHz);

    m_lastFilterUpdateUs = 0;
    m_hasLastFilterUpdate = false;

    m_calibrator.StartGyroCalibration(1000);

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

    ImuSample sample{};

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

    m_calibrator.Update(sample);

    ImuSample calibrated = ApplyCalibration(sample);

    if (m_enableFiltering)
    {
        calibrated = ApplyFiltering(calibrated);
    }

    ImuAxisMapper axismapper = ImuAxisMapper();

    calibrated.accel_mps2 = axismapper.MapAccel(calibrated.accel_mps2);
    calibrated.gyro_rads = axismapper.MapGyro(calibrated.gyro_rads);

    calibrated.valid = true;
    m_data = calibrated;

    return true;
}

const ImuSample & Imu_Sensor::GetData() const
{
    return m_data;
}

void Imu_Sensor::StartGyroCalibration(uint16_t sampleCount)
{
    m_calibrator.StartGyroCalibration(sampleCount);
}

void Imu_Sensor::StartLevelAccelCalibration(uint16_t sampleCount, float expectedAccelZ_mps2)
{
    m_calibrator.StartLevelAccelCalibration(sampleCount, expectedAccelZ_mps2);
}

bool Imu_Sensor::IsGyroCalibrated() const
{
    return m_calibrator.IsGyroCalibrated();
}

bool Imu_Sensor::IsAccelCalibrated() const
{
    return m_calibrator.IsAccelCalibrated();
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

ImuSample Imu_Sensor::ApplyCalibration(const ImuSample &sample) const
{
    ImuSample calibrated = sample;

    const ImuCalibrationData& calibration = m_calibrator.GetCalibrationData();

    calibrated.gyro_rads.x -= m_gyroOffsetX_rads;
    calibrated.gyro_rads.y -= m_gyroOffsetY_rads;
    calibrated.gyro_rads.z -= m_gyroOffsetZ_rads;

    if (calibration.gyroCalibrated)
    {
        calibrated.gyro_rads.x -= calibration.gyroBias_rads.x;
        calibrated.gyro_rads.y -= calibration.gyroBias_rads.y;
        calibrated.gyro_rads.z -= calibration.gyroBias_rads.z;
    }

    calibrated.accel_mps2.x -= m_accelOffsetX_mps2;
    calibrated.accel_mps2.y -= m_accelOffsetY_mps2;
    calibrated.accel_mps2.z -= m_accelOffsetZ_mps2;

    if (calibration.accelCalibrated)
    {
        calibrated.accel_mps2.x -= calibration.accelBias_mps2.x;
        calibrated.accel_mps2.y -= calibration.accelBias_mps2.y;
        calibrated.accel_mps2.z -= calibration.accelBias_mps2.z;
    }

    calibrated.valid = sample.valid;

    return calibrated;
}

ImuSample Imu_Sensor::ApplyFiltering(const ImuSample &sample)
{
    const float dt = ComputeDtSeconds(sample.timestampUs);

    if (dt <= 0.0f)
    {
        return sample;
    }

    ImuSample filtered = sample;

    filtered.gyro_rads = m_gyroFilter.Update(sample.gyro_rads, dt);
    filtered.accel_mps2 = m_accelFilter.Update(sample.accel_mps2, dt);

    filtered.valid = sample.valid;

    return filtered;
}

bool Imu_Sensor::ValidateSample(const ImuSample& sample) const
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

float Imu_Sensor::ComputeDtSeconds(uint32_t timestampUs)
{
    if (!m_hasLastFilterUpdate)
    {
        m_lastFilterUpdateUs = timestampUs;
        m_hasLastFilterUpdate = true;
        return 0.0f;
    }

    const uint32_t dtUs = timestampUs - m_lastFilterUpdateUs;
    const float dt = static_cast<float>(dtUs) / 1000000.0f;

    m_lastFilterUpdateUs = timestampUs;

    if (dt < MinDtSeconds)
    {
        return 0.0f;
    }

    if (dt > MaxDtSeconds)
    {
        return 0.0f;
    }

    return dt;
}
