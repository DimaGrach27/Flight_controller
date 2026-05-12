//
// Created by Dmytro Hrachov on 12.05.2026.
//

#include "FlightController/Sensors/IMU/imucalibarator.h"

ImuCalibrator::ImuCalibrator()
{
}

void ImuCalibrator::Init()
{
    m_calibration = {};

    ResetGyroAccumulation();
    ResetAccelAccumulation();

    m_gyroCalibrationRunning = false;
    m_accelCalibrationRunning = false;
}

void ImuCalibrator::StartGyroCalibration(uint16_t sampleCount)
{
    ResetGyroAccumulation();

    m_gyroSampleTarget = sampleCount;
    m_gyroCalibrationRunning = true;

    m_calibration.gyroCalibrated = false;
}

void ImuCalibrator::StartLevelAccelCalibration(
    uint16_t sampleCount,
    float expectedAccelZ_mps2
)
{
    ResetAccelAccumulation();

    m_accelSampleTarget = sampleCount;
    m_expectedAccelZ_mps2 = expectedAccelZ_mps2;
    m_accelCalibrationRunning = true;

    m_calibration.accelCalibrated = false;
}

void ImuCalibrator::Update(const ImuSample& sample)
{
    if (!sample.valid)
    {
        return;
    }

    if (m_gyroCalibrationRunning)
    {
        m_gyroSum.x += sample.gyro_rads.x;
        m_gyroSum.y += sample.gyro_rads.y;
        m_gyroSum.z += sample.gyro_rads.z;

        ++m_gyroSampleCount;

        if (m_gyroSampleCount >= m_gyroSampleTarget)
        {
            const float invCount = 1.0f / static_cast<float>(m_gyroSampleCount);

            m_calibration.gyroBias_rads.x = m_gyroSum.x * invCount;
            m_calibration.gyroBias_rads.y = m_gyroSum.y * invCount;
            m_calibration.gyroBias_rads.z = m_gyroSum.z * invCount;

            m_calibration.gyroCalibrated = true;
            m_gyroCalibrationRunning = false;
        }
    }

    if (m_accelCalibrationRunning)
    {
        m_accelSum.x += sample.accel_mps2.x;
        m_accelSum.y += sample.accel_mps2.y;
        m_accelSum.z += sample.accel_mps2.z;

        ++m_accelSampleCount;

        if (m_accelSampleCount >= m_accelSampleTarget)
        {
            const float invCount = 1.0f / static_cast<float>(m_accelSampleCount);

            const float avgX = m_accelSum.x * invCount;
            const float avgY = m_accelSum.y * invCount;
            const float avgZ = m_accelSum.z * invCount;

            /*
                Level calibration:
                хочемо, щоб після калібровки було:
                accel.x = 0
                accel.y = 0
                accel.z = expectedAccelZ_mps2

                Тому bias:
                bias.x = avgX - 0
                bias.y = avgY - 0
                bias.z = avgZ - expectedZ
            */
            m_calibration.accelBias_mps2.x = avgX;
            m_calibration.accelBias_mps2.y = avgY;
            m_calibration.accelBias_mps2.z = avgZ - m_expectedAccelZ_mps2;

            m_calibration.accelCalibrated = true;
            m_accelCalibrationRunning = false;
        }
    }
}

bool ImuCalibrator::IsGyroCalibrationRunning() const
{
    return m_gyroCalibrationRunning;
}

bool ImuCalibrator::IsAccelCalibrationRunning() const
{
    return m_accelCalibrationRunning;
}

bool ImuCalibrator::IsGyroCalibrated() const
{
    return m_calibration.gyroCalibrated;
}

bool ImuCalibrator::IsAccelCalibrated() const
{
    return m_calibration.accelCalibrated;
}

const ImuCalibrationData& ImuCalibrator::GetCalibrationData() const
{
    return m_calibration;
}

void ImuCalibrator::ResetGyroAccumulation()
{
    m_gyroSum = {};
    m_gyroSampleCount = 0;
    m_gyroSampleTarget = 0;
}

void ImuCalibrator::ResetAccelAccumulation()
{
    m_accelSum = {};
    m_accelSampleCount = 0;
    m_accelSampleTarget = 0;
}