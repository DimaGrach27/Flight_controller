//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "../../../../Inc/FlightController/Sensors/IMU/imu_driver_hil.h"

namespace
{
    constexpr float G = 9.80665f;

    constexpr float AccelRawPerG = 16384.0f;
    constexpr float GyroRawPerDps = 131.0f;

    constexpr float RadToDeg = 57.2957795f;
}

IMU_Driver_Hil::IMU_Driver_Hil()
{
}

bool IMU_Driver_Hil::Init()
{
    m_initialized = true;
    return true;
}

void IMU_Driver_Hil::SetHilData(const ImuSample& data)
{
    m_latestData = data;
    m_hasData = data.valid;
}

bool IMU_Driver_Hil::ReadRaw(ImuRawData& outRawData, uint32_t nowUs)
{
    if (!m_initialized || !m_hasData)
    {
        outRawData.valid = false;
        return false;
    }

    outRawData.rawAccelX = static_cast<int16_t>(
        (m_latestData.accel_mps2.x / G) * AccelRawPerG
    );

    outRawData.rawAccelY = static_cast<int16_t>(
        (m_latestData.accel_mps2.y / G) * AccelRawPerG
    );

    outRawData.rawAccelZ = static_cast<int16_t>(
        (m_latestData.accel_mps2.z / G) * AccelRawPerG
    );

    outRawData.rawGyroX = static_cast<int16_t>(
        (m_latestData.gyro_rads.x * RadToDeg) * GyroRawPerDps
    );

    outRawData.rawGyroY = static_cast<int16_t>(
        (m_latestData.gyro_rads.y * RadToDeg) * GyroRawPerDps
    );

    outRawData.rawGyroZ = static_cast<int16_t>(
        (m_latestData.gyro_rads.z * RadToDeg) * GyroRawPerDps
    );

    outRawData.temperature = 0;
    outRawData.timestampUs = nowUs;
    outRawData.valid = true;

    return true;
}

bool IMU_Driver_Hil::Read(ImuSample &outData, uint32_t nowUs)
{
    if (!m_initialized || !m_hasData)
    {
        outData.valid = false;
        return false;
    }

    outData = m_latestData;
    outData.timestampUs = nowUs;
    outData.valid = true;

    return true;
}
