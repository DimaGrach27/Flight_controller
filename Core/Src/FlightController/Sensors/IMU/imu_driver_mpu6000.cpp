//
// Created by Dmytro Hrachov on 19.07.2026.
//

#include "FlightController/Sensors/IMU/imu_driver_mpu6000.h"

#include <cstdint>

namespace
{
    constexpr float G = 9.80665f;
    constexpr float DegToRad = 0.01745329252f;

    constexpr uint8_t SampleRateDivider = 0x19;
    constexpr uint8_t Config = 0x1A;
    constexpr uint8_t GyroConfig = 0x1B;
    constexpr uint8_t AccelConfig = 0x1C;
    constexpr uint8_t AccelXoutH = 0x3B;
    constexpr uint8_t SignalPathReset = 0x68;
    constexpr uint8_t UserControl = 0x6A;
    constexpr uint8_t PowerManagement1 = 0x6B;
    constexpr uint8_t PowerManagement2 = 0x6C;
    constexpr uint8_t WhoAmI = 0x75;

    constexpr uint8_t WhoAmIExpected = 0x68;

    constexpr uint8_t SpiReadBit = 0x80;
    constexpr uint8_t SpiWriteMask = 0x7F;

    constexpr uint8_t DeviceResetBit = 0x80;
    constexpr uint8_t ClockSourcePllXGyro = 0x01;
    constexpr uint8_t DisableI2cInterface = 0x10;
    constexpr uint8_t ResetSignalPaths = 0x07;
    constexpr uint8_t EnableAllAxes = 0x00;

    constexpr uint8_t Dlpf98Hz = 0x02;
    constexpr uint8_t SampleRate1Khz = 0x00;
    constexpr uint8_t GyroRange2000Dps = 0x18;
    constexpr uint8_t AccelRange8G = 0x10;

    constexpr float AccelSensitivity_gPerLsb = 8.0f / 32768.0f;
    constexpr float GyroSensitivity_dpsPerLsb = 2000.0f / 32768.0f;
}

IMU_MPU6000::IMU_MPU6000(SpiDmaBus& spiBus)
    : m_spiBus(spiBus)
{

}

bool IMU_MPU6000::Init()
{
    m_initialized = false;
    m_spiBus.ResetState();

    if (!m_spiBus.WriteRegister(PowerManagement1, DeviceResetBit, SpiWriteMask))
    {
        return false;
    }

    HAL_Delay(100);

    if (!m_spiBus.WriteRegister(PowerManagement1, ClockSourcePllXGyro, SpiWriteMask))
    {
        return false;
    }

    HAL_Delay(10);

    if (!m_spiBus.WriteRegister(UserControl, DisableI2cInterface, SpiWriteMask))
    {
        return false;
    }

    if (!m_spiBus.WriteRegister(SignalPathReset, ResetSignalPaths, SpiWriteMask))
    {
        return false;
    }

    HAL_Delay(100);

    if (!CheckDeviceId())
    {
        return false;
    }

    if (!ConfigureDevice())
    {
        return false;
    }

    if (!StartReadRaw())
    {
        return false;
    }

    m_accelScale_mps2 = AccelSensitivity_gPerLsb * G;
    m_gyroScale_rads = GyroSensitivity_dpsPerLsb * DegToRad;

    m_initialized = true;
    return true;
}

bool IMU_MPU6000::StartReadRaw()
{
    if (m_spiBus.IsBusy())
    {
        return false;
    }

    m_txBuffer[0] = AccelXoutH | SpiReadBit;

    for (uint8_t i = 1; i < SpiFrameSize; ++i)
    {
        m_txBuffer[i] = 0x00;
    }

    m_spiBus.ResetState();

    return m_spiBus.TransmitReceive(m_txBuffer, m_rxBuffer, SpiFrameSize);
}

bool IMU_MPU6000::IsReadComplete() const
{
    return m_spiBus.IsDone();
}

bool IMU_MPU6000::HasError() const
{
    return m_spiBus.HasError();
}

bool IMU_MPU6000::ReadRaw(ImuRawData& outRawData, const uint32_t nowUs)
{
    if (!m_spiBus.IsDone())
    {
        outRawData.valid = false;
        return false;
    }

    const uint8_t* data = &m_rxBuffer[1];

    outRawData.rawAccelX = MakeInt16(data[0], data[1]);
    outRawData.rawAccelY = MakeInt16(data[2], data[3]);
    outRawData.rawAccelZ = MakeInt16(data[4], data[5]);
    outRawData.temperature = MakeInt16(data[6], data[7]);
    outRawData.rawGyroX = MakeInt16(data[8], data[9]);
    outRawData.rawGyroY = MakeInt16(data[10], data[11]);
    outRawData.rawGyroZ = MakeInt16(data[12], data[13]);

    m_spiBus.ResetState();

    if (!StartReadRaw())
    {
        outRawData.valid = false;
        return false;
    }

    outRawData.timestampUs = nowUs;
    outRawData.valid = true;

    return true;
}

bool IMU_MPU6000::Read(ImuSample& outData, const uint32_t nowUs)
{
    ImuRawData outRawData;
    if (!ReadRaw(outRawData, nowUs))
    {
        outData.valid = false;
        return false;
    }

    outData.accel_mps2.x = static_cast<float>(outRawData.rawAccelX) * m_accelScale_mps2;
    outData.accel_mps2.y = static_cast<float>(outRawData.rawAccelY) * m_accelScale_mps2;
    outData.accel_mps2.z = static_cast<float>(outRawData.rawAccelZ) * m_accelScale_mps2;

    outData.gyro_rads.x = static_cast<float>(outRawData.rawGyroX) * m_gyroScale_rads;
    outData.gyro_rads.y = static_cast<float>(outRawData.rawGyroY) * m_gyroScale_rads;
    outData.gyro_rads.z = static_cast<float>(outRawData.rawGyroZ) * m_gyroScale_rads;

    outData.temperature_C = static_cast<float>(outRawData.temperature) / 340.0f + 36.53f;
    outData.timestampUs = nowUs;
    outData.valid = true;

    return true;
}

bool IMU_MPU6000::CheckDeviceId() const
{
    uint8_t whoAmI = 0;

    if (!m_spiBus.ReadRegisters(WhoAmI, &whoAmI, 1, SpiReadBit))
    {
        return false;
    }

    return whoAmI == WhoAmIExpected;
}

bool IMU_MPU6000::ConfigureDevice()
{
    if (!m_spiBus.WriteRegister(Config, Dlpf98Hz, SpiWriteMask))
    {
        return false;
    }

    if (!m_spiBus.WriteRegister(SampleRateDivider, SampleRate1Khz, SpiWriteMask))
    {
        return false;
    }

    if (!m_spiBus.WriteRegister(GyroConfig, GyroRange2000Dps, SpiWriteMask))
    {
        return false;
    }

    if (!m_spiBus.WriteRegister(AccelConfig, AccelRange8G, SpiWriteMask))
    {
        return false;
    }

    if (!m_spiBus.WriteRegister(PowerManagement2, EnableAllAxes, SpiWriteMask))
    {
        return false;
    }

    HAL_Delay(50);

    return true;
}

void IMU_MPU6000::Reset()
{
    m_spiBus.ResetState();
    m_initialized = false;
}

int16_t IMU_MPU6000::MakeInt16(uint8_t high, uint8_t low) const
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}
