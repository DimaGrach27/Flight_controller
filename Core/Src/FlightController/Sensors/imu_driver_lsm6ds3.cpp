//
// Created by Dmytro Hrachov on 10.05.2026.
//

#include "FlightController/Sensors/imu_driver_lsm6ds3.h"

#include <cstdint>

namespace
{
    constexpr float G = 9.80665f;
    constexpr float DegToRad = 0.01745329252f;

    constexpr uint8_t WHO_AM_I_REG = 0x0F;
    constexpr uint8_t WHO_AM_I_EXPECTED = 0x69;

    constexpr uint8_t CTRL1_XL = 0x10;
    constexpr uint8_t CTRL2_G  = 0x11;
    constexpr uint8_t CTRL3_C  = 0x12;

    constexpr uint8_t OUT_TEMP_L = 0x20;

    /*
        CTRL1_XL:
        ODR_XL = 0111 -> 833 Hz
        FS_XL  = 00   -> ±2g
        BW_XL  = 00

        0b01110000 = 0x70
    */
    constexpr uint8_t CTRL1_XL_833HZ_2G = 0x70;

    /*
        CTRL2_G:
        ODR_G = 0111 -> 833 Hz
        FS_G  = 00   -> ±245 dps

        0b01110000 = 0x70
    */
    constexpr uint8_t CTRL2_G_833HZ_245DPS = 0x70;

    /*
        CTRL3_C:
        BDU    = 1 -> block data update
        IF_INC = 1 -> auto increment register address

        BDU bit    = bit 6
        IF_INC bit = bit 2

        0b01000100 = 0x44
    */
    constexpr uint8_t CTRL3_C_BDU_IF_INC = 0x44;

    /*
        Sensitivity for LSM6DS3:

        Accel ±2g:
        0.061 mg/LSB = 0.000061 g/LSB

        Gyro ±245 dps:
        8.75 mdps/LSB = 0.00875 dps/LSB
    */
    constexpr float AccelSensitivity_gPerLsb = 0.000061f;
    constexpr float GyroSensitivity_dpsPerLsb = 0.00875f;

    constexpr uint8_t SpiReadBit = 0x80;
    constexpr uint8_t SpiWriteMask = 0x7F;
}

IMU_Lsm6ds3::IMU_Lsm6ds3(ISpiBus& spiBus)
    : m_spiBus(spiBus)
{

}

bool IMU_Lsm6ds3::Init()
{
    m_initialized = false;

    if (!CheckDeviceId())
    {
        return false;
    }

    if (!ConfigureDevice())
    {
        return false;
    }

    m_accelScale_mps2 = AccelSensitivity_gPerLsb * G;
    m_gyroScale_rads = GyroSensitivity_dpsPerLsb * DegToRad;

    m_initialized = true;
    return true;
}

bool IMU_Lsm6ds3::ReadRaw(ImuRawData& outRawData, uint32_t nowUs)
{
    if (!m_initialized)
    {
        outRawData.valid = false;
        return false;
    }

    uint8_t buffer[14] = {};

    const bool ok = m_spiBus.ReadRegisters(OUT_TEMP_L, buffer, sizeof(buffer), SpiReadBit);
    if (!ok)
    {
        outRawData.valid = false;
        return false;
    }

    // LSM6DS3 order from OUT_TEMP_L:
    // TEMP_L, TEMP_H,
    // GYRO_X_L, GYRO_X_H,
    // GYRO_Y_L, GYRO_Y_H,
    // GYRO_Z_L, GYRO_Z_H,
    // ACCEL_X_L, ACCEL_X_H,
    // ACCEL_Y_L, ACCEL_Y_H,
    // ACCEL_Z_L, ACCEL_Z_H

    outRawData.temperature = ReadInt16Le(buffer, 0);

    outRawData.rawGyroX = ReadInt16Le(buffer, 2);
    outRawData.rawGyroY = ReadInt16Le(buffer, 4);
    outRawData.rawGyroZ = ReadInt16Le(buffer, 6);

    outRawData.rawAccelX = ReadInt16Le(buffer, 8);
    outRawData.rawAccelY = ReadInt16Le(buffer, 10);
    outRawData.rawAccelZ = ReadInt16Le(buffer, 12);

    outRawData.timestampUs = nowUs;
    outRawData.valid = true;

    return true;
}

bool IMU_Lsm6ds3::Read(ImuSample &outData, uint32_t nowUs)
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

    // Для LSM6DS3 температура приблизно:
    // 25 + raw / 16
    outData.temperature_C = 25.0f + static_cast<float>(outRawData.temperature) / 16.0f;

    outData.timestampUs = nowUs;
    outData.valid = true;

    return true;
}

bool IMU_Lsm6ds3::CheckDeviceId()
{
    uint8_t whoAmI = 0;

    if (!m_spiBus.ReadRegisters(WHO_AM_I_REG, &whoAmI, 1, SpiReadBit))
    {
        return false;
    }

    return whoAmI == WHO_AM_I_EXPECTED;
}

bool IMU_Lsm6ds3::ConfigureDevice()
{
    if (!m_spiBus.WriteRegister(CTRL3_C, CTRL3_C_BDU_IF_INC, SpiWriteMask))
    {
        return false;
    }

    if (!m_spiBus.WriteRegister(CTRL1_XL, CTRL1_XL_833HZ_2G, SpiWriteMask))
    {
        return false;
    }

    if (!m_spiBus.WriteRegister(CTRL2_G, CTRL2_G_833HZ_245DPS, SpiWriteMask))
    {
        return false;
    }

    return true;
}

int16_t IMU_Lsm6ds3::ReadInt16Le(const uint8_t *buffer, uint8_t lowIndex) const
{
    const uint16_t low = static_cast<uint16_t>(buffer[lowIndex]);
    const uint16_t high = static_cast<uint16_t>(buffer[lowIndex + 1]);

    return static_cast<int16_t>((high << 8) | low);
}
