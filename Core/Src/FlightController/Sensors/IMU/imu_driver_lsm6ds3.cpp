//
// Created by Dmytro Hrachov on 10.05.2026.
//

#include "FlightController/Sensors/IMU/imu_driver_lsm6ds3.h"

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

    //register for burst read Gyro and Accel
    constexpr uint8_t OUTX_L_G = 0x22;
}

IMU_Lsm6ds3::IMU_Lsm6ds3(SpiDmaBus& spiBus)
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

    if (!StartReadRaw())
    {
        return false;
    }

    m_accelScale_mps2 = AccelSensitivity_gPerLsb * G;
    m_gyroScale_rads = GyroSensitivity_dpsPerLsb * DegToRad;

    m_initialized = true;
    return true;
}

bool IMU_Lsm6ds3::StartReadRaw()
{
    if (m_spiBus.IsBusy())
    {
        return false;
    }

    m_txBuffer[0] = OUTX_L_G | SpiReadBit;

    for (uint8_t i = 1; i < SpiFrameSize; ++i)
    {
        m_txBuffer[i] = 0x00;
    }

    m_spiBus.ResetState();

    return m_spiBus.TransmitReceive(m_txBuffer, m_rxBuffer, SpiFrameSize);
}

bool IMU_Lsm6ds3::IsReadComplete() const
{
    return m_spiBus.IsDone();
}

bool IMU_Lsm6ds3::HasError() const
{
    return m_spiBus.HasError();
}

bool IMU_Lsm6ds3::ReadRaw(ImuRawData& outRawData, const uint32_t nowUs)
{
    if (m_spiBus.HasError())
    {
        m_spiBus.ResetState();

        if (!StartReadRaw())
        {
            outRawData.valid = false;
            return false;
        }
    }

    if (!m_spiBus.IsDone())
    {
        if (!m_spiBus.IsBusy())
        {
            StartReadRaw();
        }

        outRawData.valid = false;
        return false;
    }

    // rx[0] — garbage byte, реальні дані починаються з rx[1]
    const uint8_t* data = &m_rxBuffer[1];

    outRawData.rawGyroX  = MakeInt16(data[0],  data[1]);
    outRawData.rawGyroY  = MakeInt16(data[2],  data[3]);
    outRawData.rawGyroZ  = MakeInt16(data[4],  data[5]);

    outRawData.rawAccelX = MakeInt16(data[6],  data[7]);
    outRawData.rawAccelY = MakeInt16(data[8],  data[9]);
    outRawData.rawAccelZ = MakeInt16(data[10], data[11]);

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

bool IMU_Lsm6ds3::Read(ImuSample &outData, const uint32_t nowUs)
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
    outData.temperature_C = 25.0f;

    outData.timestampUs = nowUs;
    outData.valid = true;

    return true;
}

bool IMU_Lsm6ds3::CheckDeviceId() const
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

void IMU_Lsm6ds3::Reset()
{
    m_spiBus.ResetState();
}

int16_t IMU_Lsm6ds3::MakeInt16(uint8_t low, uint8_t high) const
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}
