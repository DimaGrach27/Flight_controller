//
// Created by Dmytro Hrachov on 10.05.2026.
//

#include "../../../Inc/FlightController/Sensors/imu_lsm6ds3.h"

#include "FlightController/peripheralhandler.h"

namespace
{
    constexpr uint8_t WHO_AM_I_REG = 0x0F;
    constexpr uint8_t WHO_AM_I_EXPECTED = 0x69;

    constexpr uint8_t CTRL1_XL = 0x10;
    constexpr uint8_t CTRL2_G  = 0x11;
    constexpr uint8_t CTRL3_C  = 0x12;

    constexpr uint8_t OUTX_L_G = 0x22;

    constexpr uint8_t SPI_READ_BIT = 0x80;
    constexpr uint8_t SPI_WRITE_MASK = 0x7F;

    constexpr uint32_t SPI_TIMEOUT_MS = 100;
}

IMU_Lsm6ds3::IMU_Lsm6ds3(
    GPIO_TypeDef* csPort,
    uint16_t csPin
)
    : m_csPort(csPort)
    , m_csPin(csPin)
    , m_gyroSensitivity_dps(70.0f / 1000.0f) // ±2000 dps = 70 mdps/LSB
    , m_accelSensitivity_g(0.244f / 1000.0f) // ±8g = 0.244 mg/LSB
{
}

bool IMU_Lsm6ds3::Init()
{
    Deselect();
    HAL_Delay(10);

    uint8_t whoAmI = 0;

    if (!ReadWhoAmI(whoAmI))
        return false;

    if (whoAmI != WHO_AM_I_EXPECTED)
        return false;

    // CTRL3_C:
    // IF_INC = 1, auto-increment register address
    if (!WriteReg(CTRL3_C, 0x04))
        return false;

    HAL_Delay(10);

    // CTRL2_G:
    // ODR_G = 1.66 kHz
    // FS_G = ±2000 dps
    //
    // 0x8C:
    // ODR bits = 1000
    // FS bits для ±2000 dps
    if (!WriteReg(CTRL2_G, 0x8C))
        return false;

    // CTRL1_XL:
    // ODR_XL = 1.66 kHz
    // FS_XL = ±8g
    if (!WriteReg(CTRL1_XL, 0x8C))
        return false;

    HAL_Delay(50);

    return true;
}

bool IMU_Lsm6ds3::ReadWhoAmI(uint8_t& outValue)
{
    return ReadReg(WHO_AM_I_REG, outValue);
}

bool IMU_Lsm6ds3::ReadRaw(RawData& outData)
{
    uint8_t buffer[12] = {};

    if (!ReadRegs(OUTX_L_G, buffer, sizeof(buffer)))
        return false;

    outData.gyroX = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    outData.gyroY = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    outData.gyroZ = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

    outData.accelX = static_cast<int16_t>((buffer[7] << 8) | buffer[6]);
    outData.accelY = static_cast<int16_t>((buffer[9] << 8) | buffer[8]);
    outData.accelZ = static_cast<int16_t>((buffer[11] << 8) | buffer[10]);

    return true;
}

bool IMU_Lsm6ds3::Read(Data& outData)
{
    RawData raw{};

    if (!ReadRaw(raw))
        return false;

    outData.gyroX_dps = raw.gyroX * m_gyroSensitivity_dps;
    outData.gyroY_dps = raw.gyroY * m_gyroSensitivity_dps;
    outData.gyroZ_dps = raw.gyroZ * m_gyroSensitivity_dps;

    outData.accelX_g = raw.accelX * m_accelSensitivity_g;
    outData.accelY_g = raw.accelY * m_accelSensitivity_g;
    outData.accelZ_g = raw.accelZ * m_accelSensitivity_g;

    return true;
}

void IMU_Lsm6ds3::Select()
{
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
}

void IMU_Lsm6ds3::Deselect()
{
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}

bool IMU_Lsm6ds3::WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {
        static_cast<uint8_t>(reg & SPI_WRITE_MASK),
        value
    };

    Select();

    const HAL_StatusTypeDef status = PeripheralHandler::SendData_SPI(tx, sizeof(tx), SPI2);

    Deselect();

    return status == HAL_OK;
}

bool IMU_Lsm6ds3::ReadReg(uint8_t reg, uint8_t& value)
{
    uint8_t tx[2] = {
        static_cast<uint8_t>(reg | SPI_READ_BIT),
        0x00
    };

    uint8_t rx[2] = {};

    Select();

    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        m_spi,
        tx,
        rx,
        sizeof(tx),
        SPI_TIMEOUT_MS
    );

    Deselect();

    if (status != HAL_OK)
        return false;

    value = rx[1];
    return true;
}

bool IMU_Lsm6ds3::ReadRegs(uint8_t startReg, uint8_t* buffer, uint16_t length)
{
    uint8_t address = static_cast<uint8_t>(startReg | SPI_READ_BIT);

    Select();

    const HAL_StatusTypeDef status = PeripheralHandler::SendData_SPI(&address, 1, SPI2);


    if (status != HAL_OK)
    {
        Deselect();
        return false;
    }

    status = HAL_SPI_Receive(
        m_spi,
        buffer,
        length,
        SPI_TIMEOUT_MS
    );

    Deselect();

    return status == HAL_OK;
}