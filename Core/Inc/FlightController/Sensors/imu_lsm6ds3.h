//
// Created by Dmytro Hrachov on 10.05.2026.
//
#pragma once

#include <cstdint>

#include "main.h"

class IMU_Lsm6ds3
{
public:
    struct RawData
    {
        int16_t gyroX;
        int16_t gyroY;
        int16_t gyroZ;

        int16_t accelX;
        int16_t accelY;
        std::int16_t accelZ;
    };

    struct Data
    {
        float gyroX_dps;
        float gyroY_dps;
        float gyroZ_dps;

        float accelX_g;
        float accelY_g;
        float accelZ_g;
    };
public:
    IMU_Lsm6ds3(
        SPI_HandleTypeDef* spi,
        GPIO_TypeDef* csPort,
        uint16_t csPin
    );

    bool Init();

    bool ReadWhoAmI(uint8_t& outValue);
    bool ReadRaw(RawData& outData);
    bool Read(Data& outData);

private:
    void Select();
    void Deselect();

    bool WriteReg(uint8_t reg, uint8_t value);
    bool ReadReg(uint8_t reg, uint8_t& value);
    bool ReadRegs(uint8_t startReg, uint8_t* buffer, uint16_t length);

private:
    GPIO_TypeDef* m_csPort;
    uint16_t m_csPin;

    float m_gyroSensitivity_dps;
    float m_accelSensitivity_g;
};
