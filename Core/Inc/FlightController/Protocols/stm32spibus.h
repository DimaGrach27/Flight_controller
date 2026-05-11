//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "ispibus.h"
#include "stm32f4xx_hal_spi.h"

class Stm32SpiBus : public ISpiBus
{
public:
    Stm32SpiBus(
        SPI_HandleTypeDef& spiHandle,
        GPIO_TypeDef* csPort,
        uint16_t csPin
    );

    bool WriteRegister(uint8_t reg, uint8_t value, const uint8_t writeMask) override;
    bool ReadRegisters(uint8_t startReg, uint8_t* buffer, uint16_t size, const uint8_t spiReadBit) override;

private:
    void Select();
    void Deselect();

private:
    SPI_HandleTypeDef& m_spiHandle;

    GPIO_TypeDef* m_csPort = nullptr;
    uint16_t m_csPin = 0;

    uint32_t m_timeoutMs = 10;
};
