//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Protocols/stm32spibus.h"

Stm32SpiBus::Stm32SpiBus(
    SPI_HandleTypeDef& spiHandle,
    GPIO_TypeDef* csPort,
    uint16_t csPin
)
    : m_spiHandle(spiHandle)
    , m_csPort(csPort)
    , m_csPin(csPin)
{
}

bool Stm32SpiBus::WriteRegister(uint8_t reg, uint8_t value, const uint8_t writeMask)
{
    const uint8_t txBuffer[2] = {
        static_cast<uint8_t>(reg & writeMask),
        value
    };

    Select();

    const HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &m_spiHandle,
        const_cast<uint8_t*>(txBuffer),
        sizeof(txBuffer),
        m_timeoutMs
    );

    Deselect();

    return status == HAL_OK;
}

bool Stm32SpiBus::ReadRegisters(uint8_t startReg, uint8_t* buffer, uint16_t size, const uint8_t spiReadBit)
{
    if (buffer == nullptr || size == 0)
    {
        return false;
    }

    const uint8_t command = static_cast<uint8_t>(startReg | spiReadBit);

    Select();

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &m_spiHandle,
        const_cast<uint8_t*>(&command),
        1,
        m_timeoutMs
    );

    if (status != HAL_OK)
    {
        Deselect();
        return false;
    }

    status = HAL_SPI_Receive(
        &m_spiHandle,
        buffer,
        size,
        m_timeoutMs
    );

    Deselect();

    return status == HAL_OK;
}

void Stm32SpiBus::Select()
{
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
}

void Stm32SpiBus::Deselect()
{
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}