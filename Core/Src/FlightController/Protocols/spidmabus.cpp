//
// Created by Dmytro Hrachov on 21.05.2026.
//

#include "FlightController/Protocols/spidmabus.h"

SpiDmaBus::SpiDmaBus(SPI_HandleTypeDef* spi,
                     GPIO_TypeDef* csPort,
                     uint16_t csPin)
    : m_spi(spi),
      m_csPort(csPort),
      m_csPin(csPin)
{
    Deselect();
}

bool SpiDmaBus::TransmitReceive(uint8_t* txBuffer, uint8_t* rxBuffer, uint16_t size)
{
    if (m_spi == nullptr || txBuffer == nullptr || rxBuffer == nullptr || size == 0)
    {
        return false;
    }

    if (m_state == State::Busy)
    {
        return false;
    }

    m_state = State::Busy;

    Select();

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        m_spi,
        txBuffer,
        rxBuffer,
        size
    );

    if (status != HAL_OK)
    {
        Deselect();
        m_state = State::Error;
        return false;
    }

    return true;
}

bool SpiDmaBus::WriteRegister(uint8_t reg, uint8_t value, const uint8_t writeMask)
{
    const uint8_t txBuffer[2] = {
        static_cast<uint8_t>(reg & writeMask),
        value
    };

    Select();

    const HAL_StatusTypeDef status = HAL_SPI_Transmit(
        m_spi,
        const_cast<uint8_t*>(txBuffer),
        sizeof(txBuffer),
        m_timeoutMs
    );

    Deselect();

    return status == HAL_OK;
}

bool SpiDmaBus::ReadRegisters(uint8_t startReg, uint8_t *buffer, uint16_t size, const uint8_t spiReadBit)
{
    if (buffer == nullptr || size == 0)
    {
        return false;
    }

    const uint8_t command = static_cast<uint8_t>(startReg | spiReadBit);

    Select();

    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi,
        const_cast<uint8_t*>(&command),
        1,
        m_timeoutMs
    );

    if (status != HAL_OK)
    {
        Deselect();
        return false;
    }

    status = HAL_SPI_Receive(m_spi, buffer, size, m_timeoutMs);

    Deselect();

    return status == HAL_OK;
}

void SpiDmaBus::OnDmaComplete(SPI_HandleTypeDef* spi)
{
    if (spi != m_spi)
    {
        return;
    }

    Deselect();
    m_state = State::Done;
}

void SpiDmaBus::OnDmaError(SPI_HandleTypeDef* spi)
{
    if (spi != m_spi)
    {
        return;
    }

    Deselect();
    m_state = State::Error;
}

bool SpiDmaBus::IsBusy() const
{
    return m_state == State::Busy;
}

bool SpiDmaBus::IsDone() const
{
    return m_state == State::Done;
}

bool SpiDmaBus::HasError() const
{
    return m_state == State::Error;
}

void SpiDmaBus::ResetState()
{
    if (m_state != State::Busy)
    {
        m_state = State::Idle;
    }
}

void SpiDmaBus::Select() const
{
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
}

void SpiDmaBus::Deselect() const
{
    HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}