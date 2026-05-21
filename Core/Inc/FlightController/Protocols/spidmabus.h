//
// Created by Dmytro Hrachov on 21.05.2026.
//
#pragma once

#include "main.h"
#include <cstdint>
#include <cstddef>

class SpiDmaBus
{
public:
    enum class State : uint8_t
    {
        Idle,
        Busy,
        Done,
        Error
    };

    SpiDmaBus(SPI_HandleTypeDef* spi,
              GPIO_TypeDef* csPort,
              uint16_t csPin);

    bool TransmitReceive(uint8_t* txBuffer, uint8_t* rxBuffer, uint16_t size);
    bool WriteRegister(uint8_t reg, uint8_t value, const uint8_t writeMask);
    bool ReadRegisters(uint8_t startReg, uint8_t* buffer, uint16_t size, const uint8_t spiReadBit);

    void OnDmaComplete(SPI_HandleTypeDef* spi);
    void OnDmaError(SPI_HandleTypeDef* spi);

    bool IsBusy() const;
    bool IsDone() const;
    bool HasError() const;

    void ResetState();

private:
    void Select() const;
    void Deselect() const;

private:
    SPI_HandleTypeDef* m_spi = nullptr;

    GPIO_TypeDef* m_csPort = nullptr;
    uint16_t m_csPin = 0;

    volatile State m_state = State::Idle;

    const uint32_t m_timeoutMs = 10;
};
