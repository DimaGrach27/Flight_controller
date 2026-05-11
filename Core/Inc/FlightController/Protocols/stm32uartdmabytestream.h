//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include "iuartbytestrem.h"
#include "main.h"

class Stm32UartDmaByteStream final : public IUartByteStream
{
public:
    Stm32UartDmaByteStream(
        UART_HandleTypeDef& uartHandle,
        uint8_t* dmaBuffer,
        uint16_t dmaBufferSize
    );

    bool Init() override;

    uint16_t Read(uint8_t* outData, uint16_t maxSize) override;

private:
    uint16_t GetDmaWritePosition() const;

private:
    UART_HandleTypeDef& m_uartHandle;

    uint8_t* m_dmaBuffer = nullptr;
    uint16_t m_dmaBufferSize = 0;

    uint16_t m_readPos = 0;

    bool m_initialized = false;
};
