//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "main.h"

class Stm32UartDmaByteStream
{
public:
    static constexpr uint16_t DmaBufferSize = 256;
    static constexpr uint16_t RingBufferSize = 512;

    explicit Stm32UartDmaByteStream(UART_HandleTypeDef& uartHandle);

    bool Init();

    void OnIdleIrq();
    void OnDmaProgressIrq();

    uint16_t Available() const;
    uint32_t GetOverflowCount() const;

    bool ReadByte(uint8_t& byte);

private:
    void CaptureNewBytesFromDma();
    bool PushToRing(uint8_t byte);

private:
    UART_HandleTypeDef& m_uartHandle;

    uint8_t m_dmaBuffer[DmaBufferSize] = {};
    volatile uint16_t m_dmaReadPos = 0;

    uint8_t m_ringBuffer[RingBufferSize] = {};
    volatile uint16_t m_ringWritePos = 0;
    volatile uint16_t m_ringReadPos = 0;

    volatile uint32_t m_overflowCount = 0;

    bool m_initialized = false;
};
