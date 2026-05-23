//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Protocols/stm32uartdmarxstream.h"

Stm32UartDmaRxStream::Stm32UartDmaRxStream(UART_HandleTypeDef &uartHandle)
    : m_uartHandle(uartHandle)
{

}

bool Stm32UartDmaRxStream::Init()
{
    m_dmaReadPos = 0;
    m_ringWritePos = 0;
    m_ringReadPos = 0;
    m_overflowCount = 0;

    const HAL_StatusTypeDef status = HAL_UART_Receive_DMA(
        &m_uartHandle,
        m_dmaBuffer,
        DmaBufferSize
    );

    if (status != HAL_OK)
    {
        m_initialized = false;
        return false;
    }

    __HAL_UART_ENABLE_IT(&m_uartHandle, UART_IT_IDLE);
    m_initialized = true;
    return true;
}

void Stm32UartDmaRxStream::OnIdleIrq()
{
    if (__HAL_UART_GET_FLAG(&m_uartHandle, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&m_uartHandle);

        CaptureNewBytesFromDma();
    }
}

void Stm32UartDmaRxStream::OnDmaProgressIrq()
{
    CaptureNewBytesFromDma();
}

void Stm32UartDmaRxStream::CaptureNewBytesFromDma()
{
    const uint16_t dmaWritePos =
        static_cast<uint16_t>(DmaBufferSize - __HAL_DMA_GET_COUNTER(m_uartHandle.hdmarx));

    while (m_dmaReadPos != dmaWritePos)
    {
        const uint8_t byte = m_dmaBuffer[m_dmaReadPos];

        m_dmaReadPos++;
        if (m_dmaReadPos >= DmaBufferSize)
        {
            m_dmaReadPos = 0;
        }

        if (!PushToRing(byte))
        {
            m_overflowCount++;
            break;
        }
    }
}

bool Stm32UartDmaRxStream::PushToRing(const uint8_t byte)
{
    uint16_t nextWritePos = m_ringWritePos + 1;
    if (nextWritePos >= RingBufferSize)
    {
        nextWritePos = 0;
    }

    if (nextWritePos == m_ringReadPos)
    {
        return false;
    }

    m_ringBuffer[m_ringWritePos] = byte;
    m_ringWritePos = nextWritePos;

    return true;
}

bool Stm32UartDmaRxStream::ReadByte(uint8_t& byte)
{
    if (m_ringReadPos == m_ringWritePos)
    {
        return false;
    }

    byte = m_ringBuffer[m_ringReadPos];

    m_ringReadPos++;
    if (m_ringReadPos >= RingBufferSize)
    {
        m_ringReadPos = 0;
    }

    return true;
}

uint16_t Stm32UartDmaRxStream::Available() const
{
    const uint16_t writePos = m_ringWritePos;
    const uint16_t readPos = m_ringReadPos;

    if (writePos >= readPos)
    {
        return writePos - readPos;
    }

    return RingBufferSize - readPos + writePos;
}

uint32_t Stm32UartDmaRxStream::GetOverflowCount() const
{
    return m_overflowCount;
}