//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Protocols/stm32uartdmabytestream.h"

Stm32UartDmaByteStream::Stm32UartDmaByteStream(
    UART_HandleTypeDef& uartHandle,
    uint8_t* dmaBuffer,
    uint16_t dmaBufferSize
)
    : m_uartHandle(uartHandle)
    , m_dmaBuffer(dmaBuffer)
    , m_dmaBufferSize(dmaBufferSize)
{
}

bool Stm32UartDmaByteStream::Init()
{
    if (m_dmaBuffer == nullptr || m_dmaBufferSize == 0)
    {
        m_initialized = false;
        return false;
    }

    m_readPos = 0;

    const HAL_StatusTypeDef status = HAL_UART_Receive_DMA(
        &m_uartHandle,
        m_dmaBuffer,
        m_dmaBufferSize
    );

    if (status != HAL_OK)
    {
        m_initialized = false;
        return false;
    }

    m_initialized = true;
    return true;
}

uint16_t Stm32UartDmaByteStream::Read(uint8_t* outData, uint16_t maxSize)
{
    if (!m_initialized || outData == nullptr || maxSize == 0)
    {
        return 0;
    }

    const uint16_t writePos = GetDmaWritePosition();

    if (writePos == m_readPos)
    {
        return 0;
    }

    uint16_t readCount = 0;

    while (m_readPos != writePos && readCount < maxSize)
    {
        outData[readCount] = m_dmaBuffer[m_readPos];

        ++readCount;
        ++m_readPos;

        if (m_readPos >= m_dmaBufferSize)
        {
            m_readPos = 0;
        }
    }

    return readCount;
}

uint16_t Stm32UartDmaByteStream::GetDmaWritePosition() const
{
    /*
        NDTR = скільки байтів ще залишилось DMA до кінця buffer.
        Якщо buffer size = 256 і NDTR = 200,
        значить DMA вже записав 56 байтів.

        writePos = size - NDTR
    */
    const uint16_t remaining =
        static_cast<uint16_t>(__HAL_DMA_GET_COUNTER(m_uartHandle.hdmarx));

    return static_cast<uint16_t>(m_dmaBufferSize - remaining);
}