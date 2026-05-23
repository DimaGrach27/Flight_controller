//
// Created by Dmytro Hrachov on 23.05.2026.
//

#include "FlightController/Protocols/stm32uartdmatxstream.h"

Stm32UartDmaTxStream::Stm32UartDmaTxStream(UART_HandleTypeDef& uart)
    : m_uart(uart)
{
}

bool Stm32UartDmaTxStream::Init()
{
    m_head = 0;
    m_tail = 0;
    m_busy = false;
    m_activeSize = 0;
    m_droppedBytes = 0;

    return true;
}

bool Stm32UartDmaTxStream::WriteString(const char* str, const uint16_t size)
{
    if (str == nullptr)
    {
        return false;
    }

    return Write(reinterpret_cast<const uint8_t*>(str), size);
}

bool Stm32UartDmaTxStream::Write(const uint8_t* data, const uint16_t size)
{
    if (data == nullptr || size == 0)
    {
        return false;
    }

    __disable_irq();

    const uint16_t available = GetAvailableSpace();

    if (size > available)
    {
        m_droppedBytes += size;
        __enable_irq();
        return false;
    }

    for (uint16_t i = 0; i < size; ++i)
    {
        m_buffer[m_head] = data[i];
        m_head = static_cast<uint16_t>((m_head + 1) % TX_BUFFER_SIZE);
    }

    const bool needStart = !m_busy;

    __enable_irq();

    if (needStart)
    {
        return StartTransfer();
    }

    return true;
}

bool Stm32UartDmaTxStream::StartTransfer()
{
    __disable_irq();

    if (m_busy || m_head == m_tail)
    {
        __enable_irq();
        return true;
    }

    uint16_t size = 0;

    if (m_head > m_tail)
    {
        size = static_cast<uint16_t>(m_head - m_tail);
    }
    else
    {
        size = static_cast<uint16_t>(TX_BUFFER_SIZE - m_tail);
    }

    m_activeSize = size;
    m_busy = true;

    const uint8_t* txPtr = &m_buffer[m_tail];

    __enable_irq();

    const HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&m_uart, txPtr, size);

    if (status != HAL_OK)
    {
        __disable_irq();
        m_busy = false;
        m_activeSize = 0;
        __enable_irq();

        return false;
    }

    return true;
}

void Stm32UartDmaTxStream::OnTxComplete()
{
    __disable_irq();

    m_tail = static_cast<uint16_t>((m_tail + m_activeSize) % TX_BUFFER_SIZE);
    m_activeSize = 0;
    m_busy = false;

    const bool hasMoreData = (m_head != m_tail);

    __enable_irq();

    if (hasMoreData)
    {
        StartTransfer();
    }
}

void Stm32UartDmaTxStream::OnTxError()
{
    __disable_irq();

    m_busy = false;
    m_activeSize = 0;

    __enable_irq();
}

bool Stm32UartDmaTxStream::IsBusy() const
{
    return m_busy;
}

uint32_t Stm32UartDmaTxStream::GetDroppedBytes() const
{
    return m_droppedBytes;
}

uint16_t Stm32UartDmaTxStream::GetAvailableSpace() const
{
    return static_cast<uint16_t>(TX_BUFFER_SIZE - GetUsedSpace() - 1);
}

uint16_t Stm32UartDmaTxStream::GetUsedSpace() const
{
    if (m_head >= m_tail)
    {
        return static_cast<uint16_t>(m_head - m_tail);
    }

    return static_cast<uint16_t>(TX_BUFFER_SIZE - m_tail + m_head);
}