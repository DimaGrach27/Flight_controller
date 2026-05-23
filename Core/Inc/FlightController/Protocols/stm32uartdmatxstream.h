//
// Created by Dmytro Hrachov on 23.05.2026.
//
#pragma once

#include "main.h"
#include <cstdint>
#include <cstddef>

class Stm32UartDmaTxStream
{
public:
    explicit Stm32UartDmaTxStream(UART_HandleTypeDef& uart);

    bool Init();

    bool Write(const uint8_t* data, uint16_t size);
    bool WriteString(const char* str, uint16_t size);

    void OnTxComplete();
    void OnTxError();

    bool IsBusy() const;
    uint32_t GetDroppedBytes() const;

private:
    static constexpr uint16_t TX_BUFFER_SIZE = 512;

    bool StartTransfer();
    uint16_t GetAvailableSpace() const;
    uint16_t GetUsedSpace() const;

private:
    UART_HandleTypeDef& m_uart;

    uint8_t m_buffer[TX_BUFFER_SIZE] = {};

    volatile uint16_t m_head = 0;
    volatile uint16_t m_tail = 0;

    volatile bool m_busy = false;
    volatile uint16_t m_activeSize = 0;

    volatile uint32_t m_droppedBytes = 0;
};
