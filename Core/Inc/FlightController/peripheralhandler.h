//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "main.h"
#include <cstdint>

class PeripheralHandler
{
public:
    PeripheralHandler() = default;
    ~PeripheralHandler() = default;

    static HAL_StatusTypeDef SendData_UART(const uint8_t *data, const uint16_t len, const USART_TypeDef* usart);
    static HAL_StatusTypeDef SendData_SPI(const uint8_t *data, const uint16_t len, const SPI_TypeDef* spi);

private:
};