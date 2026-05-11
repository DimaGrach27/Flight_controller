//
// Created by Dmytro Hrachov on 11.05.2026.
//
#include "FlightController/peripheralhandler.h"

extern "C"
{
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi2;
}

HAL_StatusTypeDef PeripheralHandler::SendData_UART(const uint8_t *data, const uint16_t len, const USART_TypeDef* usart)
{
    if (huart1.Instance == usart)
    {
        return HAL_UART_Transmit(&huart1, data, len, 100);
    }

    if (huart2.Instance == usart)
    {
        return HAL_UART_Transmit(&huart2, data, len, 100);
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef PeripheralHandler::SendData_SPI(const uint8_t *data, const uint16_t len, const SPI_TypeDef* spi)
{
    if (hspi2.Instance == spi)
    {
        return HAL_SPI_Transmit(&hspi2, data, len, 100);
    }

    return HAL_ERROR;
}
