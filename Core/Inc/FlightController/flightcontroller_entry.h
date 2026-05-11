//
// Created by Dmytro Hrachov on 01.05.2026.
//
#pragma once

#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void flight_controller_Create(
    UART_HandleTypeDef* huart1,
    UART_HandleTypeDef* huart2,
    SPI_HandleTypeDef* hspi2,
    TIM_HandleTypeDef* htim1
    );
void flight_controller_Destroy();

void flight_controller_Init();

void flight_controller_Heartbeat(void);
void flight_controller_Update(void);

void flight_controller_MavlinkParseByte(uint8_t byte);
void flight_controller_ParseRcCommandByte(uint8_t byte);

#ifdef __cplusplus
}
#endif