//
// Created by Dmytro Hrachov on 01.05.2026.
//
#pragma once

#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void flight_controller_Init(UART_HandleTypeDef* huart2);
void flight_controller_Update(float dt);
void flight_controller_Heartbeat(void);
void flight_controller_MavlinkParseByte(uint8_t byte);

#ifdef __cplusplus
}
#endif