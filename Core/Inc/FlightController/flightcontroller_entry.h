//
// Created by Dmytro Hrachov on 01.05.2026.
//
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void flight_controller_Create(
    UART_HandleTypeDef* huart1,
    UART_HandleTypeDef* huart2,
    SPI_HandleTypeDef* hspi2,
    TIM_HandleTypeDef* htim1,
    ADC_HandleTypeDef* hadc1
    );
void flight_controller_Destroy();

void flight_controller_Init();

void flight_controller_Heartbeat(void);
void flight_controller_Update(void);

bool flight_controller_MavlinkParseByte(uint8_t byte);
void flight_controller_ParseRcCommandByte(uint8_t byte);

//hard callbacks begin
void flight_controller_OnIdleDmaReceive_UART1();
//hard callbacks end

//debug block begin
void UsbDebugConsole_OnReceived(uint8_t* data, uint32_t size);
void UsbDebugConsole_OnTransmitComplete();

void UsbDebugConsole_RunDebugCommand(uint8_t command);
void UsbDebugConsole_RunDebugTextCommand(const char* command);
//debug block end
#ifdef __cplusplus
}
#endif
