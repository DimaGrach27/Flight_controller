//
// Created by Dmytro Hrachov on 05.05.2026.
//
#pragma once

#include "main.h"
#include "../structs.h"

class Logger
{
public:
    Logger(UART_HandleTypeDef& huart2);

    FlightLogSample& GetLogSample();
    void SendFlightLogCsv();

private:
    UART_HandleTypeDef& m_huart2;

    uint32_t m_lastDebugMs = 0;

    FlightLogSample m_logSample;

    constexpr static uint32_t LOG_PERIOD_MS = 200;
};
