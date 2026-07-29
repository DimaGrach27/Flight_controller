//
// Created by Dmytro Hrachov on 05.05.2026.
//
#pragma once

#include "main.h"
#include "usbdebugconsole.h"
#include "../structs.h"

class Logger
{
public:
    Logger(UsbDebugConsole& debugConsole);

    FlightLogSample& GetLogSample();
    void SendFlightLogBinary();
    void SetBinaryUsbEnabled(bool enabled);
    bool IsBinaryUsbEnabled() const;

private:
    UsbDebugConsole& m_debugConsole;

    uint32_t m_lastDebugMs = 0;
    bool m_binaryUsbEnabled = false;

    FlightLogSample m_logSample;

    constexpr static uint32_t LOG_PERIOD_MS = 50;
};
