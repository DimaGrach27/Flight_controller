//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "imotoroutput.h"
#include "main.h"
#include "FlightController/DebugLogs/usbdebugconsole.h"

class HilMotorOutput final : public IMotorOutput
{
public:
    explicit HilMotorOutput(UsbDebugConsole& debugConsole);

    bool Init() override;

    void Write(const MotorCommand& command) override;
    void StopAll() override;

    const MotorCommand& GetLastCommand() const;

private:
    void SendServoOutput(const MotorCommand& command);
private:
    MotorCommand m_lastCommand{};
    bool m_initialized = false;
    UsbDebugConsole& m_debugConsole;
};
