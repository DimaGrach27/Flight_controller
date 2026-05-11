//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "imotoroutput.h"
#include "main.h"

class HilMotorOutput final : public IMotorOutput
{
public:
    explicit HilMotorOutput(UART_HandleTypeDef& serialUart);

    bool Init() override;

    void Write(const MotorCommand& command) override;
    void StopAll() override;

    const MotorCommand& GetLastCommand() const;

private:
    void SendServoOutput(const MotorCommand& command);
private:
    MotorCommand m_lastCommand{};
    bool m_initialized = false;
    UART_HandleTypeDef& m_serialUart;
};
