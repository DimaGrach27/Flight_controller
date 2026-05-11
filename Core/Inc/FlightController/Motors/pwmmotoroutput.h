//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include <array>
#include <cstdint>

#include "imotoroutput.h"
#include "main.h"

struct MotorOutputConfig
{
    uint16_t minPulseUs = 1000;
    uint16_t maxPulseUs = 2000;
    uint16_t stopPulseUs = 1000;
    uint16_t idlePulseUs = 1050;

    bool useIdleWhenArmed = false;
};

class PwmMotorOutput final : public IMotorOutput
{
public:
    struct MotorChannel
    {
        TIM_HandleTypeDef* timer = nullptr;
        uint32_t channel = 0;
    };

public:
    PwmMotorOutput(const std::array<MotorChannel, 4>& channels);

    bool Init() override;

    void Write(const MotorCommand& command) override;
    void StopAll() override;

private:
    void WriteMotor(uint8_t index, float normalizedValue);
    void WritePulseUs(uint8_t index, uint16_t pulseUs);

    uint16_t NormalizeToPulseUs(float value) const;

private:
    std::array<MotorChannel, 4> m_channels{};
    MotorOutputConfig m_config{};

    bool m_initialized = false;
};
