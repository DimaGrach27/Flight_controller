//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <array>

#include "imotoroutput.h"
#include "main.h"
#include "motorchannel.h"

#include <cstdint>

class DshotMotorOutput final : public IMotorOutput
{
public:
    enum class Speed : uint32_t
    {
        DShot150 = 150000U,
        DShot300 = 300000U,
        DShot600 = 600000U
    };

    struct Config
    {
        Speed speed = Speed::DShot300;

        /*
            Для NUCLEO-F411RE часто timer clock = 84 MHz,
            якщо APB timer clock налаштований стандартно.
        */
        uint32_t timerClockHz = 84000000U;

        /*
            Bidirectional DShot telemetry тут поки не реалізована.
            Це тільки telemetry request bit у DShot packet.
            Для звичайного ESC залишай false.
        */
        bool telemetryRequest = false;
    };

public:
    explicit DshotMotorOutput(TIM_HandleTypeDef* timer, const Config& config);

    bool Init() override;

    void Write(const MotorCommand& command) override;
    void StopAll() override;

    void OnDmaComplete(TIM_HandleTypeDef* timer);

private:
    void WriteRawValues(
        uint16_t motor1,
        uint16_t motor2,
        uint16_t motor3,
        uint16_t motor4
    );

    uint16_t NormalizeToDshotValue(float value) const;

    uint16_t BuildPacket(uint16_t value, bool telemetryRequest) const;
    uint8_t CalculateChecksum(uint16_t packetWithoutChecksum) const;

    void FillDmaBuffer(
        uint16_t packet1,
        uint16_t packet2,
        uint16_t packet3,
        uint16_t packet4
    );

    bool StartBurstTransfer();
    void StopBurstTransfer();

    bool IsBusy() const;
    void SetBusy(bool busy);

private:
    static constexpr uint8_t kMotorCount = 4;
    static constexpr uint8_t kPacketBits = 16;
    static constexpr uint8_t kResetSlots = 2;

    static constexpr uint8_t kFrameSlots = kPacketBits + kResetSlots;
    static constexpr uint8_t kRegistersPerSlot = 4;

    static constexpr uint16_t kMinThrottleValue = 48;
    static constexpr uint16_t kMaxThrottleValue = 2047;

private:
    static constexpr uint32_t kMotorChannels[kMotorCount] =
    {
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        TIM_CHANNEL_3,
        TIM_CHANNEL_4
    };

private:
    TIM_HandleTypeDef* m_timer = nullptr;
    Config m_config{};

    /*
        Buffer layout:

        slot 0, bit 15:
            [CCR1, CCR2, CCR3, CCR4]

        slot 1, bit 14:
            [CCR1, CCR2, CCR3, CCR4]

        ...

        slot 15, bit 0:
            [CCR1, CCR2, CCR3, CCR4]

        slot 16..17:
            [0, 0, 0, 0]
    */
    std::array<uint32_t, kFrameSlots * kRegistersPerSlot> m_dmaBuffer{};

    uint32_t m_bitPeriodTicks = 0;
    uint32_t m_zeroHighTicks = 0;
    uint32_t m_oneHighTicks = 0;

    bool m_initialized = false;
    bool m_busy = false;
};
