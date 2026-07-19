//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Motors/dshotmotoroutput.h"

#include "FlightController/Utils/mathutils.h"

namespace
{
    bool IsApb2Timer(const TIM_TypeDef* timer)
    {
        return timer == TIM1
            || timer == TIM8
#if defined(TIM9)
            || timer == TIM9
#endif
#if defined(TIM10)
            || timer == TIM10
#endif
#if defined(TIM11)
            || timer == TIM11
#endif
            ;
    }

    uint32_t GetTimerClockHz(const TIM_HandleTypeDef* timer)
    {
        if (timer == nullptr)
        {
            return 0U;
        }

        const bool apb2Timer = IsApb2Timer(timer->Instance);
        const uint32_t pclk = apb2Timer
            ? HAL_RCC_GetPCLK2Freq()
            : HAL_RCC_GetPCLK1Freq();
        const uint32_t prescalerMask = apb2Timer
            ? (RCC->CFGR & RCC_CFGR_PPRE2)
            : (RCC->CFGR & RCC_CFGR_PPRE1);

        return prescalerMask == 0U ? pclk : pclk * 2U;
    }
}

DshotMotorOutput::DshotMotorOutput(
    TIM_HandleTypeDef* timer,
    const Config& config
)
    : m_timer(timer),
      m_config(config)
{
}

bool DshotMotorOutput::Init()
{
    if (m_timer == nullptr)
    {
        m_initialized = false;
        return false;
    }

    const uint32_t dshotBitrate = static_cast<uint32_t>(m_config.speed);
    const uint32_t timerClockHz = m_config.timerClockHz != 0U
        ? m_config.timerClockHz
        : GetTimerClockHz(m_timer);

    if (dshotBitrate == 0U || timerClockHz == 0U)
    {
        m_initialized = false;
        return false;
    }

    /*
        DShot300 on the current F405 TIM8 clock:
            168 MHz / 300 kHz = 560 ticks per bit
            ARR = 559
    */
    m_bitPeriodTicks = timerClockHz / dshotBitrate;

    if (m_bitPeriodTicks < 10U)
    {
        m_initialized = false;
        return false;
    }

    /*
        DShot timing:
            bit 0: high ~37.5%
            bit 1: high ~75%
    */
    m_zeroHighTicks = (m_bitPeriodTicks * 3U) / 8U;
    m_oneHighTicks = (m_bitPeriodTicks * 6U) / 8U;

    /*
        Timer має бути:
            Prescaler = 0
            ARR = m_bitPeriodTicks - 1
    */
    __HAL_TIM_SET_AUTORELOAD(m_timer, m_bitPeriodTicks - 1U);

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        __HAL_TIM_SET_COMPARE(m_timer, kMotorChannels[i], 0U);

        const HAL_StatusTypeDef status = HAL_TIM_PWM_Start(
            m_timer,
            kMotorChannels[i]
        );

        if (status != HAL_OK)
        {
            m_initialized = false;
            return false;
        }
    }

    m_busy = false;
    m_initialized = true;

    StopAll();

    return true;
}

void DshotMotorOutput::Write(const MotorCommand& command)
{
    if (!m_initialized)
    {
        return;
    }

    /*
        Якщо DMA ще не завершив попередній DShot frame,
        краще пропустити цей write, ніж ламати сигнал.
    */
    if (IsBusy())
    {
        return;
    }

    const uint16_t motor1 = NormalizeToDshotValue(command.m1);
    const uint16_t motor2 = NormalizeToDshotValue(command.m2);
    const uint16_t motor3 = NormalizeToDshotValue(command.m3);
    const uint16_t motor4 = NormalizeToDshotValue(command.m4);

    WriteRawValues(motor1, motor2, motor3, motor4);
}

void DshotMotorOutput::StopAll()
{
    if (!m_initialized)
    {
        return;
    }

    if (IsBusy())
    {
        return;
    }

    /*
        DShot value 0 = motor stop.
    */
    WriteRawValues(0U, 0U, 0U, 0U);
}

void DshotMotorOutput::OnDmaComplete(TIM_HandleTypeDef *timer)
{
    if (timer != m_timer)
    {
        return;
    }

    StopBurstTransfer();
}

void DshotMotorOutput::WriteRawValues(
    uint16_t motor1,
    uint16_t motor2,
    uint16_t motor3,
    uint16_t motor4
)
{
    const uint16_t packet1 = BuildPacket(motor1, m_config.telemetryRequest);
    const uint16_t packet2 = BuildPacket(motor2, m_config.telemetryRequest);
    const uint16_t packet3 = BuildPacket(motor3, m_config.telemetryRequest);
    const uint16_t packet4 = BuildPacket(motor4, m_config.telemetryRequest);

    FillDmaBuffer(packet1, packet2, packet3, packet4);

    (void)StartBurstTransfer();
}

uint16_t DshotMotorOutput::NormalizeToDshotValue(float value) const
{
    value = MathUtils::Clamp01(value);

    /*
        DShot values:
            0      = stop
            1..47  = special commands
            48..2047 = throttle
    */
    if (value <= 0.001f)
    {
        return 0U;
    }

    constexpr uint16_t range = kMaxThrottleValue - kMinThrottleValue;

    return static_cast<uint16_t>(
        static_cast<float>(kMinThrottleValue) +
        value * static_cast<float>(range)
    );
}

uint16_t DshotMotorOutput::BuildPacket(
    uint16_t value,
    bool telemetryRequest
) const
{
    value &= 0x07FFU;

    /*
        12-bit payload:
            bits 11..1 = throttle / command
            bit  0     = telemetry request
    */
    uint16_t packetWithoutChecksum = static_cast<uint16_t>(value << 1U);

    if (telemetryRequest)
    {
        packetWithoutChecksum |= 0x0001U;
    }

    const uint8_t checksum = CalculateChecksum(packetWithoutChecksum);

    /*
        Final 16-bit DShot packet:
            bits 15..4 = payload
            bits 3..0  = checksum
    */
    return static_cast<uint16_t>(
        static_cast<uint16_t>(packetWithoutChecksum << 4U) | checksum
    );
}

uint8_t DshotMotorOutput::CalculateChecksum(
    uint16_t packetWithoutChecksum
) const
{
    uint16_t checksumData = packetWithoutChecksum;
    uint8_t checksum = 0U;

    for (uint8_t i = 0; i < 3U; ++i)
    {
        checksum ^= static_cast<uint8_t>(checksumData & 0x0FU);
        checksumData >>= 4U;
    }

    return static_cast<uint8_t>(checksum & 0x0FU);
}

void DshotMotorOutput::FillDmaBuffer(
    uint16_t packet1,
    uint16_t packet2,
    uint16_t packet3,
    uint16_t packet4
)
{
    const uint16_t packets[kMotorCount] =
    {
        packet1,
        packet2,
        packet3,
        packet4
    };

    /*
        DShot sends MSB first.
    */
    for (uint8_t bit = 0; bit < kPacketBits; ++bit)
    {
        const uint16_t mask = static_cast<uint16_t>(1U << (15U - bit));

        const uint32_t slotBase =
            static_cast<uint32_t>(bit) * kRegistersPerSlot;

        for (uint8_t motor = 0; motor < kMotorCount; ++motor)
        {
            m_dmaBuffer[slotBase + motor] =
                (packets[motor] & mask) != 0U
                    ? m_oneHighTicks
                    : m_zeroHighTicks;
        }
    }

    /*
        Reset / low gap після DShot frame.
        Для кожного reset slot пишемо:
            CCR1 = 0
            CCR2 = 0
            CCR3 = 0
            CCR4 = 0
    */
    for (uint8_t slot = kPacketBits; slot < kFrameSlots; ++slot)
    {
        const uint32_t slotBase =
            static_cast<uint32_t>(slot) * kRegistersPerSlot;

        m_dmaBuffer[slotBase + 0U] = 0U;
        m_dmaBuffer[slotBase + 1U] = 0U;
        m_dmaBuffer[slotBase + 2U] = 0U;
        m_dmaBuffer[slotBase + 3U] = 0U;
    }
}

bool DshotMotorOutput::StartBurstTransfer()
{
    if (m_timer == nullptr)
    {
        return false;
    }

    if (IsBusy())
    {
        return false;
    }

    SetBusy(true);

    /*
        Дуже важливо:
        DMA Burst пише не напряму в CCR1 address,
        а в TIMx_DMAR.

        TIM_DMABASE_CCR1 каже таймеру:
            починай burst з CCR1.

        TIM_DMABURSTLENGTH_4TRANSFERS каже:
            за один burst update записати CCR1, CCR2, CCR3, CCR4.

        TIM_DMA_UPDATE каже:
            запускати burst на кожен update event таймера.

        DataLength:
            kFrameSlots * 4,
            бо на кожен DShot bit slot є 4 значення: CCR1..CCR4.
    */
    const HAL_StatusTypeDef status = HAL_TIM_DMABurst_MultiWriteStart(
        m_timer,
        TIM_DMABASE_CCR1,
        TIM_DMA_UPDATE,
        m_dmaBuffer.data(),
        TIM_DMABURSTLENGTH_4TRANSFERS,
        static_cast<uint32_t>(m_dmaBuffer.size())
    );

    if (status != HAL_OK)
    {
        SetBusy(false);
        return false;
    }

    /*
        Генеруємо update event, щоб DMA стартував одразу,
        а не чекав наступний overflow.
    */
    // m_timer->Instance->EGR = TIM_EGR_UG;

    return true;
}

void DshotMotorOutput::StopBurstTransfer()
{
    if (m_timer == nullptr)
    {
        return;
    }

    (void)HAL_TIM_DMABurst_WriteStop(m_timer, TIM_DMA_UPDATE);

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        __HAL_TIM_SET_COMPARE(m_timer, kMotorChannels[i], 0U);
    }

    SetBusy(false);
}

bool DshotMotorOutput::IsBusy() const
{
    return m_busy;
}

void DshotMotorOutput::SetBusy(bool busy)
{
    m_busy = busy;
}
