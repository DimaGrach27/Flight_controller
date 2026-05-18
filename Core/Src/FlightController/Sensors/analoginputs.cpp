//
// Created by Dmytro Hrachov on 18.05.2026.
//

#include "FlightController/Sensors/analoginputs.h"

AnalogInputs::AnalogInputs(ADC_HandleTypeDef& hadc)
    : m_hadc(hadc)
{
}

bool AnalogInputs::Start()
{
    return HAL_ADC_Start_DMA(
        &m_hadc,
        reinterpret_cast<uint32_t*>(m_dmaBuffer),
        DmaBufferSize
    ) == HAL_OK;
}

void AnalogInputs::OnDmaComplete(ADC_HandleTypeDef* hadc)
{
    if (hadc != &m_hadc)
    {
        return;
    }

    m_hasNewSamples = true;
}

void AnalogInputs::Update()
{
    if (!m_hasNewSamples)
    {
        return;
    }

    m_hasNewSamples = false;

    uint32_t sums[ChannelCount]{};

    for (uint32_t sample = 0; sample < SamplesPerChannel; ++sample)
    {
        const uint32_t baseIndex = sample * ChannelCount;

        sums[ToIndex(Channel::Vbat)] += m_dmaBuffer[baseIndex + ToIndex(Channel::Vbat)];
        sums[ToIndex(Channel::Current)] += m_dmaBuffer[baseIndex + ToIndex(Channel::Current)];
    }

    for (uint32_t i = 0; i < ChannelCount; ++i)
    {
        m_raw[i] = static_cast<uint16_t>(sums[i] / SamplesPerChannel);

        if (!m_filterInitialized)
        {
            m_filteredRaw[i] = static_cast<float>(m_raw[i]);
        }
        else
        {
            m_filteredRaw[i] += m_filterAlpha * (static_cast<float>(m_raw[i]) - m_filteredRaw[i]);
        }
    }

    m_filterInitialized = true;
}

uint16_t AnalogInputs::GetRaw(Channel channel) const
{
    return m_raw[ToIndex(channel)];
}

uint16_t AnalogInputs::GetFilteredRaw(Channel channel) const
{
    return static_cast<uint16_t>(m_filteredRaw[ToIndex(channel)]);
}

float AnalogInputs::GetVoltage(Channel channel) const
{
    constexpr float adcMax = 4095.0f;
    return static_cast<float>(GetRaw(channel)) * m_vref / adcMax;
}

float AnalogInputs::GetFilteredVoltage(Channel channel) const
{
    constexpr float adcMax = 4095.0f;
    return m_filteredRaw[ToIndex(channel)] * m_vref / adcMax;
}

bool AnalogInputs::HasNewSamples() const
{
    return m_hasNewSamples;
}

uint32_t AnalogInputs::ToIndex(Channel channel)
{
    return static_cast<uint32_t>(channel);
}