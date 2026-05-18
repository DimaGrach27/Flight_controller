//
// Created by Dmytro Hrachov on 18.05.2026.
//
#pragma once

#include <cstdint>
#include "main.h"

class AnalogInputs
{
public:
    enum class Channel : uint16_t
    {
        Vbat = 0,
        Current = 1,
    };

    static constexpr uint16_t ChannelCount = 2; //2
    static constexpr uint16_t SamplesPerChannel = 32;
    static constexpr uint16_t DmaBufferSize = ChannelCount * SamplesPerChannel;

public:
    explicit AnalogInputs(ADC_HandleTypeDef& hadc);

    bool Start();

    void OnDmaComplete(ADC_HandleTypeDef* hadc);
    void Update();

    uint16_t GetRaw(Channel channel) const;
    uint16_t GetFilteredRaw(Channel channel) const;

    float GetVoltage(Channel channel) const;
    float GetFilteredVoltage(Channel channel) const;

    bool HasNewSamples() const;

private:
    static uint32_t ToIndex(Channel channel);

private:
    ADC_HandleTypeDef& m_hadc;

    uint16_t m_dmaBuffer[DmaBufferSize]{};

    uint16_t m_raw[ChannelCount]{};
    float m_filteredRaw[ChannelCount]{};

    bool m_hasNewSamples = false;
    bool m_filterInitialized = false;

    float m_filterAlpha = 0.1f;
    float m_vref = 3.3f;
};
