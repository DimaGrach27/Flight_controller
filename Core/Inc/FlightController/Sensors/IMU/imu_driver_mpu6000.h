//
// Created by Dmytro Hrachov on 10.05.2026.
//
#pragma once

#include <cstdint>

#include "main.h"

#include "FlightController/datastructs.h"
#include "FlightController/Protocols/spidmabus.h"

class IMU_MPU6000
{
public:
    struct DebugInfo
    {
        uint8_t whoAmI = 0;
        uint32_t readOkCount = 0;
        uint32_t readFailCount = 0;
        uint32_t repeatedFrameCount = 0;
        uint8_t sampleRateDivider = 0;
        uint8_t config = 0;
        uint8_t gyroConfig = 0;
        uint8_t accelConfig = 0;
        uint8_t userControl = 0;
        uint8_t powerManagement1 = 0;
        uint8_t powerManagement2 = 0;
        ImuRawData lastRaw{};
    };

    static constexpr uint8_t RawFrameSize = 14;
    static constexpr uint8_t SpiFrameSize = RawFrameSize + 1;

    explicit IMU_MPU6000(SpiDmaBus& spiBus);

    bool Init();

    bool StartReadRaw();
    bool IsReadComplete() const;
    bool HasError() const;

    void Reset();

    bool ReadRaw(ImuRawData& outRawData, uint32_t nowUs);
    bool Read(ImuSample& outData, uint32_t nowUs);

    const DebugInfo& GetDebugInfo() const;

private:
    bool CheckDeviceId();
    bool ConfigureDevice();
    bool WriteAndVerify(uint8_t reg, uint8_t value);
    void ReadDebugRegisters();

    int16_t MakeInt16(uint8_t high, uint8_t low) const;

private:
    SpiDmaBus& m_spiBus;

    bool m_initialized = false;

    float m_accelScale_mps2 = 0.0f;
    float m_gyroScale_rads = 0.0f;

    uint8_t m_txBuffer[SpiFrameSize] = {};
    uint8_t m_rxBuffer[SpiFrameSize] = {};
    uint8_t m_lastRawFrame[RawFrameSize] = {};
    bool m_hasLastRawFrame = false;

    DebugInfo m_debugInfo{};
};
