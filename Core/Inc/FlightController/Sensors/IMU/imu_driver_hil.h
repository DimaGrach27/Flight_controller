//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include "FlightController/datastructs.h"

class IMU_Driver_Hil
{
public:
    IMU_Driver_Hil();

    bool Init();
    bool ReadRaw(ImuRawData& outRawData, uint32_t nowUs);
    bool Read(ImuSample& outData, uint32_t nowUs);

    void SetHilData(const ImuSample& data);

private:
    ImuSample m_latestData{};

    bool m_hasData = false;
    bool m_initialized = false;
};
