//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include "FlightController/datastructs.h"

class StateEstimator
{
public:
    StateEstimator();

    void Init();

    bool UpdateImu(const ImuSample& imuSample);

    const VehicleState& GetState() const;

    void Reset();

private:
    float ComputeDtSeconds(uint32_t timestampUs);
    void InitializeFromAccel(const ImuSample& imuSample);

    float ComputeAccelRollRad(const ImuSample& imuSample) const;
    float ComputeAccelPitchRad(const ImuSample& imuSample) const;

private:
    VehicleState m_state{};

    uint32_t m_lastUpdateUs = 0;

    bool m_initialized = false;
    bool m_hasLastUpdate = false;

    float m_alpha = 0.98f;
};
