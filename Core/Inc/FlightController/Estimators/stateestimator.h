//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "FlightController/datastructs.h"
#include "FlightController/Estimators/ahrs.h"

class StateEstimator
{
public:
    StateEstimator();

    void Init();

    bool UpdateImu(const ImuSample& imuSample);

    const VehicleState& GetState() const;

    void Reset();

private:
    VehicleState m_state{};
    Ahrs m_ahrs;
};
