//
// Created by Dmytro Hrachov on 12.05.2026.
//
#pragma once
#include "lowpassfilter.h"
#include "FlightController/structs.h"

class Vector3LowPassFilter
{
public:
    Vector3LowPassFilter();

    void Init(float cutoffHz);

    Vector3 Update(const Vector3& input, float dt);

    void Reset(const Vector3& value = {});

private:
    LowPassFilter m_xFilter;
    LowPassFilter m_yFilter;
    LowPassFilter m_zFilter;
};
