//
// Created by Dmytro Hrachov on 12.05.2026.
//
#pragma once
#include "lowpassfilter.h"
#include "FlightController/structs.h"
#include "FlightController/Math/Vector3f.h"

class Vector3LowPassFilter
{
public:
    Vector3LowPassFilter();

    void Init(float cutoffHz);

    Vector3f Update(const Vector3f& input, float dt);

    void Reset(const Vector3f& value = {});

private:
    LowPassFilter m_xFilter;
    LowPassFilter m_yFilter;
    LowPassFilter m_zFilter;
};
