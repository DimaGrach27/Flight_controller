//
// Created by Dmytro Hrachov on 10.05.2026.
//
#pragma once

class LowPassFilter
{
public:
    LowPassFilter();
    void Init(float cutoffHz);

    float Update(float input, float dt);

    void Reset(float value = 0.0f);

    bool IsInitialized() const;

private:
    float ComputeAlpha(float dt) const;

private:
    float m_cutoffHz = 30.0f;
    float m_state = 0.0f;

    bool m_initialized = false;
};