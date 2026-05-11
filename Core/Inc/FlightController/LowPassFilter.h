//
// Created by Dmytro Hrachov on 10.05.2026.
//
#pragma once

class LowPassFilter
{
public:
    explicit LowPassFilter(float alpha);

    float Update(float value);
    void Reset(float value = 0.0f);

private:
    float m_alpha = 0.9f;
    float m_state = 0.0f;
};