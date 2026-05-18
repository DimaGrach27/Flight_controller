//
// Created by Dmytro Hrachov on 18.05.2026.
//
#pragma once

#include "main.h"
#include <cstdint>

struct MotorChannel
{
    TIM_HandleTypeDef* timer = nullptr;
    uint32_t channel = 0;
};
