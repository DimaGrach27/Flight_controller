//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "FlightController/RcInput/rcrawframe.h"

#include <cstdint>

class IRcReceiver
{
public:
    virtual ~IRcReceiver() = default;

    virtual bool Init() = 0;
    virtual bool Update(uint32_t nowUs) = 0;

    virtual bool ReadFrame(RcRawFrame& outFrame) = 0;
};
