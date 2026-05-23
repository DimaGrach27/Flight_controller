//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include <cstdint>

#include "rcrawframe.h"


class HilRcReceiver
{
public:
    HilRcReceiver();

    bool Init();
    bool Update(uint32_t nowUs);
    bool ReadFrame(RcRawFrame& outFrame);

    void SetFrame(const RcRawFrame& frame);
    void Clear();

private:
    RcRawFrame m_latestFrame{};

    bool m_hasFrame = false;
    bool m_initialized = false;
};
