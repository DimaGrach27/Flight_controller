//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include "ircreceiver.h"

class HilRcReceiver final : public IRcReceiver
{
public:
    HilRcReceiver();

    bool Init() override;
    bool Update(uint32_t nowUs) override;
    bool ReadFrame(RcRawFrame& outFrame) override;

    void SetFrame(const RcRawFrame& frame);
    void Clear();

private:
    RcRawFrame m_latestFrame{};

    bool m_hasFrame = false;
    bool m_initialized = false;
};
