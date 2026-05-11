//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/RcInput/hilrcreceiver.h"

HilRcReceiver::HilRcReceiver()
{
}

bool HilRcReceiver::Init()
{
    m_latestFrame = {};
    m_hasFrame = false;
    m_initialized = true;

    return true;
}

bool HilRcReceiver::Update(uint32_t nowUs)
{
    (void)nowUs;

    return m_initialized;
}

bool HilRcReceiver::ReadFrame(RcRawFrame& outFrame)
{
    if (!m_initialized || !m_hasFrame)
    {
        outFrame.valid = false;
        outFrame.failsafe = true;
        return false;
    }

    outFrame = m_latestFrame;
    return outFrame.valid;
}

void HilRcReceiver::SetFrame(const RcRawFrame& frame)
{
    m_latestFrame = frame;
    m_hasFrame = frame.valid;
}

void HilRcReceiver::Clear()
{
    m_latestFrame = {};
    m_hasFrame = false;
}
