//
// Created by Dmytro Hrachov on 23.05.2026.
//
#include "FlightController/RcInput/rcreceiver.h"

#if NOT_USE_HIL
RcReceiver::RcReceiver(CrsfRcReceiver& rcReceiver)
    :m_rcReceiver(rcReceiver)
{
}
#else
RcReceiver::RcReceiver(HilRcReceiver& rcReceiver)
    :m_rcReceiver(rcReceiver)
{
}
#endif

bool RcReceiver::Init()
{
    return m_rcReceiver.Init();
}

bool RcReceiver::Update(uint32_t nowUs)
{
    return m_rcReceiver.Update(nowUs);
}

bool RcReceiver::ReadFrame(RcRawFrame &outFrame)
{
    return m_rcReceiver.ReadFrame(outFrame);
}
