//
// Created by Dmytro Hrachov on 23.05.2026.
//
#pragma once

#include "rcrawframe.h"
#include <cstdint>

#include "crsfrcreceiver.h"
#include "hilrcreceiver.h"

#include "FlightController/globaldef.h"

class RcReceiver
{
public:
#if NOT_USE_HIL
    RcReceiver(CrsfRcReceiver& rcReceiver);
#else
    RcReceiver(HilRcReceiver& rcReceiver);
#endif

    bool Init();
    bool Update(uint32_t nowUs);

    bool ReadFrame(RcRawFrame& outFrame);

private:
#if NOT_USE_HIL
    CrsfRcReceiver& m_rcReceiver;
#else
    HilRcReceiver& m_rcReceiver;
#endif
};
