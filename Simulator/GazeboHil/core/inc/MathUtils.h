//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include "GlobalDef.h"
#include <algorithm>

NAMESPACE_BEGIN

inline double Clamp(double value, double minValue, double maxValue)
{
    return std::max(minValue, std::min(maxValue, value));
}

NAMESPACE_END