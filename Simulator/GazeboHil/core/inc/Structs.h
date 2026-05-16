//
// Created by Dmytro Hrachov on 04.05.2026.
//
#pragma once

#include "GlobalDef.h"
NAMESPACE_BEGIN
struct ImuData
{
    double gyroX = 0.0;
    double gyroY = 0.0;
    double gyroZ = 0.0;

    double accelX = 0.0;
    double accelY = 0.0;
    double accelZ = 0.0;

    bool valid = false;
};

struct GroundTruthState
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    bool valid = false;
};
NAMESPACE_END