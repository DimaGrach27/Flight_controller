//
// Created by Dmytro Hrachov on 24.05.2026.
//
#pragma once
#include "FlightController/datastructs.h"

class ImuAxisMapper
{
public:
    Vector3 MapAccel(const Vector3& raw) const
    {
        Vector3 tempAccel = raw;
        Vector3 accel = {};

        accel.x = tempAccel.y;
        accel.y = tempAccel.x;
        accel.z = tempAccel.z;

#if NOT_USE_HIL
        accel.x *= -1;
        accel.y *= -1;
        accel.z *= 1;
#endif
        return accel;
    }

    Vector3 MapGyro(const Vector3& raw) const
    {
        Vector3 tempGyro = raw;
        Vector3 gyro = {};

        gyro.x = tempGyro.y;
        gyro.y = tempGyro.x;
        gyro.z = tempGyro.z;

#if NOT_USE_HIL
        gyro.x *= -1;
        gyro.y *= -1;
        gyro.z *= 1;
#endif

        return gyro;
    }

private:
    // Vector3 MapSensorToBody(const Vector3& raw) const;
};