//
// Created by Dmytro Hrachov on 15.05.2026.
//
#pragma once

#include <cmath>
#include "vector3f.h"

struct EulerAngles
{
    float rollRad = 0.0f;
    float pitchRad = 0.0f;
    float yawRad = 0.0f;
};

class Quaternion
{
public:
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;

    Quaternion(float wValue, float xValue, float yValue, float zValue)
        : w(wValue)
        , x(xValue)
        , y(yValue)
        , z(zValue)
    {
    }

    void Normalize()
    {
        const float norm = std::sqrt(w * w + x * x + y * y + z * z);

        if (norm < 1.0e-6f)
        {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
            return;
        }

        const float invNorm = 1.0f / norm;
        w *= invNorm;
        x *= invNorm;
        y *= invNorm;
        z *= invNorm;
    }

    Quaternion Conjugated() const
    {
        return {w, -x, -y, -z};
    }

    Quaternion operator*(const Quaternion& rhs) const
    {
        return {
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
        };
    }

    Vector3f RotateBodyToWorld(const Vector3f& v) const
    {
        const Quaternion qv(0.0f, v.x, v.y, v.z);
        const Quaternion result = (*this) * qv * Conjugated();

        return {result.x, result.y, result.z};
    }

    Vector3f RotateWorldToBody(const Vector3f& v) const
    {
        const Quaternion qv(0.0f, v.x, v.y, v.z);
        const Quaternion result = Conjugated() * qv * (*this);

        return {result.x, result.y, result.z};
    }

    EulerAngles ToEuler() const
    {
        EulerAngles euler{};

        const float sinrCosp = 2.0f * (w * x + y * z);
        const float cosrCosp = 1.0f - 2.0f * (x * x + y * y);
        euler.rollRad = std::atan2(sinrCosp, cosrCosp);

        const float sinp = 2.0f * (w * y - z * x);
        if (std::fabs(sinp) >= 1.0f)
        {
            euler.pitchRad = std::copysign(M_1_PI / 2.0f, sinp);
        }
        else
        {
            euler.pitchRad = std::asin(sinp);
        }

        const float sinyCosp = 2.0f * (w * z + x * y);
        const float cosyCosp = 1.0f - 2.0f * (y * y + z * z);
        euler.yawRad = std::atan2(sinyCosp, cosyCosp);

        return euler;
    }
};