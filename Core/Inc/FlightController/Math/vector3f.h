//
// Created by Dmytro Hrachov on 15.05.2026.
//
#pragma once

#include <cmath>

class Vector3f
{
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vector3f() = default;

    Vector3f(float xValue, float yValue, float zValue)
        : x(xValue)
        , y(yValue)
        , z(zValue)
    {
    }

    float Length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    float LengthSquared() const
    {
        return x * x + y * y + z * z;
    }

    bool Normalize()
    {
        const float len = Length();

        if (len < 1.0e-6f)
        {
            return false;
        }

        const float invLen = 1.0f / len;
        x *= invLen;
        y *= invLen;
        z *= invLen;

        return true;
    }

    Vector3f Normalized() const
    {
        Vector3f result = *this;
        result.Normalize();
        return result;
    }

    Vector3f operator+(const Vector3f& rhs) const
    {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    Vector3f operator-(const Vector3f& rhs) const
    {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    Vector3f operator*(float scale) const
    {
        return {x * scale, y * scale, z * scale};
    }

    Vector3f operator/(float scale) const
    {
        return {x / scale, y / scale, z / scale};
    }

    Vector3f& operator+=(const Vector3f& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vector3f& operator-=(const Vector3f& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vector3f& operator*=(float scale)
    {
        x *= scale;
        y *= scale;
        z *= scale;
        return *this;
    }

    static float Dot(const Vector3f& a, const Vector3f& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Vector3f Cross(const Vector3f& a, const Vector3f& b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }
};