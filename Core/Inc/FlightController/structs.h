//
// Created by Dmytro Hrachov on 01.05.2026.
//
#pragma once

struct Vector3
{
    float x;
    float y;
    float z;
};

struct SimImuSample
{
    Vector3 gyro;
    Vector3 accel;
    bool valid;
};

struct MotorOutputs
{
    float m1;
    float m2;
    float m3;
    float m4;
};

struct RcCommand
{
    float throttle; // 0..1
    float roll;     // -1..1
    float pitch;    // -1..1
    float yaw;      // -1..1
    bool armed;
    bool acroMode;
    bool valid;
};

struct ControlOutput
{
    float roll;
    float pitch;
    float yaw;
};