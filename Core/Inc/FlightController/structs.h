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
    float m1; //LEFT_FRONT
    float m2; //RIGHT_FRONT
    float m3; //RIGHT_BACK
    float m4; //LEFT_BACK
};