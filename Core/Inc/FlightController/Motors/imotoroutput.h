//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include "motorcommand.h"

class IMotorOutput
{
public:
    virtual ~IMotorOutput() = default;

    virtual bool Init() = 0;

    virtual void Write(const MotorCommand& command) = 0;
    virtual void StopAll() = 0;
};
