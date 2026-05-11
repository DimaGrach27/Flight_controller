//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <cstdint>

class IUartByteStream
{
public:
    virtual ~IUartByteStream() = default;

    virtual bool Init() = 0;

    virtual uint16_t Read(uint8_t* outData, uint16_t maxSize) = 0;
};