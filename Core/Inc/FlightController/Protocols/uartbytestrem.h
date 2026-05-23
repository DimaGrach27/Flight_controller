//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <cstdint>

#include "stm32uartdmabytestream.h"

class UartByteStream
{
public:
    explicit UartByteStream(Stm32UartDmaByteStream& byteStream)
        : m_byteStream(byteStream)
    {

    }

    bool Init()
    {
        return m_byteStream.Init();
    }

    bool ReadByte(uint8_t& byte)
    {
        return m_byteStream.ReadByte(byte);
    }

private:
    Stm32UartDmaByteStream& m_byteStream;
};
