//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <cstdint>

#include "stm32uartdmarxstream.h"
#include "stm32uartdmatxstream.h"

class UartByteStream
{
public:
    explicit UartByteStream(Stm32UartDmaRxStream& byteRxStream,
        Stm32UartDmaTxStream& byteTxStream)
        : m_byteRxStream(byteRxStream)
        , m_byteTxStream(byteTxStream)
    {

    }

    bool Init()
    {
        return m_byteRxStream.Init() && m_byteTxStream.Init();
    }

    bool ReadByte(uint8_t& byte)
    {
        return m_byteRxStream.ReadByte(byte);
    }

    bool Write(const uint8_t* data, uint16_t size)
    {
        return m_byteTxStream.Write(data, size);
    }

    bool WriteString(const char* str, uint16_t size)
    {
        return m_byteTxStream.WriteString(str, size);
    }

private:
    Stm32UartDmaRxStream& m_byteRxStream;
    Stm32UartDmaTxStream& m_byteTxStream;
};
