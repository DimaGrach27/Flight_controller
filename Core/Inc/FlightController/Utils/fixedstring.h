//
// Created by Dmytro Hrachov on 20.05.2026.
//
#pragma once

#include <array>
#include <cstddef>

class FixedString128
{
public:
    bool Append(const char* value)
    {
        if (length >= BufferSize - 1)
        {
            return false;
        }

        uint8_t index = 0;
        char letter = value[index];
        while (letter != '\0')
        {
            letter = value[index];
            index++;
            buffer[length] = letter;
            length++;

            if (length >= BufferSize - 1)
            {
                return false;
            }

            letter = value[index];
        }

        buffer[length] = '\0';

        return true;
    }

    bool Append(char value)
    {
        if (length >= BufferSize - 1)
        {
            return false;
        }

        buffer[length] = value;
        length++;
        buffer[length] = '\0';

        return true;
    }

    const char* CStr() const
    {
        return buffer.data();
    }

    std::size_t Size() const
    {
        return length;
    }

    std::size_t Capacity() const
    {
        return BufferSize - 1;
    }

    bool IsEmpty() const
    {
        return length == 0;
    }

    void Clear()
    {
        length = 0;
        buffer[0] = '\0';
    }

private:
    static constexpr std::size_t BufferSize = 128;

    std::array<char, BufferSize> buffer {};
    std::size_t length = 0;
};