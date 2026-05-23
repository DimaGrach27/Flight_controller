//
// Created by Dmytro Hrachov on 11.05.2026.
//
#include "FlightController/RcInput/crsfrcreceiver.h"

namespace
{
    /*
        CRSF device addresses.

        Для receiver -> flight controller RC packets часто address = 0xC8.
        Але в потоці можуть бути й інші CRSF frames, тому parser допускає кілька
        відомих адрес і просто ігнорує не-RC frames.
    */
    constexpr uint8_t CrsfAddressBroadcast = 0x00;
    constexpr uint8_t CrsfAddressUsb = 0x10;
    constexpr uint8_t CrsfAddressTbsCorePnpPro = 0x80;
    constexpr uint8_t CrsfAddressReserved1 = 0x8A;
    constexpr uint8_t CrsfAddressCurrentSensor = 0xC0;
    constexpr uint8_t CrsfAddressGps = 0xC2;
    constexpr uint8_t CrsfAddressTbsBlackbox = 0xC4;
    constexpr uint8_t CrsfAddressFlightController = 0xC8;
    constexpr uint8_t CrsfAddressReserved2 = 0xCA;
    constexpr uint8_t CrsfAddressRaceTag = 0xCC;
    constexpr uint8_t CrsfAddressRadioTransmitter = 0xEA;

    /*
        CRSF packed RC channels frame type.

        Frame layout:
        [address][length][type][payload...][crc]

        Для RC channels:
        type    = 0x16
        payload = 22 bytes
        crc     = 1 byte

        length = type + payload + crc = 1 + 22 + 1 = 24
    */
    constexpr uint8_t CrsfFrameTypeRcChannelsPacked = 0x16;

    constexpr uint8_t CrsfRcChannelPayloadSize = 22;
    constexpr uint8_t CrsfRcChannelCount = 16;
    constexpr uint8_t CrsfRcChannelsLength = 24;

    constexpr uint8_t CrsfMinLength = 2;

    /*
        CRSF CRC-8 DVB-S2 polynomial.
    */
    constexpr uint8_t CrcPolynomial = 0xD5;
}

CrsfRcReceiver::CrsfRcReceiver(UartByteStream& byteStream)
    : m_byteStream(byteStream)
{
}

bool CrsfRcReceiver::Init()
{
    m_latestFrame = {};
    m_parserCount = 0;
    m_hasFrame = false;

    if (!m_byteStream.Init())
    {
        m_initialized = false;
        return false;
    }

    m_initialized = true;
    return true;
}

bool CrsfRcReceiver::Update(uint32_t nowUs)
{
    if (!m_initialized)
    {
        return false;
    }

    constexpr uint16_t MaxBytesPerUpdate = 64;

    uint16_t processed = 0;
    uint8_t dataByte = 0;
    while (processed < MaxBytesPerUpdate && m_byteStream.ReadByte(dataByte))
    {
        PushByte(dataByte);
        processed++;
    }

    return TryParseBuffer(nowUs);
}

bool CrsfRcReceiver::ReadFrame(RcRawFrame& outFrame)
{
    if (!m_initialized || !m_hasFrame)
    {
        outFrame = {};
        outFrame.valid = false;
        outFrame.failsafe = true;
        return false;
    }

    outFrame = m_latestFrame;
    return outFrame.valid && !outFrame.failsafe;
}

void CrsfRcReceiver::PushByte(uint8_t byte)
{
    if (m_parserCount >= MaxParserBufferSize)
    {
        /*
            Якщо parser buffer переповнився, значить ми не встигаємо
            або в потоці сміття. Скидаємо буфер, щоб відновити sync.
        */
        m_parserCount = 0;
    }

    m_parserBuffer[m_parserCount] = byte;
    ++m_parserCount;
}

void CrsfRcReceiver::PushBytes(const uint8_t* data, uint16_t size)
{
    if (data == nullptr || size == 0)
    {
        return;
    }

    for (uint16_t i = 0; i < size; ++i)
    {
        PushByte(data[i]);
    }
}

bool CrsfRcReceiver::TryParseBuffer(uint32_t nowUs)
{
    bool parsedAnyFrame = false;

    /*
        Мінімальний CRSF frame:
        address + length + type + crc
        тобто хоча б 4 bytes.
    */
    while (m_parserCount >= 4)
    {
        bool foundAddress = false;
        uint8_t addressOffset = 0;

        /*
            Шукаємо перший байт, який може бути CRSF address.
            Це дозволяє відновитися, якщо в потоці було сміття або ми
            стартували читати з середини frame.
        */
        for (uint8_t i = 0; i < m_parserCount; ++i)
        {
            if (IsKnownAddress(m_parserBuffer[i]))
            {
                foundAddress = true;
                addressOffset = i;
                break;
            }
        }

        if (!foundAddress)
        {
            m_parserCount = 0;
            return parsedAnyFrame;
        }

        if (addressOffset > 0)
        {
            RemoveBytesFromBuffer(addressOffset);
        }

        if (m_parserCount < 4)
        {
            return parsedAnyFrame;
        }

        const uint8_t length = m_parserBuffer[1];

        /*
            length включає:
            type + payload + crc

            total frame size:
            address + length byte + length
            тобто length + 2.
        */
        if (length < CrsfMinLength || length > MaxFrameSize)
        {
            /*
                Якщо length явно поганий — цей address скоріш за все був
                випадковим байтом. Зсуваємось на 1 і шукаємо далі.
            */
            RemoveBytesFromBuffer(1);
            continue;
        }

        const uint8_t totalFrameSize = static_cast<uint8_t>(length + 2U);

        /*
            Ще не весь frame прийшов.
            Чекаємо наступний Update().
        */
        if (m_parserCount < totalFrameSize)
        {
            return parsedAnyFrame;
        }

        if (TryParseFrameAt(0, nowUs))
        {
            parsedAnyFrame = true;
            RemoveBytesFromBuffer(totalFrameSize);
        }
        else
        {
            /*
                CRC не зійшовся або frame невалідний.
                Не скидаємо все, а зсуваємось на 1 байт, щоб знайти наступний
                можливий CRSF frame.
            */
            RemoveBytesFromBuffer(1);
        }
    }

    return parsedAnyFrame;
}

bool CrsfRcReceiver::TryParseFrameAt(uint8_t offset, uint32_t nowUs)
{
    if (offset + 4U > m_parserCount)
    {
        return false;
    }

    const uint8_t address = m_parserBuffer[offset + 0U];
    const uint8_t length = m_parserBuffer[offset + 1U];

    if (!IsKnownAddress(address))
    {
        return false;
    }

    if (length < CrsfMinLength || length > MaxFrameSize)
    {
        return false;
    }

    const uint8_t totalFrameSize = static_cast<uint8_t>(length + 2U);

    if (offset + totalFrameSize > m_parserCount)
    {
        return false;
    }

    const uint8_t type = m_parserBuffer[offset + 2U];

    /*
        CRC рахується по:
        type + payload

        Тобто:
        - address не входить
        - length не входить
        - сам crc byte не входить

        length = type + payload + crc
        тому кількість байтів для CRC = length - 1.
    */
    const uint8_t crcIndex = static_cast<uint8_t>(
        offset + totalFrameSize - 1U
    );

    const uint8_t expectedCrc = m_parserBuffer[crcIndex];

    const uint8_t computedCrc = ComputeCrc8DvbS2(
        &m_parserBuffer[offset + 2U],
        static_cast<uint8_t>(length - 1U)
    );

    if (computedCrc != expectedCrc)
    {
        return false;
    }

    /*
        CRC валідний. Якщо це не RC channels frame — просто вважаємо frame
        успішно розпарсеним і викидаємо його з буфера.
    */
    if (type != CrsfFrameTypeRcChannelsPacked)
    {
        return true;
    }

    if (length != CrsfRcChannelsLength)
    {
        return false;
    }

    const uint8_t* payload = &m_parserBuffer[offset + 3U];

    return DecodeRcChannels(
        payload,
        CrsfRcChannelPayloadSize,
        nowUs
    );
}

bool CrsfRcReceiver::DecodeRcChannels(
    const uint8_t* payload,
    uint8_t payloadSize,
    uint32_t nowUs
)
{
    if (payload == nullptr)
    {
        return false;
    }

    if (payloadSize < CrsfRcChannelPayloadSize)
    {
        return false;
    }

    RcRawFrame frame{};

    frame.channelCount = CrsfRcChannelCount;

    for (uint8_t i = 0; i < CrsfRcChannelCount; ++i)
    {
        frame.channels[i] = ReadPacked11BitChannel(
            payload,
            payloadSize,
            i
        );
    }

    frame.timestampUs = nowUs;
    frame.failsafe = false;
    frame.valid = true;

    m_latestFrame = frame;
    m_hasFrame = true;

    return true;
}

uint16_t CrsfRcReceiver::ReadPacked11BitChannel(
    const uint8_t* payload,
    uint8_t payloadSize,
    uint8_t channelIndex
) const
{
    /*
        CRSF channels packed:
        16 каналів по 11 біт.

        channel 0: bits 0..10
        channel 1: bits 11..21
        channel 2: bits 22..32
        etc.

        Для кожного каналу беремо 3 байти навколо потрібного bitIndex,
        зсуваємо вправо на bitOffset і маскуємо 11 біт.
    */
    const uint16_t bitIndex =
        static_cast<uint16_t>(channelIndex) * 11U;

    const uint8_t byteIndex =
        static_cast<uint8_t>(bitIndex / 8U);

    const uint8_t bitOffset =
        static_cast<uint8_t>(bitIndex % 8U);

    uint32_t value = 0;

    if (byteIndex < payloadSize)
    {
        value |= static_cast<uint32_t>(payload[byteIndex]);
    }

    if (byteIndex + 1U < payloadSize)
    {
        value |= static_cast<uint32_t>(payload[byteIndex + 1U]) << 8U;
    }

    if (byteIndex + 2U < payloadSize)
    {
        value |= static_cast<uint32_t>(payload[byteIndex + 2U]) << 16U;
    }

    return static_cast<uint16_t>((value >> bitOffset) & 0x07FFU);
}

uint8_t CrsfRcReceiver::ComputeCrc8DvbS2(
    const uint8_t* data,
    uint8_t size
) const
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < size; ++i)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = static_cast<uint8_t>(
                    static_cast<uint8_t>(crc << 1U) ^ CrcPolynomial
                );
            }
            else
            {
                crc = static_cast<uint8_t>(crc << 1U);
            }
        }
    }

    return crc;
}

bool CrsfRcReceiver::IsKnownAddress(uint8_t address) const
{
    switch (address)
    {
        case CrsfAddressBroadcast:
        case CrsfAddressUsb:
        case CrsfAddressTbsCorePnpPro:
        case CrsfAddressReserved1:
        case CrsfAddressCurrentSensor:
        case CrsfAddressGps:
        case CrsfAddressTbsBlackbox:
        case CrsfAddressFlightController:
        case CrsfAddressReserved2:
        case CrsfAddressRaceTag:
        case CrsfAddressRadioTransmitter:
            return true;

        default:
            return false;
    }
}

void CrsfRcReceiver::RemoveBytesFromBuffer(uint8_t count)
{
    if (count == 0)
    {
        return;
    }

    if (count >= m_parserCount)
    {
        m_parserCount = 0;
        return;
    }

    const uint8_t remaining = static_cast<uint8_t>(m_parserCount - count);

    for (uint8_t i = 0; i < remaining; ++i)
    {
        m_parserBuffer[i] = m_parserBuffer[i + count];
    }

    m_parserCount = remaining;
}