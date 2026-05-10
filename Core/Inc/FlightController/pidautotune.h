//
// Created by Dmytro Hrachov on 10.05.2026.
//
#pragma once

#include <cstdint>

#include "structs.h"

class PidAutoTune
{
public:
    enum class State
    {
        Idle,
        Running,
        Finished,
        Failed
    };

    struct Config
    {
        // Вихід autotune у твоїх одиницях motor correction.
        // Якщо mixer приймає correction приблизно -1..1, почни з 0.03 - 0.08.
        float relayAmplitude = 0.05f;

        // Мертва зона по gyro rate, rad/s.
        // 2 deg/s ≈ 0.035 rad/s.
        float hysteresis = 0.035f;

        // Мінімальна амплітуда відповіді, rad/s.
        float minOscillationAmplitude = 0.15f;

        // Максимальна амплітуда відповіді, rad/s.
        // Захист, щоб не розкрутити стенд.
        float maxSafeRate = 3.5f;

        // Скільки повних періодів зібрати.
        uint8_t periodsToCollect = 6;

        // Максимальний час тюнінгу однієї осі.
        float timeoutSec = 10.0f;

        // Обмеження на готові gains.
        float maxKp = 1.0f;
        float maxKi = 5.0f;
        float maxKd = 0.1f;
    };

public:
    explicit PidAutoTune(const Config& config)
        : m_config(config)
    {
    }

    void Start();
    void Stop();

    float Update(float measuredRateRadSec, float dt);

    State GetState() const;
    PidGains GetResult() const;

    bool IsFinished() const;
    bool IsFailed() const;

private:
    void UpdateRelay(float measuredRateRadSec);
    void UpdatePeaks(float measuredRateRadSec);
    void DetectPeriod(float measuredRateRadSec);
    float CalculateCurrentAmplitude() const;
    void CalculateGains();

private:
    Config m_config;

    State m_state = State::Idle;

    float m_timeSec = 0.0f;
    float m_output = 0.0f;

    float m_lastRate = 0.0f;

    float m_positivePeak = 0.0f;
    float m_negativePeak = 0.0f;

    bool m_havePositivePeak = false;
    bool m_haveNegativePeak = false;

    float m_lastPositiveCrossTime = -1.0f;

    float m_periodSum = 0.0f;
    uint8_t m_periodCount = 0;

    float m_amplitudeSum = 0.0f;
    uint8_t m_amplitudeCount = 0;

    PidGains m_result;
};