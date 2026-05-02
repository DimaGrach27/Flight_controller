//
// Created by Dmytro Hrachov on 02.05.2026.
//
#include "JoystickInput.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <iostream>

NAMESPACE_BEGIN
bool JoystickInput::Init(int joystickIndex)
{
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) != 0)
    {
        std::cerr << "[JoystickInput] SDL init failed: "
                  << SDL_GetError() << std::endl;
        return false;
    }

    int count = SDL_NumJoysticks();

    std::cout << "[JoystickInput] joystick count: " << count << std::endl;

    if (count <= 0)
    {
        std::cerr << "[JoystickInput] No joystick found" << std::endl;
        return false;
    }

    SDL_Joystick* js = SDL_JoystickOpen(joystickIndex);

    if (!js)
    {
        std::cerr << "[JoystickInput] Failed to open joystick: "
                  << SDL_GetError() << std::endl;
        return false;
    }

    m_joystick = js;

    std::cout << "[JoystickInput] Opened: "
              << SDL_JoystickName(js)
              << " axes=" << SDL_JoystickNumAxes(js)
              << " buttons=" << SDL_JoystickNumButtons(js)
              << std::endl;

    return true;
}

void JoystickInput::Shutdown()
{
    if (m_joystick)
    {
        SDL_JoystickClose(static_cast<SDL_Joystick*>(m_joystick));
        m_joystick = nullptr;
    }

    SDL_QuitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER);
}

void JoystickInput::Poll()
{
    if (!m_joystick)
        return;

    SDL_JoystickUpdate();

    // Початковий mapping. Його, скоріш за все, треба буде підправити під TX12.
    double axis0 = Axis(0);
    double axis1 = Axis(1);
    double axis2 = Axis(2);
    double axis3 = Axis(3);

    m_control.roll = ApplyDeadzone(axis0, 0.03);
    m_control.pitch = ApplyDeadzone(axis1, 0.03);
    m_control.throttle = NormalizeThrottle(axis2);
    m_control.yaw = ApplyDeadzone(axis3, 0.03);

    m_control.valid = true;

    // Кнопка 0 як arm для майбутнього. Поки можна ігнорувати.
    SDL_Joystick* js = static_cast<SDL_Joystick*>(m_joystick);

    if (SDL_JoystickNumButtons(js) > 0)
        m_control.arm = SDL_JoystickGetButton(js, 0) != 0;
}

const ManualControl& JoystickInput::Control() const
{
    return m_control;
}

double JoystickInput::Axis(int index) const
{
    SDL_Joystick* js = static_cast<SDL_Joystick*>(m_joystick);

    if (!js)
        return 0.0;

    if (index < 0 || index >= SDL_JoystickNumAxes(js))
        return 0.0;

    Sint16 raw = SDL_JoystickGetAxis(js, index);

    double value = 0.0;

    if (raw >= 0)
        value = static_cast<double>(raw) / 32767.0;
    else
        value = static_cast<double>(raw) / 32768.0;

    return std::clamp(value, -1.0, 1.0);
}

double JoystickInput::ApplyDeadzone(double value, double deadzone)
{
    if (std::abs(value) < deadzone)
        return 0.0;

    return std::clamp(value, -1.0, 1.0);
}

double JoystickInput::NormalizeThrottle(double value)
{
    // SDL axis зазвичай -1..1.
    // Перетворюємо в 0..1.
    double throttle = (value + 1.0) * 0.5;
    return std::clamp(throttle, 0.0, 1.0);
}
NAMESPACE_END