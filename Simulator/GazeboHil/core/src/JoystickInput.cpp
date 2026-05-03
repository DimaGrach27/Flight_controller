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
    double axisRoll = Axis(0);
    double axisPitch = Axis(1);
    double axisThrottle = Axis(2);
    double axisYaw = Axis(3);

    m_control.roll = ApplyExpo(ApplyDeadzone(axisRoll, 0.04), 0.3);
    m_control.pitch = ApplyExpo(ApplyDeadzone(axisPitch, 0.04), 0.3);
    m_control.yaw = -ApplyExpo(ApplyDeadzone(axisYaw, 0.04), 0.3);
    m_control.throttle = NormalizeThrottle(axisThrottle);

    m_control.valid = true;

    // Кнопка 0 як arm для майбутнього. Поки можна ігнорувати.
    SDL_Joystick* js = static_cast<SDL_Joystick*>(m_joystick);

    // std::cout << "Name: " << SDL_JoystickName(js) << "\n";
    // std::cout << "Axes: " << SDL_JoystickNumAxes(js) << "\n";
    // std::cout << "Buttons: " << SDL_JoystickNumButtons(js) << "\n";
    // std::cout << "Hats: " << SDL_JoystickNumHats(js) << "\n";

    // std::cout << "\033[2J\033[H"; // clear terminal

    // std::cout << "Axes:\n";
    // for (int i = 0; i < SDL_JoystickNumAxes(js); ++i)
    // {
    //     Sint16 value = SDL_JoystickGetAxis(js, i);
    //     std::cout << "  Axis " << i << ": " << value << "\n";
    // }

    // std::cout << "\nButtons:\n";
    // for (int i = 0; i < SDL_JoystickNumButtons(js); ++i)
    // {
    //     Uint8 value = SDL_JoystickGetButton(js, i);
    //     std::cout << "  Button " << i << ": " << static_cast<int>(value) << "\n";
    // }
    //
    // std::cout << "\nHats:\n";
    // for (int i = 0; i < SDL_JoystickNumHats(js); ++i)
    // {
    //     Uint8 value = SDL_JoystickGetHat(js, i);
    //     std::cout << "  Hat " << i << ": " << static_cast<int>(value) << "\n";
    // }

    if (SDL_JoystickNumButtons(js) > 0)
    {
        bool btn_0 = SDL_JoystickGetButton(js, 0);
        bool btn_1 = SDL_JoystickGetButton(js, 1);
        bool btn_2 = SDL_JoystickGetButton(js, 2);
        bool btn_3 = SDL_JoystickGetButton(js, 3);
        bool btn_4 = SDL_JoystickGetButton(js, 4);
        // std::cout
        //     << "[Joystick] "
        //     <<  "BTN_0=" << btn_0
        //     << " BTN_1=" << btn_1
        //     << " BTN_2=" << btn_2
        //     << " BTN_3=" << btn_3
        //     << " BTN_4=" << btn_4
        // << std::endl;

        //arm button it is B on controller and 6 axis in code 1 = 32768
        //acro mode it is E on controller and 5 axis in code 1 = 32768
        m_control.arm = SDL_JoystickGetAxis(js, 6) > 16000;
        m_control.acroMode = SDL_JoystickGetAxis(js, 5) > 16000;
    }
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

    double sign = value > 0.0 ? 1.0 : -1.0;
    double normalized = (std::abs(value) - deadzone) / (1.0 - deadzone);

    return sign * std::clamp(normalized, 0.0, 1.0);
}

double JoystickInput::ApplyExpo(double value, double expo)
{
    return (1.0 - expo) * value + expo * value * value * value;
}

double JoystickInput::NormalizeThrottle(double value)
{
    // SDL axis зазвичай -1..1.
    // Перетворюємо в 0..1.
    double throttle = (value + 1.0) * 0.5;
    return std::clamp(throttle, 0.0, 1.0);
}
NAMESPACE_END