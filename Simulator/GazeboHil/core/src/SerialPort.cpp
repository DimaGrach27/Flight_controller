//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "SerialPort.h"

#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#ifdef __APPLE__
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>
#endif

NAMESPACE_BEGIN
static speed_t BaudToTermios(int baud)
{
    switch (baud)
    {
        case 9600: return B9600;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return B115200;
    }
}

SerialPort::~SerialPort()
{
    Close();
}

bool SerialPort::Open(const std::string& path, int baud)
{
    Close();

    fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ < 0)
    {
        std::cerr << "[SerialPort] Failed to open: " << path << std::endl;
        return false;
    }

    termios tty{};

    if (tcgetattr(fd_, &tty) != 0)
    {
        std::cerr << "[SerialPort] tcgetattr failed" << std::endl;
        Close();
        return false;
    }

    cfmakeraw(&tty);

    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);

#ifdef CRTSCTS
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    speed_t speed = BaudToTermios(baud);

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "[SerialPort] tcsetattr failed" << std::endl;
        Close();
        return false;
    }

#ifdef __APPLE__
    // macOS: set custom baud rates like 460800 / 921600 / 1000000.
    if (baud > 230400)
    {
        speed_t customBaud = static_cast<speed_t>(baud);

        if (ioctl(fd_, IOSSIOSPEED, &customBaud) == -1)
        {
            std::cerr << "[SerialPort] ioctl on " << customBaud << " failed" << std::endl;
            return false;
        }
    }
#endif

    tcflush(fd_, TCIOFLUSH);

    std::cout << "[SerialPort] Opened: " << path << " @ " << baud << std::endl;

    return true;
}

void SerialPort::Close()
{
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::IsOpen() const
{
    return fd_ >= 0;
}

int SerialPort::Read(uint8_t* buffer, size_t maxLen)
{
    if (fd_ < 0)
        return -1;

    ssize_t n = read(fd_, buffer, maxLen);

    if (n <= 0)
        return 0;

    return static_cast<int>(n);
}

bool SerialPort::Write(const uint8_t* data, size_t len)
{
    if (fd_ < 0)
        return false;

    ssize_t written = write(fd_, data, len);

    return written == static_cast<ssize_t>(len);
}
NAMESPACE_END