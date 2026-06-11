//
// Created by Dmytro Hrachov on 07.06.2026.
//

#include "SerialPort_USB.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <iostream>

#ifdef __APPLE__
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>
#endif

NAMESPACE_BEGIN

static speed_t ToPosixBaudrate(int baudrate)
{
    switch (baudrate)
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

SerialPort_USB::~SerialPort_USB()
{
    CloseInternal();
}

bool SerialPort_USB::Open(const std::string &path, int baud)
{
    Close();

    m_fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (m_fd < 0)
    {
        std::cerr << "Failed to open serial port: " << path
                  << " error: " << std::strerror(errno) << std::endl;
        return false;
    }

    termios tty {};
    if (tcgetattr(m_fd, &tty) != 0)
    {
        std::cerr << "tcgetattr failed: " << std::strerror(errno) << std::endl;
        Close();
        return false;
    }

    cfmakeraw(&tty);

    const speed_t speed = ToPosixBaudrate(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

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

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "tcsetattr failed: " << std::strerror(errno) << std::endl;
        Close();
        return false;
    }

    tcflush(m_fd, TCIOFLUSH);

    return true;
}

void SerialPort_USB::Close()
{
    CloseInternal();
}

bool SerialPort_USB::IsOpen() const
{
    return m_fd >= 0;
}

int SerialPort_USB::Read(uint8_t *buffer, size_t maxLen)
{
    if (m_fd < 0 || buffer == nullptr || maxLen == 0)
    {
        return -1;
    }

    const ssize_t result = read(m_fd, buffer, maxLen);

    if (result < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0;
        }

        return -1;
    }

    return static_cast<int>(result);
}

bool SerialPort_USB::Write(const uint8_t *data, size_t len)
{
    if (m_fd < 0 || data == nullptr || len == 0)
    {
        return false;
    }

    const ssize_t written = write(m_fd, data, len);

    return written == static_cast<ssize_t>(len);
}

void SerialPort_USB::CloseInternal()
{
    if (m_fd >= 0)
    {
        close(m_fd);
        m_fd = -1;
    }
}

NAMESPACE_END
