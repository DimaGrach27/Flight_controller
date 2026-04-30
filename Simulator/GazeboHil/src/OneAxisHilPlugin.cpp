//
// Created by Dmytro Hrachov on 30.04.2026.
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include "mavlink/common/mavlink.h"

namespace one_axis_hil
{

static double Clamp(double value, double minValue, double maxValue)
{
    return std::max(minValue, std::min(maxValue, value));
}

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

class SerialPort
{
public:
    ~SerialPort()
    {
        Close();
    }

    bool Open(const std::string& path, int baud)
    {
        Close();

        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd_ < 0)
        {
            std::cerr << "[OneAxisHil] Failed to open serial port: "
                      << path << std::endl;
            return false;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0)
        {
            std::cerr << "[OneAxisHil] tcgetattr failed" << std::endl;
            Close();
            return false;
        }

        cfmakeraw(&tty);

        speed_t speed = BaudToTermios(baud);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
        tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
        tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
        tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
        tty.c_cflag |= CS8;

#ifdef CRTSCTS
        tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            std::cerr << "[OneAxisHil] tcsetattr failed" << std::endl;
            Close();
            return false;
        }

        tcflush(fd_, TCIOFLUSH);

        std::cout << "[OneAxisHil] Serial opened: "
                  << path << " @ " << baud << std::endl;

        return true;
    }

    void Close()
    {
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }

    bool IsOpen() const
    {
        return fd_ >= 0;
    }

    int Read(uint8_t* buffer, size_t maxLen)
    {
        if (fd_ < 0)
            return -1;

        ssize_t n = read(fd_, buffer, maxLen);

        if (n <= 0)
            return 0;

        return static_cast<int>(n);
    }

    bool Write(const uint8_t* data, size_t len)
    {
        if (fd_ < 0)
            return false;

        ssize_t written = write(fd_, data, len);
        return written == static_cast<ssize_t>(len);
    }

private:
    int fd_ = -1;
};

class OneAxisHilPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager&) override
    {
        model_ = gz::sim::Model(entity);

        if (!model_.Valid(ecm))
        {
            std::cerr << "[OneAxisHil] Plugin must be attached to a model"
                      << std::endl;
            return;
        }

        if (sdf->HasElement("joint_name"))
            jointName_ = sdf->Get<std::string>("joint_name");

        if (sdf->HasElement("serial_port"))
            serialPortPath_ = sdf->Get<std::string>("serial_port");

        if (sdf->HasElement("baud"))
            baud_ = sdf->Get<int>("baud");

        if (sdf->HasElement("max_torque"))
            maxTorque_ = sdf->Get<double>("max_torque");

        if (sdf->HasElement("torque_sign"))
            torqueSign_ = sdf->Get<double>("torque_sign");

        if (sdf->HasElement("hil_rate_hz"))
            hilRateHz_ = sdf->Get<double>("hil_rate_hz");

        if (sdf->HasElement("disturbance_torque"))
            disturbanceTorque_ = sdf->Get<double>("disturbance_torque");

        if (sdf->HasElement("disturbance_start"))
            disturbanceStartSec_ = sdf->Get<double>("disturbance_start");

        if (sdf->HasElement("disturbance_end"))
            disturbanceEndSec_ = sdf->Get<double>("disturbance_end");

        gz::sim::Entity jointEntity = model_.JointByName(ecm, jointName_);

        if (jointEntity == gz::sim::kNullEntity)
        {
            std::cerr << "[OneAxisHil] Could not find joint: "
                      << jointName_ << std::endl;
            return;
        }

        joint_ = gz::sim::Joint(jointEntity);

        joint_.EnablePositionCheck(ecm, true);
        joint_.EnableVelocityCheck(ecm, true);

        serial_.Open(serialPortPath_, baud_);

        std::cout << "[OneAxisHil] Configured"
                  << " joint=" << jointName_
                  << " maxTorque=" << maxTorque_
                  << " torqueSign=" << torqueSign_
                  << " hilRateHz=" << hilRateHz_
                  << " disturbanceTorque=" << disturbanceTorque_
                  << " disturbanceStartSec=" << disturbanceStartSec_
                  << " disturbanceEndSec=" << disturbanceEndSec_
                  << std::endl;
    }

    void PreUpdate(
        const gz::sim::UpdateInfo& info,
        gz::sim::EntityComponentManager& ecm) override
    {
        if (info.paused)
            return;

        ReadMavlink();

        double simTimeSec = std::chrono::duration<double>(info.simTime).count();

        double torque = torqueSign_ * (rightMotor_ - leftMotor_) * maxTorque_;

        // Startup disturbance: push the beam for a short time
        if (simTimeSec > disturbanceStartSec_ &&
            simTimeSec < disturbanceEndSec_)
        {
            torque += disturbanceTorque_;
        }

        torque = Clamp(torque, -maxTorque_, maxTorque_);

        if (joint_.Valid(ecm))
        {
            joint_.SetForce(ecm, {torque});
        }
    }

    void PostUpdate(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm) override
    {
        if (info.paused)
            return;

        if (!serial_.IsOpen())
            return;

        if (!joint_.Valid(ecm))
            return;

        auto pos = joint_.Position(ecm);
        auto vel = joint_.Velocity(ecm);

        if (!pos || !vel || pos->empty() || vel->empty())
            return;

        double angleRad = (*pos)[0];
        double angularVelocityRad = (*vel)[0];

        double simTimeSec =
            std::chrono::duration<double>(info.simTime).count();

        if (simTimeSec - lastHilSendSec_ < 1.0 / hilRateHz_)
            return;

        lastHilSendSec_ = simTimeSec;

        SendHilSensor(info, angleRad, angularVelocityRad);

        double deltaTimeSec = simTimeSec - lastDebugTick;

        constexpr double DEBUG_TIME_STEP = 0.5;
        if (deltaTimeSec >=  DEBUG_TIME_STEP)
        {
            std::cout
                << "[OneAxisHil] angle=" << angleRad * 57.2957795
                << " vel=" << angularVelocityRad * 57.2957795
                << " L=" << leftMotor_
                << " R=" << rightMotor_
                << std::endl;
        }
    }

private:
    void ReadMavlink()
    {
        uint8_t buffer[512];

        while (true)
        {
            int n = serial_.Read(buffer, sizeof(buffer));

            if (n <= 0)
                break;

            for (int i = 0; i < n; ++i)
            {
                mavlink_message_t msg;
                mavlink_status_t status;

                if (mavlink_parse_char(
                        MAVLINK_COMM_0,
                        buffer[i],
                        &msg,
                        &status))
                {
                    HandleMavlinkMessage(msg);
                }
            }
        }
    }

    void HandleMavlinkMessage(const mavlink_message_t& msg)
    {
        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            {
                mavlink_servo_output_raw_t servo{};
                mavlink_msg_servo_output_raw_decode(&msg, &servo);

                leftMotor_ = PwmToMotor(servo.servo1_raw);
                rightMotor_ = PwmToMotor(servo.servo2_raw);

                break;
            }

            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                // We do not need to handle it now.
                break;
            }

            default:
                break;
        }
    }

    double PwmToMotor(uint16_t pwm) const
    {
        double value = (static_cast<double>(pwm) - 1000.0) / 1000.0;
        return Clamp(value, 0.0, 1.0);
    }

    void SendHilSensor(
        const gz::sim::UpdateInfo& info,
        double angleRad,
        double angularVelocityRad)
    {
        constexpr double g = 9.80665;

        float xacc = 0.0f;
        float yacc = static_cast<float>(std::sin(angleRad) * g);
        float zacc = static_cast<float>(std::cos(angleRad) * g);

        float xgyro = static_cast<float>(angularVelocityRad);
        float ygyro = 0.0f;
        float zgyro = 0.0f;

        float xmag = 0.0f;
        float ymag = 0.0f;
        float zmag = 0.0f;

        float absPressure = 1013.25f;
        float diffPressure = 0.0f;
        float pressureAlt = 0.0f;
        float temperature = 25.0f;

        uint64_t timeUsec =
            static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    info.simTime).count());

        uint32_t fieldsUpdated = 0xFFFF;

        mavlink_message_t msg;
        uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_hil_sensor_pack(
            255,                    // bridge system id
            1,                      // component id
            &msg,
            timeUsec,
            xacc,
            yacc,
            zacc,
            xgyro,
            ygyro,
            zgyro,
            xmag,
            ymag,
            zmag,
            absPressure,
            diffPressure,
            pressureAlt,
            temperature,
            fieldsUpdated,
            0
        );

        uint16_t len = mavlink_msg_to_send_buffer(txBuffer, &msg);
        serial_.Write(txBuffer, len);
    }

private:
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Joint joint_{gz::sim::kNullEntity};

    SerialPort serial_;

    std::string jointName_ = "roll_joint";
    std::string serialPortPath_ = "/dev/cu.usbmodem103";
    int baud_ = 115200;

    double maxTorque_ = 0.15;
    double torqueSign_ = 1.0;
    double hilRateHz_ = 100.0;
    double lastHilSendSec_ = -1.0;

    double leftMotor_ = 0.5;
    double rightMotor_ = 0.5;

    double disturbanceTorque_ = 0.08;
    double disturbanceStartSec_ = 0.2;
    double disturbanceEndSec_ = 0.6;

    //debug zone
    double lastDebugTick = 0.0;
};

} // namespace one_axis_hil

GZ_ADD_PLUGIN(
    one_axis_hil::OneAxisHilPlugin,
    gz::sim::System,
    one_axis_hil::OneAxisHilPlugin::ISystemConfigure,
    one_axis_hil::OneAxisHilPlugin::ISystemPreUpdate,
    one_axis_hil::OneAxisHilPlugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    one_axis_hil::OneAxisHilPlugin,
    "one_axis_hil::OneAxisHilPlugin"
)