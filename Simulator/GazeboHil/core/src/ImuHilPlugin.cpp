//
// Created by Dmytro Hrachov on 03.05.2026.
//

#include "ImuHilPlugin.h"

#include <gz/plugin/Register.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/math/Quaternion.hh>

#include <algorithm>
#include <cmath>
#include <iostream>

NAMESPACE_BEGIN
ImuHilPlugin::~ImuHilPlugin()
{

}

void ImuHilPlugin::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)

{
    (void)entity;
    (void)ecm;
    (void)eventMgr;

    if (sdf->HasElement("imu_topic"))
    {
        m_imuTopic = sdf->Get<std::string>("imu_topic");
    }

    if (sdf->HasElement("motor_topic"))
    {
        m_motorTopic = sdf->Get<std::string>("motor_topic");
    }

    if (sdf->HasElement("throttle"))
    {
        m_throttle = sdf->Get<float>("throttle");
    }

    if (sdf->HasElement("serial_port"))
        m_serialPortPath = sdf->Get<std::string>("serial_port");

    if (sdf->HasElement("baud"))
        m_baud = sdf->Get<int>("baud");

    if (sdf->HasElement("use_joystick"))
        m_useJoystick = sdf->Get<bool>("use_joystick");

    if (sdf->HasElement("joystick_index"))
        m_joystickIndex = sdf->Get<int>("joystick_index");

    m_motorPublisher = m_node.Advertise<gz::msgs::Actuators>(m_motorTopic);

    if (!m_motorPublisher)
    {
        std::cerr
            << "[ImuHilPlugin] Failed to advertise motor topic: "
            << m_motorTopic
            << "\n";
    }
    else
    {
        std::cout
            << "[ImuHilPlugin] Publishing motors to: "
            << m_motorTopic
            << "\n";
    }

    const bool subscribed = m_node.Subscribe(
        m_imuTopic,
        &ImuHilPlugin::OnImu,
        this
    );

    if (!subscribed)
    {
        std::cerr
            << "[ImuHilPlugin] Failed to subscribe to IMU topic: "
            << m_imuTopic
            << "\n";
    }
    else
    {
        std::cout
            << "[ImuHilPlugin] Subscribed to IMU topic: "
            << m_imuTopic
            << "\n";
    }

    if (m_useJoystick)
    {
        m_joystickInput.Init(m_joystickIndex);
    }

    if (m_mavlinkBridge.Open(m_serialPortPath, m_baud))
    {
        printf("[ImuHilPlugin] Mavlink opened on %s", m_serialPortPath.c_str());
    }
}

void ImuHilPlugin::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
    (void)ecm;

    if (info.paused)
    {
        return;
    }

    m_mavlinkBridge.Poll();

    double dt = 0.001;

    if (m_hasLastSimTime)
    {
        const auto delta = info.simTime - m_lastSimTime;
        dt = std::chrono::duration<double>(delta).count();
        dt = std::clamp(dt, 0.0001, 0.02);
    }

    m_lastSimTime = info.simTime;
    m_hasLastSimTime = true;

    const ImuData imu = GetLatestImu();

    if (!imu.valid)
    {
        SendMotorSpeeds(0.0, 0.0, 0.0, 0.0);
        return;
    }

    UpdateAttitudeEstimator(imu, dt);

    /*
        Тут поки мінімальна перевірка:
        всі мотори однаково.

        Наступним кроком сюди підключимо твій flight controller:

            SensorData sensor;
            sensor.rollRad = attitude_.rollRad;
            sensor.pitchRad = attitude_.pitchRad;
            sensor.rollRateRadSec = imu.gyroX;
            sensor.pitchRateRadSec = imu.gyroY;
            sensor.yawRateRadSec = imu.gyroZ;

            MotorOutputs motors = controller.Update(sensor, rcInput, dt);
    */

    const MotorOutputs& motorsData = m_mavlinkBridge.Motors();

    // SendMotorSpeeds(
    //     ToMotorSpeed(m_throttle),
    //     ToMotorSpeed(m_throttle),
    //     ToMotorSpeed(m_throttle),
    //     ToMotorSpeed(m_throttle)
    // );

    SendMotorSpeeds(
        ToMotorSpeed(motorsData.m2),
        ToMotorSpeed(motorsData.m1),
        ToMotorSpeed(motorsData.m4),
        ToMotorSpeed(motorsData.m3)
    );

    // SendMotorSpeeds(
    //     ToMotorSpeed(0.96), //front right
    //     ToMotorSpeed(0.72), //back left
    //     ToMotorSpeed(0.96), //front left
    //     ToMotorSpeed(0.72)  //back right
    // );
}

void ImuHilPlugin::PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm)
{
    if (info.paused)
        return;

    const ImuData imu = GetLatestImu();
    if (!imu.valid)
    {
        printf("[ImuHilPlugin] ImuData invalid");
        return;
    }

    double simTimeSec = std::chrono::duration<double>(info.simTime).count();

    double dt = 0.001;
    if (m_hasLastSimTime)
    {
        const auto delta = info.simTime - m_lastSimTime;
        dt = std::chrono::duration<double>(delta).count();
        dt = std::clamp(dt, 0.0001, 0.02);
    }

    UpdateAttitudeEstimator(imu, dt);

    if (m_lastHilSendSec < 0.0 ||
    simTimeSec - m_lastHilSendSec >= 1.0 / m_hilRateHz)
    {
        m_lastHilSendSec = simTimeSec;

        m_mavlinkBridge.SendHilSensorFromImu(
            static_cast<uint64_t>(simTimeSec),
            imu
        );
    }

    if (m_useJoystick)
    {
        m_joystickInput.Poll();

        if (m_lastManualSendSec < 0.0 ||
            simTimeSec - m_lastManualSendSec >= 1.0 / m_manualRateHz)
        {
            m_lastManualSendSec = simTimeSec;

            const ManualControl& control = m_joystickInput.Control();

            // printf("[Joystick] arm=%d roll=%f pitch=%f yaw=%f throttle=%f\n",
            //     control.arm,
            //     control.roll,
            //     control.pitch,
            //     control.yaw,
            //     control.throttle);
            // std::cout
            // << "[Joystick] "
            // <<  "arm=" << control.arm
            // << "roll=" << control.roll
            // << " pitch=" << control.pitch
            // << " throttle=" << control.throttle
            // << " yaw=" << control.yaw
            // << std::endl;

            if (control.valid)
            {
                m_mavlinkBridge.SendManualControl(
                    control.arm,
                    control.acroMode,
                    control.roll,
                    control.pitch,
                    control.throttle,
                    control.yaw
                );
            }
        }
    }
}

void ImuHilPlugin::OnImu(const gz::msgs::IMU &msg)
{
    ImuData data;

    data.gyroX = msg.angular_velocity().x();
    data.gyroY = msg.angular_velocity().y();
    data.gyroZ = msg.angular_velocity().z();

    data.accelX = msg.linear_acceleration().x();
    data.accelY = msg.linear_acceleration().y();
    data.accelZ = msg.linear_acceleration().z();

    data.valid = true;

    {
        std::lock_guard<std::mutex> lock(m_imuMutex);
        m_latestImu = data;
    }
}

ImuData ImuHilPlugin::GetLatestImu() const
{
    std::lock_guard<std::mutex> lock(m_imuMutex);
    return m_latestImu;
}

void ImuHilPlugin::UpdateAttitudeEstimator(const ImuData &imu, double dt)
{
    const double accelRoll = std::atan2(
       imu.accelY,
       imu.accelZ
   );

    const double accelPitch = std::atan2(
        -imu.accelX,
        std::sqrt(imu.accelY * imu.accelY + imu.accelZ * imu.accelZ)
    );

    if (!m_attitude.initialized)
    {
        m_attitude.rollRad = accelRoll;
        m_attitude.pitchRad = accelPitch;
        m_attitude.initialized = true;
        return;
    }

    m_attitude.rollRad += imu.gyroX * dt;
    m_attitude.pitchRad += imu.gyroY * dt;

    constexpr double alpha = 0.98;

    m_attitude.rollRad =
        alpha * m_attitude.rollRad +
        (1.0 - alpha) * accelRoll;

    m_attitude.pitchRad =
        alpha * m_attitude.pitchRad +
        (1.0 - alpha) * accelPitch;
}

void ImuHilPlugin::SendMotorSpeeds(double m0, double m1, double m2, double m3)
{
    gz::msgs::Actuators msg;

    msg.add_velocity(m0);
    msg.add_velocity(m1);
    msg.add_velocity(m2);
    msg.add_velocity(m3);

    printf("[ImuHilPlugin] SendMotorSpeeds: %f %f %f %f\n", m0, m1, m2, m3);
    m_motorPublisher.Publish(msg);
}

double ImuHilPlugin::ToMotorSpeed(double normalized)
{
    normalized = std::clamp(normalized, 0.0, 1.0);

    constexpr double minSpeed = 0.0;
    constexpr double maxSpeed = 1200.0;

    return minSpeed + normalized * (maxSpeed - minSpeed);
}

NAMESPACE_END

GZ_ADD_PLUGIN(
    hil_plugin::ImuHilPlugin,
    gz::sim::System,
    hil_plugin::ImuHilPlugin::ISystemConfigure,
    hil_plugin::ImuHilPlugin::ISystemPreUpdate,
    hil_plugin::ImuHilPlugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    hil_plugin::ImuHilPlugin,
    "hil_plugin::ImuHilPlugin"
)