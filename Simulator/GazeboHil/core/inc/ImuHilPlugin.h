//
// Created by Dmytro Hrachov on 03.05.2026.
//
#pragma once
#include <gz/sim/System.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>

#include "GlobalDef.h"
#include "JoystickInput.h"
#include "MavlinkBridge.h"
#include "Structs.h"

NAMESPACE_BEGIN
struct AttitudeEstimate
{
    double rollRad = 0.0;
    double pitchRad = 0.0;
    bool initialized = false;
};

class ImuHilPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{

public:
    ~ImuHilPlugin() override;

    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& eventMgr
    ) override;

    void PreUpdate(
        const gz::sim::UpdateInfo& info,
        gz::sim::EntityComponentManager& ecm
    ) override;

    void PostUpdate(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm
    ) override;

private:
    void OnImu(const gz::msgs::IMU& msg);

    ImuData GetLatestImu() const;

    void UpdateAttitudeEstimator(
        const ImuData& imu,
        double dt
    );

    void SendMotorSpeeds(
        double m0,
        double m1,
        double m2,
        double m3
    );

    static double ToMotorSpeed(double normalized);

private:
    gz::transport::Node m_node;
    gz::transport::Node::Publisher m_motorPublisher;

    std::string m_imuTopic = "/world/quadcopter/model/X3/link/base_link/sensor/imu_sensor/imu";
    std::string m_motorTopic = "/X3/gazebo/command/motor_speed";

    mutable std::mutex m_imuMutex;
    ImuData m_latestImu;

    AttitudeEstimate m_attitude;

    std::chrono::steady_clock::duration m_lastSimTime{0};

    bool m_hasLastSimTime = false;

    float m_throttle = 0.70f;

    std::string m_serialPortPath = "/dev/cu.usbmodem1103";

    MavlinkBridge m_mavlinkBridge;
    JoystickInput m_joystickInput;

    int m_baud = 115200;

    bool m_useJoystick = true;
    int m_joystickIndex = 0;

    double m_hilRateHz = 100.0;
    double m_manualRateHz = 50.0;

    double m_lastHilSendSec = -1.0;
    double m_lastManualSendSec = -1.0;
};
NAMESPACE_END