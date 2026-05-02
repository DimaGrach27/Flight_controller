//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <string>
#include <memory>

#include <sdf/Element.hh>

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include "CsvLogger.h"
#include "MavlinkBridge.h"
#include "GlobalDef.h"
#include "JoystickInput.h"

NAMESPACE_BEGIN
class HilPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
    ~HilPlugin() override;

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
    void LoadConfig(const std::shared_ptr<const sdf::Element>& sdf);
    void ConfigureJoint(gz::sim::EntityComponentManager& ecm, const std::string& jointName, gz::sim::Joint& joint);
    void LogSample(double simTimeSec, double angleRad, double angularVelocityRad);
    uint64_t SimTimeUsec(const gz::sim::UpdateInfo& info) const;

private:
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Joint m_rollJoint{gz::sim::kNullEntity};
    gz::sim::Joint m_pitchJoint{gz::sim::kNullEntity};

    MavlinkBridge mavlink_;
    CsvLogger logger_;
    JoystickInput m_joystickInput;

    std::string m_rollJointName = "roll_joint";
    std::string m_pitchJointName = "pitch_joint";
    std::string serialPortPath_ = "/dev/cu.usbmodem1103";
    std::string logPath_ = "one_axis_hil_log.csv";

    int baud_ = 115200;

    bool m_useJoystick = true;
    int m_joystickIndex = 0;

    double m_manualRateHz = 50.0;
    double m_lastManualSendSec = -1.0;

    double m_maxRollTorque = 0.15;
    double m_maxPitchTorque = 0.15;
    double m_rollTorqueSign = 1.0;
    double m_pitchTorqueSign = 1.0;
    double hilRateHz_ = 100.0;
    double logRateHz_ = 50.0;

    double m_disturbanceRollTorque = 0.06;
    double m_disturbancePitchTorque = -0.05;
    double disturbanceStartSec_ = 0.2;
    double disturbanceEndSec_ = 0.6;

    double lastHilSendSec_ = -1.0;
    double lastLogSec_ = -1.0;
    double m_lastRollTorque = 0.0;
    double m_lastPitchTorque = 0.0;
};
NAMESPACE_END