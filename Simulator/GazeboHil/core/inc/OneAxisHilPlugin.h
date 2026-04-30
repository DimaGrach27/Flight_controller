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

NAMESPACE_BEGIN
class OneAxisHilPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
    ~OneAxisHilPlugin() override;

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
    void ConfigureJoint(gz::sim::EntityComponentManager& ecm);
    void LogSample(double simTimeSec, double angleRad, double angularVelocityRad);
    uint64_t SimTimeUsec(const gz::sim::UpdateInfo& info) const;

private:
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Joint joint_{gz::sim::kNullEntity};

    MavlinkBridge mavlink_;
    CsvLogger logger_;

    std::string jointName_ = "roll_joint";
    std::string serialPortPath_ = "/dev/cu.usbmodem103";
    std::string logPath_ = "one_axis_hil_log.csv";

    int baud_ = 115200;

    double maxTorque_ = 0.15;
    double torqueSign_ = 1.0;
    double hilRateHz_ = 100.0;
    double logRateHz_ = 50.0;

    double disturbanceTorque_ = 0.08;
    double disturbanceStartSec_ = 0.2;
    double disturbanceEndSec_ = 0.6;

    double lastHilSendSec_ = -1.0;
    double lastLogSec_ = -1.0;
    double lastAppliedTorque_ = 0.0;
};
NAMESPACE_END