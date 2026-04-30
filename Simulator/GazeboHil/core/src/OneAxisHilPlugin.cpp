//
// Created by Dmytro Hrachov on 30.04.2026.
//
#include "OneAxisHilPlugin.h"

#include "MathUtils.h"

#include <chrono>
#include <iostream>

#include <gz/plugin/Register.hh>

NAMESPACE_BEGIN
OneAxisHilPlugin::~OneAxisHilPlugin()
{
    if (logger_.IsOpen())
    {
        logger_.Close();
    }
}

void OneAxisHilPlugin::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&)
{
    model_ = gz::sim::Model(entity);

    if (!model_.Valid(ecm))
    {
        std::cerr << "[OneAxisHil] Plugin must be attached to a model" << std::endl;
        return;
    }

    LoadConfig(sdf);
    ConfigureJoint(ecm);

    mavlink_.Open(serialPortPath_, baud_);

    if (logger_.Open(logPath_))
    {
        std::cout << "[OneAxisHil] Logging to: " << logPath_ << std::endl;
    }

    std::cout
        << "[OneAxisHil] Configured"
        << " joint=" << jointName_
        << " serial=" << serialPortPath_
        << " baud=" << baud_
        << " maxTorque=" << maxTorque_
        << " torqueSign=" << torqueSign_
        << std::endl;
}

void OneAxisHilPlugin::LoadConfig(const std::shared_ptr<const sdf::Element>& sdf)
{
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

    if (sdf->HasElement("log_rate_hz"))
        logRateHz_ = sdf->Get<double>("log_rate_hz");

    if (sdf->HasElement("log_path"))
        logPath_ = sdf->Get<std::string>("log_path");

    if (sdf->HasElement("disturbance_torque"))
        disturbanceTorque_ = sdf->Get<double>("disturbance_torque");

    if (sdf->HasElement("disturbance_start"))
        disturbanceStartSec_ = sdf->Get<double>("disturbance_start");

    if (sdf->HasElement("disturbance_end"))
        disturbanceEndSec_ = sdf->Get<double>("disturbance_end");
}

void OneAxisHilPlugin::ConfigureJoint(gz::sim::EntityComponentManager& ecm)
{
    gz::sim::Entity jointEntity = model_.JointByName(ecm, jointName_);

    if (jointEntity == gz::sim::kNullEntity)
    {
        std::cerr << "[OneAxisHil] Could not find joint: " << jointName_ << std::endl;
        return;
    }

    joint_ = gz::sim::Joint(jointEntity);

    joint_.EnablePositionCheck(ecm, true);
    joint_.EnableVelocityCheck(ecm, true);
}

void OneAxisHilPlugin::PreUpdate(
    const gz::sim::UpdateInfo& info,
    gz::sim::EntityComponentManager& ecm)
{
    if (info.paused)
        return;

    mavlink_.Poll();

    double simTimeSec = std::chrono::duration<double>(info.simTime).count();

    const MotorOutputs& motors = mavlink_.Motors();

    double torque = torqueSign_ * (motors.right - motors.left) * maxTorque_;

    if (simTimeSec > disturbanceStartSec_ &&
        simTimeSec < disturbanceEndSec_)
    {
        torque += disturbanceTorque_;
    }

    torque = Clamp(torque, -maxTorque_, maxTorque_);
    lastAppliedTorque_ = torque;

    if (joint_.Valid(ecm))
    {
        joint_.SetForce(ecm, {torque});
    }
}

void OneAxisHilPlugin::PostUpdate(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm)
{
    if (info.paused)
        return;

    if (!joint_.Valid(ecm))
        return;

    auto pos = joint_.Position(ecm);
    auto vel = joint_.Velocity(ecm);

    if (!pos || !vel || pos->empty() || vel->empty())
        return;

    double angleRad = (*pos)[0];
    double angularVelocityRad = (*vel)[0];

    double simTimeSec = std::chrono::duration<double>(info.simTime).count();

    if (lastHilSendSec_ < 0.0 ||
        simTimeSec - lastHilSendSec_ >= 1.0 / hilRateHz_)
    {
        lastHilSendSec_ = simTimeSec;

        mavlink_.SendHilSensor(
            SimTimeUsec(info),
            angleRad,
            angularVelocityRad
        );
    }

    LogSample(simTimeSec, angleRad, angularVelocityRad);
}

void OneAxisHilPlugin::LogSample(
    double simTimeSec,
    double angleRad,
    double angularVelocityRad)
{
    if (lastLogSec_ >= 0.0 &&
        simTimeSec - lastLogSec_ < 1.0 / logRateHz_)
    {
        return;
    }

    lastLogSec_ = simTimeSec;

    const MotorOutputs& motors = mavlink_.Motors();

    logger_.Log(
        simTimeSec,
        angleRad * 57.2957795,
        angularVelocityRad * 57.2957795,
        motors.left,
        motors.right,
        lastAppliedTorque_
    );
}

uint64_t OneAxisHilPlugin::SimTimeUsec(const gz::sim::UpdateInfo& info) const
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            info.simTime
        ).count()
    );
}
NAMESPACE_END

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