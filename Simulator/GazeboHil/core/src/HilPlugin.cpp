//
// Created by Dmytro Hrachov on 30.04.2026.
//
#include "HilPlugin.h"

#include "MathUtils.h"

#include <chrono>
#include <iostream>

#include <gz/plugin/Register.hh>

NAMESPACE_BEGIN
HilPlugin::~HilPlugin()
{
    if (logger_.IsOpen())
    {
        logger_.Close();
    }
}

void HilPlugin::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&)
{
    model_ = gz::sim::Model(entity);

    if (!model_.Valid(ecm))
    {
        std::cerr << "[HilPlugin] Plugin must be attached to a model" << std::endl;
        return;
    }

    LoadConfig(sdf);
    ConfigureJoint(ecm, m_rollJointName, m_rollJoint);
    ConfigureJoint(ecm, m_pitchJointName, m_pitchJoint);

    if (mavlink_.Open(serialPortPath_, baud_))
    {
        std::cout << "[HilPlugin] Mavlink opened on " << serialPortPath_ << std::endl;
    }

    if (logger_.Open(logPath_))
    {
        std::cout << "[HilPlugin] Logging to: " << logPath_ << std::endl;
    }

    std::cout
        << "[HilPlugin] Configured"
        << " jointRoll=" << m_rollJointName
        << " jointPitch=" << m_pitchJointName
        << " serial=" << serialPortPath_
        << " baud=" << baud_
        << " maxTorqueRoll=" << m_maxRollTorque
        << " maxTorquePitch=" << m_maxPitchTorque
        << " torqueSignRoll=" << m_rollTorqueSign
        << " torqueSignPitch=" << m_pitchTorqueSign
        << std::endl;
}

void HilPlugin::LoadConfig(const std::shared_ptr<const sdf::Element>& sdf)
{
    if (sdf->HasElement("roll_joint_name"))
        m_rollJointName = sdf->Get<std::string>("roll_joint_name");

    if (sdf->HasElement("pitch_joint_name"))
        m_pitchJointName = sdf->Get<std::string>("pitch_joint_name");

    if (sdf->HasElement("serial_port"))
        serialPortPath_ = sdf->Get<std::string>("serial_port");

    if (sdf->HasElement("baud"))
        baud_ = sdf->Get<int>("baud");

    if (sdf->HasElement("max_roll_torque"))
        m_maxRollTorque = sdf->Get<double>("max_roll_torque");

    if (sdf->HasElement("max_pitch_torque"))
        m_maxPitchTorque = sdf->Get<double>("max_pitch_torque");

    if (sdf->HasElement("roll_torque_sign"))
        m_rollTorqueSign = sdf->Get<double>("roll_torque_sign");

    if (sdf->HasElement("pitch_torque_sign"))
        m_pitchTorqueSign = sdf->Get<double>("pitch_torque_sign");

    if (sdf->HasElement("hil_rate_hz"))
        hilRateHz_ = sdf->Get<double>("hil_rate_hz");

    if (sdf->HasElement("log_rate_hz"))
        logRateHz_ = sdf->Get<double>("log_rate_hz");

    if (sdf->HasElement("log_path"))
        logPath_ = sdf->Get<std::string>("log_path");

    if (sdf->HasElement("disturbance_roll_torque"))
        m_disturbanceRollTorque = sdf->Get<double>("disturbance_roll_torque");

    if (sdf->HasElement("disturbance_pitch_torque"))
        m_disturbancePitchTorque = sdf->Get<double>("disturbance_pitch_torque");

    if (sdf->HasElement("disturbance_start"))
        disturbanceStartSec_ = sdf->Get<double>("disturbance_start");

    if (sdf->HasElement("disturbance_end"))
        disturbanceEndSec_ = sdf->Get<double>("disturbance_end");
}

void HilPlugin::ConfigureJoint(gz::sim::EntityComponentManager& ecm, const std::string& jointName, gz::sim::Joint& joint)
{
    gz::sim::Entity jointEntity = model_.JointByName(ecm, jointName);

    if (jointEntity == gz::sim::kNullEntity)
    {
        std::cerr << "[HilPlugin] Could not find joint: " << jointName << std::endl;
        return;
    }

    joint = gz::sim::Joint(jointEntity);

    joint.EnablePositionCheck(ecm, true);
    joint.EnableVelocityCheck(ecm, true);
}

void HilPlugin::PreUpdate(
    const gz::sim::UpdateInfo& info,
    gz::sim::EntityComponentManager& ecm)
{
    if (info.paused)
        return;

    mavlink_.Poll();

    double simTimeSec = std::chrono::duration<double>(info.simTime).count();
    const MotorOutputs& m = mavlink_.Motors();

    /*
    Спрощений quad mixer mapping.

    M1 front-left
    M2 front-right
    M3 rear-right
    M4 rear-left

    Якщо напрямки неправильні — міняємо signs у SDF.
    */

    double rollCmd = (-(m.m2 + m.m3) + (m.m1 + m.m4)) * 0.5;

    double pitchCmd = ((m.m1 + m.m2) - (m.m3 + m.m4)) * 0.5;

    double rollTorque = m_rollTorqueSign * rollCmd * m_maxRollTorque;

    double pitchTorque = m_pitchTorqueSign * pitchCmd * m_maxPitchTorque;

    if (simTimeSec > disturbanceStartSec_ &&
        simTimeSec < disturbanceEndSec_)
    {
        rollTorque += m_disturbanceRollTorque;
        pitchTorque += m_disturbancePitchTorque;
    }

    rollTorque = Clamp(rollTorque, -m_maxRollTorque, m_maxRollTorque);
    pitchTorque = Clamp(pitchTorque, -m_maxPitchTorque, m_maxPitchTorque);

    m_lastRollTorque = rollTorque;
    m_lastPitchTorque = pitchTorque;

    if (m_rollJoint.Valid(ecm))
        m_rollJoint.SetForce(ecm, {rollTorque});

    if (m_pitchJoint.Valid(ecm))
        m_pitchJoint.SetForce(ecm, {pitchTorque});

    // static double lastPrintSec = 0.0;
    // if (simTimeSec - lastPrintSec > 0.2)
    // {
    //     lastPrintSec = simTimeSec;
    //
    //     std::cout
    //         << "[Hil] "
    //         << "m1=" << m.m1
    //         << " m2=" << m.m2
    //         << " m3=" << m.m3
    //         << " m4=" << m.m4
    //         << " rollTorque=" << rollTorque
    //         << " pitchTorque=" << pitchTorque
    //         << std::endl;
    // }
}

void HilPlugin::PostUpdate(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm)
{
    if (info.paused)
        return;

    if (!m_rollJoint.Valid(ecm) || !m_pitchJoint.Valid(ecm))
        return;

    auto rollPos = m_rollJoint.Position(ecm);
    auto rollVel = m_rollJoint.Velocity(ecm);

    auto pitchPos = m_pitchJoint.Position(ecm);
    auto pitchVel = m_pitchJoint.Velocity(ecm);

    if (!rollPos || !rollVel || rollPos->empty() || rollVel->empty())
        return;

    if (!pitchPos || !pitchVel || pitchPos->empty() || pitchVel->empty())
        return;

    double rollRad = (*rollPos)[0];
    double rollRateRad = (*rollVel)[0];

    double pitchRad = (*pitchPos)[0];
    double pitchRateRad = (*pitchVel)[0];

    double simTimeSec = std::chrono::duration<double>(info.simTime).count();

    // std::cout
    // << "[State] "
    // << "simTimeSec=" << simTimeSec
    // << " lastHilSendSec_=" << lastHilSendSec_
    // << " hilRateHz_=" << hilRateHz_
    // << " 1.0 / hilRateHz_=" << 1.0 / hilRateHz_
    // << std::endl;

    if (lastHilSendSec_ < 0.0 ||
        simTimeSec - lastHilSendSec_ >= 1.0 / hilRateHz_)
    {
        lastHilSendSec_ = simTimeSec;

        mavlink_.SendHilSensor(
            SimTimeUsec(info),
            rollRad,
            pitchRad,
            rollRateRad,
            pitchRateRad
        );
    }

    // LogSample(simTimeSec, angleRadY, angularVelocityRadY);

    // std::cout
    // << "[State] "
    // << "roll=" << rollRad * 57.2957795
    // << " pitch=" << pitchRad * 57.2957795
    // << " rollRate=" << rollRateRad * 57.2957795
    // << " pitchRate=" << pitchRateRad * 57.2957795
    // << std::endl;
}

void HilPlugin::LogSample(
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
        motors.m1,
        motors.m2,
        m_lastRollTorque
    );
}

uint64_t HilPlugin::SimTimeUsec(const gz::sim::UpdateInfo& info) const
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            info.simTime
        ).count()
    );
}

NAMESPACE_END

GZ_ADD_PLUGIN(
    hil_plugin::HilPlugin,
    gz::sim::System,
    hil_plugin::HilPlugin::ISystemConfigure,
    hil_plugin::HilPlugin::ISystemPreUpdate,
    hil_plugin::HilPlugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    hil_plugin::HilPlugin,
    "one_axis_hil::OneAxisHilPlugin"
)