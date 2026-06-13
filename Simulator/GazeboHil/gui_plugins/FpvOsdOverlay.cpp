#include "FpvOsdOverlay.h"

#include <iostream>

#include <gz/plugin/Register.hh>
#include <QMetaObject>

namespace gz::gui::plugins
{
FpvOsdOverlay::FpvOsdOverlay()
{
    title = "FPV OSD";
}

QString FpvOsdOverlay::OsdText() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return osdText_;
}

void FpvOsdOverlay::LoadConfig(const tinyxml2::XMLElement* pluginElem)
{
    if (pluginElem != nullptr)
    {
        if (const auto* topicElem = pluginElem->FirstChildElement("topic");
            topicElem != nullptr && topicElem->GetText() != nullptr)
        {
            topic_ = topicElem->GetText();
        }
    }

    if (!node_.Subscribe(topic_, &FpvOsdOverlay::OnTelemetry, this))
    {
        std::cerr
            << "[FpvOsdOverlay] Failed to subscribe telemetry topic: "
            << topic_
            << std::endl;
    }
    else
    {
        std::cout
            << "[FpvOsdOverlay] Subscribed telemetry topic: "
            << topic_
            << std::endl;
    }
}

void FpvOsdOverlay::ApplyOsdText(const QString& text)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        osdText_ = text;
    }

    emit OsdTextChanged();
}

void FpvOsdOverlay::OnTelemetry(const gz::msgs::StringMsg& msg)
{
    const QString text = QString::fromStdString(msg.data());
    QMetaObject::invokeMethod(
        this,
        "ApplyOsdText",
        Qt::QueuedConnection,
        Q_ARG(QString, text)
    );
}
}

GZ_ADD_PLUGIN(
    gz::gui::plugins::FpvOsdOverlay,
    gz::gui::Plugin
)

GZ_ADD_PLUGIN_ALIAS(
    gz::gui::plugins::FpvOsdOverlay,
    "FpvOsdOverlay"
)
