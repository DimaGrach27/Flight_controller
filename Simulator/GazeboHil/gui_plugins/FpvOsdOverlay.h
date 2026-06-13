#pragma once

#include <mutex>
#include <string>

#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

namespace gz::gui::plugins
{
class FpvOsdOverlay : public gz::gui::Plugin
{
    Q_OBJECT
    Q_PROPERTY(QString osdText READ OsdText NOTIFY OsdTextChanged)

public:
    FpvOsdOverlay();
    ~FpvOsdOverlay() override = default;

    QString OsdText() const;

signals:
    void OsdTextChanged();

protected:
    void LoadConfig(const tinyxml2::XMLElement* pluginElem) override;

private slots:
    void ApplyOsdText(const QString& text);

private:
    void OnTelemetry(const gz::msgs::StringMsg& msg);

private:
    gz::transport::Node node_;
    std::string topic_ = "/fc/telemetry/osd";
    QString osdText_ = "NO TELEMETRY";
    mutable std::mutex mutex_;
};
}
