//
// Created by Dmytro Hrachov on 06.05.2026.
//
#pragma once
#include "GlobalDef.h"

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include <atomic>
#include <cstdint>
#include <iostream>
#include <string>

NAMESPACE_BEGIN
class FpvCameraSubscriber
{

    public:
        bool Start(const std::string& topic);

    private:
        void OnImage(const gz::msgs::Image& image);

    private:
        gz::transport::Node m_node;

    const std::string topic = "/world/quadcopter/model/X3/link/base_link/sensor/fpv_camera/image";
    int main()
    {
        FpvCameraSubscriber camera;

        const std::string topic =
            "/world/quadcopter/model/X3/link/base_link/sensor/fpv_camera/image";

        if (!camera.Start(topic))
        {
            std::cerr << "Failed to subscribe to camera topic\n";
            return 1;
        }

        std::cout << "Subscribed to FPV camera\n";

        while (true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return 0;
    }
};
NAMESPACE_END