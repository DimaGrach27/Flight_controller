//
// Created by Dmytro Hrachov on 06.05.2026.
//

#include "FpvCameraSubscriber.h"
NAMESPACE_BEGIN
bool FpvCameraSubscriber::Start(const std::string &topic)
{
    return m_node.Subscribe(topic, &FpvCameraSubscriber::OnImage, this);
}

void FpvCameraSubscriber::OnImage(const gz::msgs::Image &image)
{
    std::cout
        << "Image: "
        << image.width()
        << "x"
        << image.height()
        << " pixel_format="
        << image.pixel_format_type()
        << " data_size="
        << image.data().size()
        << std::endl;
}

NAMESPACE_END
