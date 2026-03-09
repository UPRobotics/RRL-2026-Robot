#include <rclcpp/rclcpp.hpp>
#include "mic_pkg/mic_transmitter.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mic_pkg::MicTransmitter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
