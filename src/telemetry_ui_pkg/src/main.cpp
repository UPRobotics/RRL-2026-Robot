#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include "telemetry_ui_pkg/main_window.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("telemetry_ui");

    telemetry_ui::MainWindow window("TELEMETRY DASHBOARD", 1280, 720);

    if (!window.initialize()) {
        spdlog::error("Failed to initialize TelemetryUI window");
        rclcpp::shutdown();
        return 1;
    }

    while (rclcpp::ok() && window.isRunning()) {
        rclcpp::spin_some(node);
        window.spinOnce();
    }

    window.shutdown();
    rclcpp::shutdown();
    return 0;
}
