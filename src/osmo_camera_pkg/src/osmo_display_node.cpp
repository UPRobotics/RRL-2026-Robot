#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <fstream>
#include <vector>
#include <memory>

#include "osmo_camera_pkg/display_window.hpp"
#include "osmo_camera_pkg/types.hpp"

static const rclcpp::QoS CAM_QOS =
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    spdlog::set_level(spdlog::level::info);

    auto node = rclcpp::Node::make_shared("osmo_display_node");

    std::string defaultCfgDir;
    try {
        defaultCfgDir = ament_index_cpp::get_package_share_directory("osmo_camera_pkg") + "/config";
    } catch (...) { defaultCfgDir = "."; }

    node->declare_parameter<std::string>("config_dir", defaultCfgDir);
    const std::string cfgDir = node->get_parameter("config_dir").as_string();

    std::ifstream f(cfgDir + "/cameras.json");
    if (!f.is_open()) {
        spdlog::error("osmo_display_node: cannot open {}/cameras.json", cfgDir);
        rclcpp::shutdown();
        return 1;
    }
    const auto j = nlohmann::json::parse(f);

    osmo::DisplayConfig dispCfg;
    if (j.contains("display")) {
        dispCfg.window_width  = j["display"].value("window_width",  1920);
        dispCfg.window_height = j["display"].value("window_height",  540);
    }

    std::vector<std::string> topics;
    for (auto& cam : j["cameras"]) {
        if (cam.value("enabled", true))
            topics.push_back(cam.value("topic", ""));
    }

    osmo::DisplayWindow window(dispCfg, static_cast<int>(topics.size()));
    if (!window.initialize()) {
        spdlog::error("osmo_display_node: DisplayWindow init failed");
        rclcpp::shutdown();
        return 1;
    }

    using CompressedImage = sensor_msgs::msg::CompressedImage;
    std::vector<rclcpp::Subscription<CompressedImage>::SharedPtr> subs;
    subs.reserve(topics.size());

    for (int i = 0; i < static_cast<int>(topics.size()); ++i) {
        auto sub = node->create_subscription<CompressedImage>(
            topics[i], CAM_QOS,
            [&window, i](const CompressedImage::SharedPtr msg) {
                window.submitJpegFrame(i, msg->data.data(),
                                       static_cast<int>(msg->data.size()));
            });
        subs.push_back(sub);
    }

    spdlog::info("osmo_display_node: subscribing to {} topic(s)", topics.size());

    while (window.spinOnce()) {
        rclcpp::spin_some(node);
    }

    window.shutdown();
    rclcpp::shutdown();
    return 0;
}
