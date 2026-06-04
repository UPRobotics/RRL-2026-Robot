#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <fstream>
#include <vector>
#include <memory>

#include "osmo_camera_pkg/usb_camera.hpp"
#include "osmo_camera_pkg/types.hpp"

extern "C" {
#include <libavutil/log.h>
}

static const rclcpp::QoS CAM_QOS =
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    av_log_set_level(AV_LOG_QUIET);
    spdlog::set_level(spdlog::level::info);

    auto node = rclcpp::Node::make_shared("osmo_stream_node");

    std::string defaultCfgDir;
    try {
        defaultCfgDir = ament_index_cpp::get_package_share_directory("osmo_camera_pkg") + "/config";
    } catch (...) { defaultCfgDir = "."; }

    node->declare_parameter<std::string>("config_dir", defaultCfgDir);
    const std::string cfgDir = node->get_parameter("config_dir").as_string();

    std::ifstream f(cfgDir + "/cameras.json");
    if (!f.is_open()) {
        spdlog::error("osmo_stream_node: cannot open {}/cameras.json", cfgDir);
        rclcpp::shutdown();
        return 1;
    }
    const auto j = nlohmann::json::parse(f);

    std::vector<osmo::CameraConfig> configs;
    for (auto& cam : j["cameras"]) {
        if (!cam.value("enabled", true)) continue;
        osmo::CameraConfig c;
        c.id     = cam.value("id",     "");
        c.name   = cam.value("name",   "");
        c.device = cam.value("device", "");
        c.topic  = cam.value("topic",  "");
        c.width  = cam.value("width",  1920);
        c.height = cam.value("height", 1080);
        c.fps    = cam.value("fps",    60);
        configs.push_back(c);
    }

    using CompressedImage = sensor_msgs::msg::CompressedImage;
    std::vector<rclcpp::Publisher<CompressedImage>::SharedPtr> pubs;
    std::vector<std::unique_ptr<osmo::UsbCamera>> cameras;

    for (auto& cfg : configs) {
        auto pub = node->create_publisher<CompressedImage>(cfg.topic, CAM_QOS);
        pubs.push_back(pub);

        auto cam = std::make_unique<osmo::UsbCamera>(cfg);

        cam->setFrameCallback([pub, cfg](const uint8_t* data, int size) {
            auto msg = std::make_unique<CompressedImage>();
            msg->header.stamp    = rclcpp::Clock{}.now();
            msg->header.frame_id = cfg.id;
            msg->format          = "jpeg";
            msg->data.assign(data, data + size);
            pub->publish(std::move(msg));
        });

        cameras.push_back(std::move(cam));
    }

    for (auto& cam : cameras) cam->start();

    spdlog::info("osmo_stream_node: streaming {} camera(s)", cameras.size());
    rclcpp::spin(node);

    for (auto& cam : cameras) cam->stop();
    rclcpp::shutdown();
    return 0;
}
