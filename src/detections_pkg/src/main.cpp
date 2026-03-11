#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <cstring>

#include "detections_pkg/detection_window.h"

extern "C" {
#include <libavutil/log.h>
}

static bool g_debug = false;

static void ffmpegLogCallback(void*, int level, const char* fmt, va_list vl) {
    if (!g_debug) return;
    if (level > AV_LOG_VERBOSE) return;
    char buf[2048];
    vsnprintf(buf, sizeof(buf), fmt, vl);
    size_t len = strlen(buf);
    while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) buf[--len] = '\0';
    if (len == 0) return;
    fprintf(stderr, "[FFmpeg] %s\n", buf);
    fflush(stderr);
}

static detections::DetectionConfig loadConfig(const std::string& path) {
    detections::DetectionConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) {
        spdlog::warn("Config not found at {}, using defaults", path);
        cfg.rtsp_url = "rtsp://admin:admin@192.168.0.206:554/profile0";
        cfg.rtsp_url_lowres = "rtsp://admin:admin@192.168.0.206:554/profile1";
        return cfg;
    }
    try {
        auto j = nlohmann::json::parse(f);
        auto& cam = j["camera"];
        cfg.rtsp_url        = cam.value("rtsp_url",        "rtsp://admin:admin@192.168.0.206:554/profile0");
        cfg.rtsp_url_lowres = cam.value("rtsp_url_lowres", "rtsp://admin:admin@192.168.0.206:554/profile1");

        if (j.contains("detection")) {
            auto& det = j["detection"];
            cfg.qr_detect_interval       = det.value("qr_detect_interval", 5);
            cfg.motion_min_area          = det.value("motion_min_area", 500);
            cfg.motion_skip_frames       = det.value("motion_skip_frames", 5);
            cfg.motion_threshold         = det.value("motion_threshold", 30.0);
            cfg.motion_accumulate_weight = det.value("motion_accumulate_weight", 0.05);
        }
        if (j.contains("hazmat")) {
            auto& hz = j["hazmat"];
            cfg.enable_hazmat          = hz.value("enabled", true);
            cfg.hazmat_detect_interval = hz.value("detect_interval", 2);
            cfg.hazmat_conf_threshold  = hz.value("confidence_threshold", 0.5f);
            cfg.hazmat_nms_threshold   = hz.value("nms_threshold", 0.45f);
            cfg.hazmat_use_cuda        = hz.value("use_cuda", true);
            cfg.hazmat_yolo_model      = hz.value("yolo_model", "");
            cfg.hazmat_resnet_model    = hz.value("resnet_model", "");
        }
        if (j.contains("display")) {
            auto& disp = j["display"];
            cfg.window_width    = disp.value("window_width", 1280);
            cfg.window_height   = disp.value("window_height", 720);
            cfg.mag_panel_width = disp.value("magnetometer_panel_width", 300);
        }
        if (j.contains("camera")) {
            cfg.rotation = j["camera"].value("rotation", 0);
        }
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
    }
    return cfg;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--debug") == 0 || strcmp(argv[i], "-d") == 0) {
            g_debug = true;
        }
    }

    if (g_debug) {
        spdlog::set_level(spdlog::level::trace);
        av_log_set_level(AV_LOG_VERBOSE);
        av_log_set_callback(ffmpegLogCallback);
    } else {
        spdlog::set_level(spdlog::level::info);
        av_log_set_level(AV_LOG_QUIET);
    }

    spdlog::info("=== Detection Monitor (ROS2) ===");

    auto node = rclcpp::Node::make_shared("detections_pkg");

    // Resolve config directory
    std::string defaultConfigDir;
    try {
        defaultConfigDir = ament_index_cpp::get_package_share_directory("detections_pkg") + "/config";
    } catch (const std::exception&) {
        defaultConfigDir = ".";
    }
    node->declare_parameter<std::string>("config_dir", defaultConfigDir);
    std::string configDir = node->get_parameter("config_dir").as_string();
    spdlog::info("Config directory: {}", configDir);

    std::string configPath = configDir + "/detections_config.json";

    // Resolve symlinks so we log/save to the real source path
    try {
        configPath = std::filesystem::canonical(configPath).string();
    } catch (...) {}

    auto config = loadConfig(configPath);
    config.config_path = configPath;

    // Resolve hazmat model paths relative to config directory
    if (!config.hazmat_yolo_model.empty() &&
        config.hazmat_yolo_model[0] != '/') {
        config.hazmat_yolo_model = configDir + "/" + config.hazmat_yolo_model;
    }
    if (!config.hazmat_resnet_model.empty() &&
        config.hazmat_resnet_model[0] != '/') {
        config.hazmat_resnet_model = configDir + "/" + config.hazmat_resnet_model;
    }

    spdlog::info("RTSP URL: {}", config.rtsp_url);
    spdlog::info("Window: {}x{}, panel width: {}",
                 config.window_width, config.window_height, config.mag_panel_width);

    detections::DetectionWindow window(config, node);
    if (!window.initialize()) {
        spdlog::error("Failed to initialize detection window");
        rclcpp::shutdown();
        return 1;
    }

    spdlog::info("Entering main loop");
    while (window.spinOnce()) {
        rclcpp::spin_some(node);
    }
    spdlog::info("Main loop ended");

    window.shutdown();
    rclcpp::shutdown();
    spdlog::info("Application terminated");
    return 0;
}
