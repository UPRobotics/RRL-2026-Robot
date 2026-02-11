#include <iostream>
#include <algorithm>
#include <cstring>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include "camera_pkg/main_window.h"
#include "camera_pkg/camera_manager.h"
#include "camera_pkg/settings_manager.h"

extern "C" {
#include <libavutil/log.h>
}

// Global debug flag
bool g_debugMode = false;
camera_viewer::DecodeMode g_decodeMode = camera_viewer::DecodeMode::GPU;

// FFmpeg log callback for debug mode - show ALL messages
// Using fprintf directly to avoid potential spdlog threading issues
static void ffmpegLogCallback(void* ptr, int level, const char* fmt, va_list vl) {
    (void)ptr; // Unused parameter
    
    if (!g_debugMode) return;
    
    // Filter based on level - show warnings and above, plus info
    if (level > AV_LOG_VERBOSE) return;
    
    char buf[2048];
    vsnprintf(buf, sizeof(buf), fmt, vl);
    
    // Remove trailing newline
    size_t len = strlen(buf);
    while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) {
        buf[--len] = '\0';
    }
    
    // Skip empty messages
    if (len == 0) return;
    
    // Use fprintf for thread safety (spdlog can crash from FFmpeg threads)
    const char* levelStr = "INFO";
    switch (level) {
        case AV_LOG_PANIC:
        case AV_LOG_FATAL:
            levelStr = "FATAL";
            break;
        case AV_LOG_ERROR:
            levelStr = "ERROR";
            break;
        case AV_LOG_WARNING:
            levelStr = "WARN";
            break;
        case AV_LOG_INFO:
            levelStr = "INFO";
            break;
        case AV_LOG_VERBOSE:
            levelStr = "VERBOSE";
            break;
        default:
            levelStr = "DEBUG";
            break;
    }
    
    fprintf(stderr, "[FFmpeg-%s] %s\n", levelStr, buf);
    fflush(stderr);
}

int main(int argc, char* argv[]) {
    // ----------------------------------------------------------------
    // Initialise ROS2 first (consumes --ros-args and everything after)
    // ----------------------------------------------------------------
    rclcpp::init(argc, argv);

    // ----------------------------------------------------------------
    // Parse remaining (non-ROS) command-line arguments
    // ----------------------------------------------------------------
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--debug") == 0 || strcmp(argv[i], "-d") == 0) {
            g_debugMode = true;
        } else if (strcmp(argv[i], "--cpu") == 0) {
            g_decodeMode = camera_viewer::DecodeMode::CPU;
        } else if (strcmp(argv[i], "--gpu") == 0) {
            g_decodeMode = camera_viewer::DecodeMode::GPU;
        }
    }
    
    // Set log level based on debug mode
    if (g_debugMode) {
        spdlog::set_level(spdlog::level::trace);
        // Set FFmpeg to VERBOSE level to see all connection and decoding details
        av_log_set_level(AV_LOG_VERBOSE);
        av_log_set_callback(ffmpegLogCallback);
        spdlog::info("Debug mode enabled - FFmpeg VERBOSE output will be shown");
    } else {
        spdlog::set_level(spdlog::level::info);
        av_log_set_level(AV_LOG_QUIET);
    }
    
    spdlog::info("=== Camera Visualize (ROS2) ===");
    spdlog::info("Starting application...");

    // ----------------------------------------------------------------
    // Create the ROS2 node that thermal streams will subscribe through
    // ----------------------------------------------------------------
    auto ros_node = rclcpp::Node::make_shared("camera_pkg");
    spdlog::info("ROS2 node 'camera_pkg' created");

    // ----------------------------------------------------------------
    // Resolve the config directory shipped with the ROS2 package.
    // The launch file passes it as a parameter named "config_dir".
    // Fall back to the current working directory otherwise.
    // ----------------------------------------------------------------
    std::string configDir = ".";
    if (ros_node->has_parameter("config_dir")) {
        configDir = ros_node->get_parameter("config_dir").as_string();
    } else {
        ros_node->declare_parameter<std::string>("config_dir", ".");
        configDir = ros_node->get_parameter("config_dir").as_string();
    }
    spdlog::info("Config directory: {}", configDir);
    camera_viewer::SettingsManager::instance().setConfigDir(configDir);

    // Load settings (will resolve relative paths through configDir)
    camera_viewer::SettingsManager::instance().load();
    
    // ----------------------------------------------------------------
    // Create the SDL main window and pass the ROS node to CameraManager
    // ----------------------------------------------------------------
    camera_viewer::MainWindow mainWindow(
        "Camera Visualize - ROS2 Multi-Camera Monitor", 1280, 720, g_decodeMode);
    
    // Initialize
    if (!mainWindow.initialize()) {
        spdlog::error("Failed to initialize main window");
        rclcpp::shutdown();
        return 1;
    }

    // Inject the ROS node into the camera manager so thermal streams
    // can create subscriptions.
    mainWindow.getCameraManager()->setRosNode(ros_node);

    // ----------------------------------------------------------------
    // Main loop — interleave ROS2 callbacks with SDL rendering.
    // spinOnce() does one SDL frame (~4 ms cap, ~240 FPS).
    // spin_some() drains any pending ROS2 callbacks (non-blocking).
    // ----------------------------------------------------------------
    spdlog::info("Entering main loop");
    while (mainWindow.spinOnce()) {
        rclcpp::spin_some(ros_node);
    }
    spdlog::info("Main loop ended");
    
    // ----------------------------------------------------------------
    // Cleanup
    // ----------------------------------------------------------------
    mainWindow.shutdown();
    rclcpp::shutdown();
    
    spdlog::info("Application terminated successfully");
    return 0;
}
