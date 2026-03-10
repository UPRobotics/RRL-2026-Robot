#pragma once

#include <cstdint>
#include <string>
#include <chrono>
#include <vector>
#include <opencv2/core.hpp>

namespace detections {

using TimePoint = std::chrono::steady_clock::time_point;

// Camera connection state
enum class StreamState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Error
};

// Detected QR code
struct QRDetection {
    std::string data;
    std::vector<cv::Point> points;  // 4 corner points
};

// Detected motion region
struct MotionBox {
    int x, y, w, h;
};

// Magnetometer reading
struct MagReading {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double magnitude = 0.0;
    std::string unit = "uT";
    TimePoint timestamp;
};

// Camera stream stats
struct StreamStats {
    uint64_t total_frames = 0;
    float current_fps = 0.0f;
    StreamState state = StreamState::Disconnected;
    int frame_width = 0;
    int frame_height = 0;
};

// Configuration loaded from JSON
struct DetectionConfig {
    // Camera
    std::string rtsp_url;
    std::string rtsp_url_lowres;

    // Detection parameters
    int qr_detect_interval = 5;       // run QR detection every N frames
    int motion_min_area = 500;
    int motion_skip_frames = 5;
    double motion_threshold = 30.0;
    double motion_accumulate_weight = 0.05;

    // Display
    int window_width = 1280;
    int window_height = 720;
    int mag_panel_width = 300;

    // Runtime (not serialized)
    std::string config_path;  // path to JSON file for saving
};

} // namespace detections
