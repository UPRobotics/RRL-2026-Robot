#pragma once
#include <cstdint>
#include <string>
#include <chrono>

namespace osmo {

using TimePoint = std::chrono::steady_clock::time_point;

struct CameraConfig {
    std::string id;
    std::string name;
    std::string device;
    std::string topic;
    int width  = 1920;
    int height = 1080;
    int fps    = 60;
    bool enabled = true;
};

struct DisplayConfig {
    int window_width  = 1920;
    int window_height = 540;
};

struct CaptureStats {
    uint64_t  total_frames   = 0;
    uint64_t  dropped_frames = 0;
    float     current_fps    = 0.0f;
    TimePoint last_frame_time;
};

}  // namespace osmo
