#pragma once

#include <cstdint>
#include <string>
#include <chrono>
#include <atomic>

namespace camera_viewer {

// Basic types
using TimePoint = std::chrono::steady_clock::time_point;
using Duration = std::chrono::milliseconds;

// Stream quality
enum class StreamQuality {
    High,   // 1080p
    Low     // 480p
};

// Camera connection state
enum class CameraState {
    Disconnected,   // Not connected
    Connecting,     // Attempting to connect
    Connected,      // Streaming successfully
    Reconnecting,   // Lost connection, trying to reconnect
    Error           // Fatal error
};

// Decode mode
enum class DecodeMode {
    CPU,
    GPU
};

// Camera source type
enum class CameraSourceType {
    RTSP,       // Traditional RTSP camera via FFmpeg
    Thermal     // ROS2 thermal camera topic
};

// Camera configuration from settings.json
struct CameraConfig {
    std::string id;
    std::string name;
    std::string ip;
    int port = 554;
    std::string url_highres;
    std::string url_lowres;
    bool enabled = true;
    int rotation_deg = 0; // 0, 90, 180, 270
    CameraSourceType source_type = CameraSourceType::RTSP;
    std::string ros_topic;  // For ROS-based cameras (e.g. "/thermal_data")
    
    // Runtime state
    bool available = false;  // Set after ping check
};

// Camera statistics
struct CameraStats {
    uint64_t total_frames = 0;
    uint64_t dropped_frames = 0;
    float current_fps = 0.0f;
    float latency_ms = 0.0f;
    uint32_t reconnect_count = 0;
    TimePoint last_frame_time;
    CameraState state = CameraState::Disconnected;
    int frame_width = 0;
    int frame_height = 0;
};

// Streaming settings
struct StreamingSettings {
    StreamQuality default_quality = StreamQuality::High;
    int ping_timeout_ms = 1000;
    int connection_timeout_ms = 5000;
    int reconnect_delay_ms = 2000;
    int max_reconnect_attempts = 5;
    int frame_buffer_size = 1;
    int frame_timeout_ms = 5000;  // Auto-restart camera if no frames for this duration
};

// Frame data structure for decoded video
struct VideoFrame {
    uint8_t* data = nullptr;
    int width = 0;
    int height = 0;
    int linesize = 0;
    TimePoint timestamp;
    
    // Disable copy, enable move
    VideoFrame() = default;
    VideoFrame(const VideoFrame&) = delete;
    VideoFrame& operator=(const VideoFrame&) = delete;
    VideoFrame(VideoFrame&& other) noexcept 
        : data(other.data), width(other.width), height(other.height), 
          linesize(other.linesize), timestamp(other.timestamp) {
        other.data = nullptr;
    }
    VideoFrame& operator=(VideoFrame&& other) noexcept {
        if (this != &other) {
            delete[] data;
            data = other.data;
            width = other.width;
            height = other.height;
            linesize = other.linesize;
            timestamp = other.timestamp;
            other.data = nullptr;
        }
        return *this;
    }
    ~VideoFrame() {
        delete[] data;
    }
};

} // namespace camera_viewer
