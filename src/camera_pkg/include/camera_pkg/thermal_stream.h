#pragma once

#include "camera_pkg/types.h"
#include <SDL2/SDL.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <atomic>
#include <mutex>
#include <vector>
#include <functional>

namespace camera_viewer {

/**
 * @brief Renders a thermal camera feed from a ROS2 Float32MultiArray topic
 * 
 * Subscribes to a ROS2 topic carrying a 32x24 array of temperature floats,
 * converts them into a false-color BGRA image, and exposes the result as an
 * SDL_Texture that the CameraGrid can render alongside RTSP cameras.
 * 
 * The subscription callback runs on the ROS2 executor thread and writes to a
 * lock-free pending-frame buffer.  The main/render thread calls
 * updateTextureFromMainThread() each frame to upload the latest data to the GPU
 * — exactly the same pattern used by CameraStream for RTSP cameras.
 * 
 * If the topic stops publishing, the last received frame stays on screen and
 * the state transitions to Disconnected after a configurable timeout.  This
 * never blocks or affects other cameras.
 */
class ThermalStream {
public:
    using FrameCallback = std::function<void(int cameraIndex, SDL_Texture* texture, const CameraStats& stats)>;

    /**
     * @param cameraIndex  Logical camera index used by CameraManager
     * @param config       Camera configuration (ros_topic must be set)
     * @param renderer     SDL renderer (not owned)
     * @param node         Shared pointer to the ROS2 node used for subscriptions
     */
    ThermalStream(int cameraIndex, const CameraConfig& config,
                  SDL_Renderer* renderer, rclcpp::Node::SharedPtr node);
    ~ThermalStream();

    // Non-copyable
    ThermalStream(const ThermalStream&) = delete;
    ThermalStream& operator=(const ThermalStream&) = delete;

    /** Start subscribing to the thermal topic */
    bool start(StreamQuality quality = StreamQuality::High);

    /** Stop the subscription */
    void stop();

    bool isRunning() const { return m_running.load(); }
    CameraState getState() const;
    CameraStats getStats() const;

    /** Thread-safe access to the current texture */
    SDL_Texture* getFrameTexture();

    /** Upload pending pixel data to SDL texture (MUST be called from render thread) */
    bool updateTextureFromMainThread();

    void setFrameCallback(FrameCallback cb);
    void setQuality(StreamQuality /*quality*/) { /* thermal has one resolution */ }

    int getCameraIndex() const { return m_cameraIndex; }
    const CameraConfig& getConfig() const { return m_config; }

private:
    /** ROS2 subscription callback — runs on executor thread */
    void topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /** Map temperature value to a false-color BGRA pixel */
    static void temperatureToColor(float temp, float minTemp, float maxTemp,
                                   uint8_t& b, uint8_t& g, uint8_t& r, uint8_t& a);

    // Identity
    int m_cameraIndex;
    CameraConfig m_config;

    // SDL (renderer not owned)
    SDL_Renderer* m_renderer;
    SDL_Texture* m_texture = nullptr;
    std::mutex m_textureMutex;

    // Pending frame buffer (written by ROS callback, read by main thread)
    std::vector<uint8_t> m_pendingFrameData;
    int m_pendingFrameWidth = 0;
    int m_pendingFrameHeight = 0;
    int m_pendingFramePitch = 0;
    std::atomic<bool> m_hasPendingFrame{false};
    std::mutex m_pendingFrameMutex;

    // ROS2
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_subscription;

    // State
    std::atomic<bool> m_running{false};
    mutable std::mutex m_statsMutex;
    CameraStats m_stats;
    TimePoint m_lastFpsUpdate;
    int m_frameCountSinceLastUpdate = 0;

    // Callback
    FrameCallback m_frameCallback;
    std::mutex m_callbackMutex;

    // Thermal sensor dimensions
    static constexpr int SENSOR_WIDTH  = 32;
    static constexpr int SENSOR_HEIGHT = 24;

    // Scale factor for display (render at SENSOR_WIDTH*SCALE x SENSOR_HEIGHT*SCALE)
    static constexpr int DISPLAY_SCALE = 8;
    static constexpr int DISPLAY_WIDTH  = SENSOR_WIDTH  * DISPLAY_SCALE;  // 256
    static constexpr int DISPLAY_HEIGHT = SENSOR_HEIGHT * DISPLAY_SCALE;  // 192
};

} // namespace camera_viewer
