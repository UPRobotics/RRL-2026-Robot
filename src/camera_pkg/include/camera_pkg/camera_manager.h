#pragma once

#include "camera_pkg/types.h"
#include "camera_pkg/camera_stream.h"
#include <SDL2/SDL.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <functional>

namespace camera_viewer {

/** @brief Manages all RTSP camera streams including discovery and lifecycle */
class CameraManager {
public:
    using DiscoveryCallback = std::function<void(int availableCount, int totalCount)>;
    using StateChangeCallback = std::function<void(int cameraIndex, CameraState newState)>;
    
    explicit CameraManager(SDL_Renderer* renderer, rclcpp::Node::SharedPtr node = nullptr);
    ~CameraManager();
    
    // Disable copy
    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

    /** Set the ROS2 node (can be called after construction) */
    void setRosNode(rclcpp::Node::SharedPtr node) { m_rosNode = node; }
    
    void setCameraConfigs(const std::vector<CameraConfig>& configs);
    void setStreamingSettings(const StreamingSettings& settings);
    void setDecodeMode(DecodeMode mode) { m_decodeMode = mode; }
    
    int discoverCameras(DiscoveryCallback callback = nullptr);
    
    void startAll(StreamQuality quality = StreamQuality::High);
    void stopAll();
    void restartAll();
    
    bool startCamera(int index, StreamQuality quality = StreamQuality::High);
    void stopCamera(int index);
    
    int getCameraCount() const { return static_cast<int>(m_configs.size()); }
    int getAvailableCameraCount() const;
    std::vector<int> getAvailableCameraIndices() const;
    int getRunningCameraCount() const;
    
    const CameraConfig& getCameraConfig(int index) const;
    int getCameraRotation(int index) const;
    void setCameraRotation(int index, int rotationDeg);
    
    CameraStats getCameraStats(int index) const;
    SDL_Texture* getCameraTexture(int index);
    
    void updateTexturesFromMainThread();
    
    bool isCameraAvailable(int index) const;
    void setStateChangeCallback(StateChangeCallback callback);
    void setAllQuality(StreamQuality quality);
    
    float getAverageFps() const;
    float getAverageLatency() const;
    
    void checkAutoRecovery();
    
private:
    bool pingHost(const std::string& ip, int timeoutMs);
    void discoveryThread(DiscoveryCallback callback);
    
    using StreamVar = std::unique_ptr<CameraStream>;

    bool streamIsRunning(const StreamVar& sv) const;
    void streamStop(StreamVar& sv);
    CameraStats streamGetStats(const StreamVar& sv) const;
    SDL_Texture* streamGetTexture(StreamVar& sv);
    void streamUpdateTexture(StreamVar& sv);
    
    // Renderer (not owned)
    SDL_Renderer* m_renderer;
    
    // ROS2 node (shared, not owned)
    rclcpp::Node::SharedPtr m_rosNode;
    
    // Camera configurations
    std::vector<CameraConfig> m_configs;
    mutable std::mutex m_configMutex;
    
    // Camera streams (RTSP or Thermal)
    std::vector<StreamVar> m_streams;
    mutable std::mutex m_streamsMutex;
    
    // Settings
    StreamingSettings m_settings;
    StreamQuality m_currentQuality = StreamQuality::High;
    DecodeMode m_decodeMode = DecodeMode::GPU;
    
    // Callbacks
    StateChangeCallback m_stateChangeCallback;
    
    // Discovery
    std::atomic<bool> m_discoveryRunning{false};
    std::thread m_discoveryThread;
    
    // Auto-recovery tracking
    std::chrono::steady_clock::time_point m_lastRecoveryCheck;
    
    // Recovery queue
    std::vector<size_t> m_recoveryQueue;
    std::mutex m_recoveryMutex;
    std::thread m_recoveryThread;
    std::atomic<bool> m_recoveryRunning{false};
    
    void recoveryThreadFunc();
};

} // namespace camera_viewer
