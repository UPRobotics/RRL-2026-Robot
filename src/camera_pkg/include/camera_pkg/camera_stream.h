#pragma once

#include "camera_pkg/types.h"
#include <SDL2/SDL.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <memory>
#include <functional>
#include <vector>

// Forward declarations for FFmpeg
extern "C" {
struct AVFormatContext;
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;
struct AVBufferRef;
}

namespace camera_viewer {

/**
 * @brief Manages a single camera RTSP stream with its own decoder thread
 * 
 * Each CameraStream runs in its own thread to ensure isolation - if one
 * camera freezes or crashes, it doesn't affect other cameras.
 */
class CameraStream {
public:
    using FrameCallback = std::function<void(int cameraIndex, SDL_Texture* texture, const CameraStats& stats)>;
    
    explicit CameraStream(int cameraIndex, const CameraConfig& config, SDL_Renderer* renderer, DecodeMode decodeMode);
    ~CameraStream();
    
    // Disable copy
    CameraStream(const CameraStream&) = delete;
    CameraStream& operator=(const CameraStream&) = delete;
    
    /**
     * @brief Start the camera stream
     * @param quality Stream quality to use (High = 1080p, Low = 480p)
     */
    bool start(StreamQuality quality = StreamQuality::High);
    
    /**
     * @brief Stop the camera stream
     */
    void stop();
    
    /**
     * @brief Check if stream is running
     */
    bool isRunning() const { return m_running.load(); }
    
    /**
     * @brief Get current camera state
     */
    CameraState getState() const { return m_stats.state; }
    
    /**
     * @brief Get camera statistics
     */
    CameraStats getStats() const;
    
    /**
     * @brief Get the current frame texture (thread-safe)
     * @return SDL_Texture* or nullptr if no frame available
     */
    SDL_Texture* getFrameTexture();
    
    /**
     * @brief Update texture from pending frame data (MUST be called from main/render thread)
     * @return true if texture was updated
     */
    bool updateTextureFromMainThread();
    
    /**
     * @brief Set callback for frame updates
     */
    void setFrameCallback(FrameCallback callback);
    
    /**
     * @brief Switch stream quality
     */
    void setQuality(StreamQuality quality);
    
    /**
     * @brief Get camera index
     */
    int getCameraIndex() const { return m_cameraIndex; }
    
    /**
     * @brief Get camera config
     */
    const CameraConfig& getConfig() const { return m_config; }
    
private:
    /**
     * @brief Main decoder thread function
     */
    void decoderThread();
    
    /**
     * @brief Initialize FFmpeg decoder
     */
    bool initDecoder(const std::string& url);
    
    /**
     * @brief Cleanup FFmpeg resources
     */
    void cleanupDecoder();
    
    /**
     * @brief Decode a single frame
     */
    bool decodeFrame();
    
    /**
     * @brief Update the SDL texture with decoded frame
     */
    bool updateTexture(AVFrame* frame);
    
    /**
     * @brief Attempt to reconnect after connection loss
     */
    void attemptReconnect();
    
    // Camera configuration
    int m_cameraIndex;
    CameraConfig m_config;
    StreamQuality m_currentQuality;
    DecodeMode m_decodeMode;
    
    // SDL renderer (not owned)
    SDL_Renderer* m_renderer;
    
    // SDL texture for rendering (owned)
    SDL_Texture* m_texture = nullptr;
    std::mutex m_textureMutex;
    
    // Pending frame data for main thread texture update
    std::vector<uint8_t> m_pendingFrameData;
    int m_pendingFrameWidth = 0;
    int m_pendingFrameHeight = 0;
    int m_pendingFramePitch = 0;
    std::atomic<bool> m_hasPendingFrame{false};
    std::mutex m_pendingFrameMutex;
    
    // Thread control
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_stopRequested{false};
    std::thread m_thread;
    
    // FFmpeg context (owned by decoder thread)
    AVFormatContext* m_formatCtx = nullptr;
    AVCodecContext* m_codecCtx = nullptr;
    AVFrame* m_frame = nullptr;
    AVFrame* m_frameRGB = nullptr;
    AVPacket* m_packet = nullptr;
    SwsContext* m_swsCtx = nullptr;
    int m_videoStreamIndex = -1;
    AVBufferRef* m_hwDeviceCtx = nullptr;
    
    // Statistics
    mutable std::mutex m_statsMutex;
    CameraStats m_stats;
    TimePoint m_lastFpsUpdate;
    int m_frameCountSinceLastUpdate = 0;
    bool m_formatLogged = false;
    
    // Reconnection
    int m_reconnectAttempts = 0;
    static constexpr int MAX_RECONNECT_ATTEMPTS = 5;
    static constexpr int RECONNECT_DELAY_MS = 2000;
    
    // Frame callback
    FrameCallback m_frameCallback;
    std::mutex m_callbackMutex;
};

} // namespace camera_viewer
