#pragma once

#include "detections_pkg/types.h"
#include <SDL2/SDL.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <functional>

extern "C" {
struct AVFormatContext;
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;
struct AVBufferRef;
}

namespace detections {

/// Decodes a single RTSP stream in a background thread.
/// Produces BGRA pixel data for SDL texture upload AND
/// BGR24 cv::Mat frames for OpenCV detections.
class DetectionStream {
public:
    explicit DetectionStream(const DetectionConfig& config, SDL_Renderer* renderer);
    ~DetectionStream();

    DetectionStream(const DetectionStream&) = delete;
    DetectionStream& operator=(const DetectionStream&) = delete;

    bool start();
    void stop();
    bool isRunning() const { return m_running.load(); }

    StreamStats getStats() const;

    /// Call from main thread – uploads pending frame to SDL texture.
    /// Returns true if a new frame was uploaded.
    bool updateTextureFromMainThread();

    /// Get the SDL texture for rendering (call after updateTextureFromMainThread).
    SDL_Texture* getTexture();

    /// Get latest BGR frame for OpenCV processing (thread-safe copy).
    bool getLatestBGRFrame(cv::Mat& out);

private:
    void decoderThread();
    bool initDecoder(const std::string& url);
    void cleanupDecoder();
    bool decodeFrame();
    bool convertFrame(AVFrame* frame);
    void attemptReconnect();

    DetectionConfig m_config;
    SDL_Renderer* m_renderer;

    // Thread control
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_stopRequested{false};
    std::thread m_thread;

    // FFmpeg
    AVFormatContext* m_formatCtx = nullptr;
    AVCodecContext*  m_codecCtx  = nullptr;
    AVFrame*         m_frame     = nullptr;
    AVFrame*         m_frameBGRA = nullptr;
    AVFrame*         m_frameBGR  = nullptr;
    AVPacket*        m_packet    = nullptr;
    SwsContext*      m_swsCtxBGRA = nullptr;
    SwsContext*      m_swsCtxBGR  = nullptr;
    int m_videoStreamIndex = -1;
    AVBufferRef* m_hwDeviceCtx = nullptr;

    // Pending BGRA frame for SDL texture
    std::vector<uint8_t> m_pendingFrameData;
    int m_pendingWidth = 0;
    int m_pendingHeight = 0;
    int m_pendingPitch = 0;
    std::atomic<bool> m_hasPendingFrame{false};
    std::mutex m_pendingMutex;

    // Latest BGR frame for OpenCV
    cv::Mat m_latestBGR;
    std::mutex m_bgrMutex;
    std::atomic<bool> m_hasNewBGR{false};

    // SDL texture
    SDL_Texture* m_texture = nullptr;
    std::mutex m_textureMutex;

    // Stats
    mutable std::mutex m_statsMutex;
    StreamStats m_stats;
    TimePoint m_lastFpsUpdate;
    int m_fpsFrameCount = 0;
    bool m_formatLogged = false;

    // Reconnect
    int m_reconnectAttempts = 0;
    static constexpr int MAX_RECONNECT = 10;
    static constexpr int RECONNECT_DELAY_MS = 3000;
};

} // namespace detections
