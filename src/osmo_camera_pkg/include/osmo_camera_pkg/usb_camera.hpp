#pragma once
#include "osmo_camera_pkg/types.hpp"
#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

extern "C" {
struct AVFormatContext;
struct AVPacket;
}

namespace osmo {

class UsbCamera {
public:
    using FrameCallback = std::function<void(const uint8_t* data, int size)>;

    explicit UsbCamera(const CameraConfig& config);
    ~UsbCamera();

    UsbCamera(const UsbCamera&)            = delete;
    UsbCamera& operator=(const UsbCamera&) = delete;

    bool start();
    void stop();
    bool isRunning() const { return m_running.load(); }

    void setFrameCallback(FrameCallback cb);
    CaptureStats getStats() const;

private:
    void captureThread();
    bool openDevice();
    void closeDevice();
    void updateStats();

    CameraConfig m_config;

    std::atomic<bool> m_running{false};
    std::atomic<bool> m_stopRequested{false};
    std::thread       m_thread;

    AVFormatContext* m_fmtCtx     = nullptr;
    AVPacket*        m_pkt        = nullptr;
    int              m_videoStream = -1;

    FrameCallback m_callback;
    std::mutex    m_callbackMutex;

    mutable std::mutex m_statsMutex;
    CaptureStats       m_stats;
    TimePoint          m_lastFpsUpdate;
    int                m_fpsCount = 0;
};

}  // namespace osmo
