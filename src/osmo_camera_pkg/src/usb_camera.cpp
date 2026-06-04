#include "osmo_camera_pkg/usb_camera.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <cstring>
#include <string>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavutil/log.h>
#include <libavutil/error.h>
}

namespace osmo {

UsbCamera::UsbCamera(const CameraConfig& config)
    : m_config(config)
{
    m_lastFpsUpdate = std::chrono::steady_clock::now();
    m_stats.last_frame_time = m_lastFpsUpdate;
}

UsbCamera::~UsbCamera() {
    stop();
}

void UsbCamera::setFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lk(m_callbackMutex);
    m_callback = std::move(cb);
}

bool UsbCamera::start() {
    if (m_running.load()) return true;
    m_stopRequested = false;
    m_running = true;
    m_thread = std::thread(&UsbCamera::captureThread, this);
    return true;
}

void UsbCamera::stop() {
    if (!m_running.load()) return;
    m_stopRequested = true;
    m_running = false;
    if (m_thread.joinable()) m_thread.join();
}

CaptureStats UsbCamera::getStats() const {
    std::lock_guard<std::mutex> lk(m_statsMutex);
    return m_stats;
}

bool UsbCamera::openDevice() {
    avdevice_register_all();

    AVInputFormat* v4l2Fmt = const_cast<AVInputFormat*>(av_find_input_format("v4l2"));
    if (!v4l2Fmt) {
        spdlog::error("[{}] v4l2 input format not available", m_config.id);
        return false;
    }

    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "input_format",    "mjpeg",                        0);
    av_dict_set(&opts, "video_size",
        (std::to_string(m_config.width) + "x" +
         std::to_string(m_config.height)).c_str(),                        0);
    av_dict_set(&opts, "framerate",       std::to_string(m_config.fps).c_str(), 0);
    av_dict_set(&opts, "analyzeduration", "0",                            0);
    av_dict_set(&opts, "probesize",       "8192",                         0);
    av_dict_set(&opts, "fflags",          "nobuffer+flush_packets",       0);
    av_dict_set(&opts, "flags",           "low_delay",                    0);

    m_fmtCtx = avformat_alloc_context();
    int ret = avformat_open_input(&m_fmtCtx, m_config.device.c_str(), v4l2Fmt, &opts);
    av_dict_free(&opts);

    if (ret < 0) {
        char buf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, buf, sizeof(buf));
        spdlog::error("[{}] cannot open {}: {}", m_config.id, m_config.device, buf);
        avformat_free_context(m_fmtCtx);
        m_fmtCtx = nullptr;
        return false;
    }

    avformat_find_stream_info(m_fmtCtx, nullptr);

    for (unsigned i = 0; i < m_fmtCtx->nb_streams; ++i) {
        if (m_fmtCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            m_videoStream = static_cast<int>(i);
            break;
        }
    }

    if (m_videoStream < 0) {
        spdlog::error("[{}] no video stream in {}", m_config.id, m_config.device);
        avformat_close_input(&m_fmtCtx);
        return false;
    }

    const auto* par = m_fmtCtx->streams[m_videoStream]->codecpar;
    spdlog::info("[{}] opened {} @ {}x{} codec={}",
        m_config.id, m_config.device, par->width, par->height,
        avcodec_get_name(par->codec_id));

    m_pkt = av_packet_alloc();
    return true;
}

void UsbCamera::closeDevice() {
    if (m_pkt) {
        av_packet_free(&m_pkt);
        m_pkt = nullptr;
    }
    if (m_fmtCtx) {
        avformat_close_input(&m_fmtCtx);
        m_fmtCtx = nullptr;
    }
    m_videoStream = -1;
}

void UsbCamera::updateStats() {
    auto now = std::chrono::steady_clock::now();
    m_fpsCount++;

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_lastFpsUpdate).count();

    if (elapsed >= 1000) {
        std::lock_guard<std::mutex> lk(m_statsMutex);
        m_stats.current_fps   = m_fpsCount * 1000.0f / static_cast<float>(elapsed);
        m_stats.last_frame_time = now;
        m_fpsCount = 0;
        m_lastFpsUpdate = now;
    }

    {
        std::lock_guard<std::mutex> lk(m_statsMutex);
        m_stats.total_frames++;
        m_stats.last_frame_time = now;
    }
}

void UsbCamera::captureThread() {
    spdlog::info("[{}] capture thread started", m_config.id);

    while (!m_stopRequested.load()) {
        if (!openDevice()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            continue;
        }

        while (!m_stopRequested.load()) {
            int ret = av_read_frame(m_fmtCtx, m_pkt);
            if (ret < 0) {
                if (ret != AVERROR_EOF) {
                    char buf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, buf, sizeof(buf));
                    spdlog::warn("[{}] av_read_frame: {}", m_config.id, buf);
                }
                break;
            }

            if (m_pkt->stream_index == m_videoStream && m_pkt->size > 0) {
                {
                    std::lock_guard<std::mutex> lk(m_callbackMutex);
                    if (m_callback) {
                        m_callback(m_pkt->data, m_pkt->size);
                    }
                }
                updateStats();
            }
            av_packet_unref(m_pkt);
        }

        closeDevice();

        if (!m_stopRequested.load()) {
            spdlog::warn("[{}] disconnected, retrying in 2s", m_config.id);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
    }

    spdlog::info("[{}] capture thread stopped", m_config.id);
}

}  // namespace osmo
