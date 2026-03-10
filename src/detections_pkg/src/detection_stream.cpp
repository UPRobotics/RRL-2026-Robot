#include "detections_pkg/detection_stream.h"
#include <spdlog/spdlog.h>
#include <chrono>
#include <algorithm>
#include <cstring>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/hwcontext.h>
}

namespace detections {

DetectionStream::DetectionStream(const DetectionConfig& config, SDL_Renderer* renderer)
    : m_config(config)
    , m_renderer(renderer)
{
    m_stats.state = StreamState::Disconnected;
    m_lastFpsUpdate = std::chrono::steady_clock::now();
}

DetectionStream::~DetectionStream() {
    stop();
    std::lock_guard<std::mutex> lock(m_textureMutex);
    if (m_texture) {
        SDL_DestroyTexture(m_texture);
        m_texture = nullptr;
    }
}

bool DetectionStream::start() {
    if (m_running.load()) return true;

    m_stopRequested = false;
    m_running = true;
    m_reconnectAttempts = 0;

    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats = StreamStats{};
        m_stats.state = StreamState::Connecting;
    }

    spdlog::info("Starting detection stream");
    m_thread = std::thread(&DetectionStream::decoderThread, this);
    return true;
}

void DetectionStream::stop() {
    if (!m_running.load()) return;

    spdlog::info("Stopping detection stream");
    m_stopRequested = true;
    m_running = false;

    if (m_thread.joinable()) m_thread.join();

    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = StreamState::Disconnected;
    }
}

StreamStats DetectionStream::getStats() const {
    std::lock_guard<std::mutex> lock(m_statsMutex);
    return m_stats;
}

SDL_Texture* DetectionStream::getTexture() {
    std::lock_guard<std::mutex> lock(m_textureMutex);
    return m_texture;
}

bool DetectionStream::getLatestBGRFrame(cv::Mat& out) {
    if (!m_hasNewBGR.load()) return false;
    std::lock_guard<std::mutex> lock(m_bgrMutex);
    if (m_latestBGR.empty()) return false;
    out = m_latestBGR.clone();
    m_hasNewBGR.store(false);
    return true;
}

// --------------------------------------------------------
// Decoder thread
// --------------------------------------------------------

void DetectionStream::decoderThread() {
    spdlog::debug("Decoder thread started");

    const std::string& url = m_config.rtsp_url;

    while (!m_stopRequested.load()) {
        if (!initDecoder(url)) {
            spdlog::error("Failed to init decoder for {}", url);
            attemptReconnect();
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(m_statsMutex);
            m_stats.state = StreamState::Connected;
        }
        spdlog::info("Connected to {}", url);

        while (!m_stopRequested.load() && decodeFrame()) {}

        cleanupDecoder();

        if (!m_stopRequested.load()) attemptReconnect();
    }

    cleanupDecoder();
    spdlog::debug("Decoder thread ended");
}

bool DetectionStream::initDecoder(const std::string& url) {
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = StreamState::Connecting;
    }

    m_formatCtx = avformat_alloc_context();
    if (!m_formatCtx) return false;

    // Low-latency RTSP options (same as camera_pkg)
    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);
    av_dict_set(&opts, "rtsp_flags", "prefer_tcp", 0);
    av_dict_set(&opts, "stimeout", "5000000", 0);
    av_dict_set(&opts, "analyzeduration", "0", 0);
    av_dict_set(&opts, "probesize", "8192", 0);
    av_dict_set(&opts, "fflags", "nobuffer+discardcorrupt+flush_packets+genpts", 0);
    av_dict_set(&opts, "flags", "low_delay", 0);
    av_dict_set(&opts, "max_delay", "0", 0);
    av_dict_set(&opts, "reorder_queue_size", "0", 0);
    av_dict_set(&opts, "buffer_size", "65536", 0);
    av_dict_set(&opts, "max_interleave_delta", "0", 0);
    av_dict_set(&opts, "avioflags", "direct", 0);

    spdlog::info("Connecting to: {}", url);

    int ret = avformat_open_input(&m_formatCtx, url.c_str(), nullptr, &opts);
    av_dict_free(&opts);
    if (ret < 0) {
        char buf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, buf, sizeof(buf));
        spdlog::error("Failed to open stream: {}", buf);
        avformat_free_context(m_formatCtx);
        m_formatCtx = nullptr;
        return false;
    }

    AVDictionary* findOpts = nullptr;
    av_dict_set(&findOpts, "analyzeduration", "0", 0);
    av_dict_set(&findOpts, "probesize", "8192", 0);
    m_formatCtx->max_analyze_duration = 0;
    m_formatCtx->fps_probe_size = 0;
    ret = avformat_find_stream_info(m_formatCtx, nullptr);
    av_dict_free(&findOpts);
    if (ret < 0) {
        spdlog::error("Failed to find stream info");
        avformat_close_input(&m_formatCtx);
        return false;
    }

    m_videoStreamIndex = -1;
    for (unsigned i = 0; i < m_formatCtx->nb_streams; i++) {
        if (m_formatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            m_videoStreamIndex = static_cast<int>(i);
            break;
        }
    }
    if (m_videoStreamIndex < 0) {
        spdlog::error("No video stream found");
        avformat_close_input(&m_formatCtx);
        return false;
    }

    // Codec – try GPU first, fall back to CPU
    const AVCodec* codec = nullptr;
    const AVCodecID cid = m_formatCtx->streams[m_videoStreamIndex]->codecpar->codec_id;

    // Try CUDA decoders
    {
        const char* name = nullptr;
        switch (cid) {
            case AV_CODEC_ID_H264:  name = "h264_cuvid";  break;
            case AV_CODEC_ID_HEVC:  name = "hevc_cuvid";  break;
            default: break;
        }
        if (name) {
            codec = avcodec_find_decoder_by_name(name);
            if (!codec) spdlog::warn("GPU decoder {} not found, using CPU", name);
        }
    }
    if (!codec) codec = avcodec_find_decoder(cid);
    if (!codec) {
        spdlog::error("Unsupported codec");
        avformat_close_input(&m_formatCtx);
        return false;
    }

    m_codecCtx = avcodec_alloc_context3(codec);
    if (!m_codecCtx) {
        avformat_close_input(&m_formatCtx);
        return false;
    }

    ret = avcodec_parameters_to_context(
        m_codecCtx, m_formatCtx->streams[m_videoStreamIndex]->codecpar);
    if (ret < 0) {
        avcodec_free_context(&m_codecCtx);
        avformat_close_input(&m_formatCtx);
        return false;
    }

    // GPU hw context
    if (std::string(codec->name).find("cuvid") != std::string::npos) {
        if (av_hwdevice_ctx_create(&m_hwDeviceCtx, AV_HWDEVICE_TYPE_CUDA,
                                    nullptr, nullptr, 0) < 0) {
            spdlog::warn("CUDA hwdevice failed, falling back to CPU");
            avcodec_free_context(&m_codecCtx);
            codec = avcodec_find_decoder(cid);
            m_codecCtx = avcodec_alloc_context3(codec);
            avcodec_parameters_to_context(
                m_codecCtx, m_formatCtx->streams[m_videoStreamIndex]->codecpar);
        } else {
            m_codecCtx->hw_device_ctx = av_buffer_ref(m_hwDeviceCtx);
        }
    }

    m_codecCtx->flags  |= AV_CODEC_FLAG_LOW_DELAY | AV_CODEC_FLAG_OUTPUT_CORRUPT;
    m_codecCtx->flags2 |= AV_CODEC_FLAG2_FAST | AV_CODEC_FLAG2_CHUNKS;
    m_codecCtx->thread_count = 1;
    m_codecCtx->thread_type  = 0;
    m_codecCtx->delay = 0;
    m_codecCtx->has_b_frames = 0;
    m_codecCtx->skip_loop_filter = AVDISCARD_ALL;
    m_codecCtx->skip_idct        = AVDISCARD_NONKEY;
    m_codecCtx->skip_frame       = AVDISCARD_DEFAULT;
    m_codecCtx->err_recognition  = 0;
    m_codecCtx->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;

    AVDictionary* codecOpts = nullptr;
    av_dict_set(&codecOpts, "threads", "1", 0);
    ret = avcodec_open2(m_codecCtx, codec, &codecOpts);
    av_dict_free(&codecOpts);
    if (ret < 0) {
        spdlog::error("Failed to open codec");
        avcodec_free_context(&m_codecCtx);
        avformat_close_input(&m_formatCtx);
        if (m_hwDeviceCtx) { av_buffer_unref(&m_hwDeviceCtx); m_hwDeviceCtx = nullptr; }
        return false;
    }

    m_frame     = av_frame_alloc();
    m_frameBGRA = av_frame_alloc();
    m_frameBGR  = av_frame_alloc();
    m_packet    = av_packet_alloc();
    if (!m_frame || !m_frameBGRA || !m_frameBGR || !m_packet) {
        cleanupDecoder();
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.frame_width  = m_codecCtx->width;
        m_stats.frame_height = m_codecCtx->height;
    }

    spdlog::info("Decoder initialized: {}x{} codec={}",
                 m_codecCtx->width, m_codecCtx->height,
                 avcodec_get_name(codec->id));
    return true;
}

void DetectionStream::cleanupDecoder() {
    if (m_swsCtxBGRA) { sws_freeContext(m_swsCtxBGRA); m_swsCtxBGRA = nullptr; }
    if (m_swsCtxBGR)  { sws_freeContext(m_swsCtxBGR);  m_swsCtxBGR  = nullptr; }
    if (m_hwDeviceCtx) { av_buffer_unref(&m_hwDeviceCtx); m_hwDeviceCtx = nullptr; }
    if (m_packet)   { av_packet_free(&m_packet);  m_packet  = nullptr; }
    if (m_frameBGR) {
        if (m_frameBGR->data[0]) av_freep(&m_frameBGR->data[0]);
        av_frame_free(&m_frameBGR);  m_frameBGR = nullptr;
    }
    if (m_frameBGRA) {
        if (m_frameBGRA->data[0]) av_freep(&m_frameBGRA->data[0]);
        av_frame_free(&m_frameBGRA); m_frameBGRA = nullptr;
    }
    if (m_frame)    { av_frame_free(&m_frame);   m_frame   = nullptr; }
    if (m_codecCtx) { avcodec_free_context(&m_codecCtx); m_codecCtx = nullptr; }
    if (m_formatCtx){ avformat_close_input(&m_formatCtx); m_formatCtx = nullptr; }
    m_videoStreamIndex = -1;
    m_formatLogged = false;
}

bool DetectionStream::decodeFrame() {
    int ret = av_read_frame(m_formatCtx, m_packet);
    if (ret < 0) {
        if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN)) return false;
        return false;
    }
    if (m_packet->stream_index != m_videoStreamIndex) {
        av_packet_unref(m_packet);
        return true;
    }

    ret = avcodec_send_packet(m_codecCtx, m_packet);
    av_packet_unref(m_packet);
    if (ret < 0) return (ret == AVERROR(EAGAIN));

    while (true) {
        ret = avcodec_receive_frame(m_codecCtx, m_frame);
        if (ret < 0) break;

        AVFrame* usable = m_frame;
        AVFrame* sw = nullptr;
        if (m_frame->format == AV_PIX_FMT_CUDA) {
            sw = av_frame_alloc();
            if (sw && av_hwframe_transfer_data(sw, m_frame, 0) == 0) {
                usable = sw;
            } else {
                if (sw) av_frame_free(&sw);
                av_frame_unref(m_frame);
                continue;
            }
        }

        convertFrame(usable);

        // FPS
        auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(m_statsMutex);
            m_stats.total_frames++;
            m_fpsFrameCount++;
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - m_lastFpsUpdate);
            if (elapsed.count() >= 1000) {
                m_stats.current_fps = m_fpsFrameCount * 1000.0f / elapsed.count();
                m_fpsFrameCount = 0;
                m_lastFpsUpdate = now;
            }
        }

        if (sw) av_frame_free(&sw);
        av_frame_unref(m_frame);
    }
    return true;
}

bool DetectionStream::convertFrame(AVFrame* frame) {
    if (!frame || frame->width == 0 || frame->height == 0) return false;

    auto srcFmt = static_cast<AVPixelFormat>(frame->format);

    if (!m_formatLogged) {
        spdlog::info("Frame format={} size={}x{}",
                     av_get_pix_fmt_name(srcFmt), frame->width, frame->height);
        m_formatLogged = true;
    }

    int w = frame->width;
    int h = frame->height;

    // ---- BGRA for SDL ----
    if (!m_swsCtxBGRA) {
        m_swsCtxBGRA = sws_getContext(
            w, h, srcFmt, w, h, AV_PIX_FMT_BGRA,
            SWS_POINT, nullptr, nullptr, nullptr);
        if (!m_swsCtxBGRA) return false;

        int sz = av_image_get_buffer_size(AV_PIX_FMT_BGRA, w, h, 1);
        uint8_t* buf = static_cast<uint8_t*>(av_malloc(sz));
        if (!buf) { sws_freeContext(m_swsCtxBGRA); m_swsCtxBGRA = nullptr; return false; }
        av_image_fill_arrays(m_frameBGRA->data, m_frameBGRA->linesize,
                             buf, AV_PIX_FMT_BGRA, w, h, 1);
        m_frameBGRA->width  = w;
        m_frameBGRA->height = h;
        m_frameBGRA->format = AV_PIX_FMT_BGRA;
    }

    sws_scale(m_swsCtxBGRA, frame->data, frame->linesize, 0, h,
              m_frameBGRA->data, m_frameBGRA->linesize);

    // Store for main-thread texture upload
    {
        std::lock_guard<std::mutex> lock(m_pendingMutex);
        int dataSize = m_frameBGRA->linesize[0] * h;
        m_pendingFrameData.resize(dataSize);
        memcpy(m_pendingFrameData.data(), m_frameBGRA->data[0], dataSize);
        m_pendingWidth  = w;
        m_pendingHeight = h;
        m_pendingPitch  = m_frameBGRA->linesize[0];
        m_hasPendingFrame.store(true);
    }

    // ---- BGR24 for OpenCV ----
    if (!m_swsCtxBGR) {
        m_swsCtxBGR = sws_getContext(
            w, h, srcFmt, w, h, AV_PIX_FMT_BGR24,
            SWS_POINT, nullptr, nullptr, nullptr);
        if (!m_swsCtxBGR) return true;  // SDL frame still ok

        int sz = av_image_get_buffer_size(AV_PIX_FMT_BGR24, w, h, 1);
        uint8_t* buf = static_cast<uint8_t*>(av_malloc(sz));
        if (!buf) { sws_freeContext(m_swsCtxBGR); m_swsCtxBGR = nullptr; return true; }
        av_image_fill_arrays(m_frameBGR->data, m_frameBGR->linesize,
                             buf, AV_PIX_FMT_BGR24, w, h, 1);
        m_frameBGR->width  = w;
        m_frameBGR->height = h;
        m_frameBGR->format = AV_PIX_FMT_BGR24;
    }

    sws_scale(m_swsCtxBGR, frame->data, frame->linesize, 0, h,
              m_frameBGR->data, m_frameBGR->linesize);

    {
        std::lock_guard<std::mutex> lock(m_bgrMutex);
        m_latestBGR = cv::Mat(h, w, CV_8UC3, m_frameBGR->data[0],
                              m_frameBGR->linesize[0]).clone();
        m_hasNewBGR.store(true);
    }

    return true;
}

bool DetectionStream::updateTextureFromMainThread() {
    if (!m_hasPendingFrame.load()) return false;

    std::lock_guard<std::mutex> fLock(m_pendingMutex);
    std::lock_guard<std::mutex> tLock(m_textureMutex);

    if (m_pendingFrameData.empty()) return false;

    if (!m_texture) {
        m_texture = SDL_CreateTexture(
            m_renderer, SDL_PIXELFORMAT_ARGB8888,
            SDL_TEXTUREACCESS_STREAMING,
            m_pendingWidth, m_pendingHeight);
        if (!m_texture) {
            spdlog::error("Failed to create texture: {}", SDL_GetError());
            return false;
        }
    }

    void* pixels = nullptr;
    int pitch = 0;
    if (SDL_LockTexture(m_texture, nullptr, &pixels, &pitch) == 0) {
        if (pitch == m_pendingPitch) {
            memcpy(pixels, m_pendingFrameData.data(),
                   m_pendingPitch * m_pendingHeight);
        } else {
            const uint8_t* src = m_pendingFrameData.data();
            uint8_t* dst = static_cast<uint8_t*>(pixels);
            int rowBytes = std::min(pitch, m_pendingPitch);
            for (int row = 0; row < m_pendingHeight; ++row) {
                memcpy(dst, src, rowBytes);
                src += m_pendingPitch;
                dst += pitch;
            }
        }
        SDL_UnlockTexture(m_texture);
    } else {
        SDL_UpdateTexture(m_texture, nullptr,
                          m_pendingFrameData.data(), m_pendingPitch);
    }

    m_hasPendingFrame.store(false);
    return true;
}

void DetectionStream::attemptReconnect() {
    m_reconnectAttempts++;
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = StreamState::Reconnecting;
    }

    if (m_reconnectAttempts > MAX_RECONNECT) {
        spdlog::error("Max reconnect attempts reached");
        {
            std::lock_guard<std::mutex> lock(m_statsMutex);
            m_stats.state = StreamState::Error;
        }
        m_stopRequested = true;
        return;
    }

    spdlog::info("Reconnect attempt {}/{} in {}ms",
                 m_reconnectAttempts, MAX_RECONNECT, RECONNECT_DELAY_MS);
    std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
}

} // namespace detections
