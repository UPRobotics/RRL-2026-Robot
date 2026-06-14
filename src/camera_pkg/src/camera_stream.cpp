#include "camera_pkg/camera_stream.h"
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

namespace camera_viewer {

CameraStream::CameraStream(int cameraIndex, const CameraConfig& config, SDL_Renderer* renderer, DecodeMode decodeMode)
    : m_cameraIndex(cameraIndex)
    , m_config(config)
    , m_currentQuality(StreamQuality::High)
    , m_decodeMode(decodeMode)
    , m_renderer(renderer)
{
    m_stats.state = CameraState::Disconnected;
    m_lastFpsUpdate = std::chrono::steady_clock::now();
}

CameraStream::~CameraStream() {
    stop();
    
    std::lock_guard<std::mutex> lock(m_textureMutex);
    if (m_texture) {
        SDL_DestroyTexture(m_texture);
        m_texture = nullptr;
    }
}

bool CameraStream::start(StreamQuality quality) {
    if (m_running.load()) {
        spdlog::warn("Camera {} stream already running", m_cameraIndex + 1);
        return true;
    }
    
    m_currentQuality = quality;
    m_stopRequested = false;
    m_running = true;
    m_reconnectAttempts = 0;
    
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = CameraState::Connecting;
        m_stats.total_frames = 0;
        m_stats.dropped_frames = 0;
        m_stats.current_fps = 0;
        m_stats.reconnect_count = 0;
        m_stats.last_frame_time = std::chrono::steady_clock::now();
    }
    
    spdlog::info("Starting camera {} stream ({})", 
                 m_cameraIndex + 1, 
                 quality == StreamQuality::High ? "1080p" : "480p");
    
    m_thread = std::thread(&CameraStream::decoderThread, this);
    return true;
}

void CameraStream::stop() {
    if (!m_running.load()) {
        return;
    }
    
    spdlog::info("Stopping camera {} stream", m_cameraIndex + 1);
    
    m_stopRequested = true;
    m_running = false;
    
    if (m_thread.joinable()) {
        m_thread.join();
    }
    
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = CameraState::Disconnected;
    }
}

CameraStats CameraStream::getStats() const {
    std::lock_guard<std::mutex> lock(m_statsMutex);
    return m_stats;
}

SDL_Texture* CameraStream::getFrameTexture() {
    std::lock_guard<std::mutex> lock(m_textureMutex);
    return m_texture;
}

void CameraStream::setFrameCallback(FrameCallback callback) {
    std::lock_guard<std::mutex> lock(m_callbackMutex);
    m_frameCallback = std::move(callback);
}

void CameraStream::setQuality(StreamQuality quality) {
    if (m_currentQuality == quality) {
        return;
    }
    
    spdlog::info("Camera {} switching to {} quality", 
                 m_cameraIndex + 1,
                 quality == StreamQuality::High ? "high" : "low");
    
    bool wasRunning = m_running.load();
    if (wasRunning) {
        stop();
        m_currentQuality = quality;
        start(quality);
    } else {
        m_currentQuality = quality;
    }
}

void CameraStream::decoderThread() {
    spdlog::debug("Camera {} decoder thread started", m_cameraIndex + 1);
    
    const std::string& url = (m_currentQuality == StreamQuality::High) 
                             ? m_config.url_highres 
                             : m_config.url_lowres;
    
    while (!m_stopRequested.load()) {
        if (!initDecoder(url)) {
            spdlog::error("Camera {} failed to initialize decoder for {}", m_cameraIndex + 1, url);
            attemptReconnect();
            continue;
        }
        
        {
            std::lock_guard<std::mutex> lock(m_statsMutex);
            m_stats.state = CameraState::Connected;
        }
        
        spdlog::info("Camera {} connected successfully", m_cameraIndex + 1);
        
        while (!m_stopRequested.load() && decodeFrame()) {
        }
        
        cleanupDecoder();
        
        if (!m_stopRequested.load()) {
            attemptReconnect();
        }
    }
    
    cleanupDecoder();
    spdlog::debug("Camera {} decoder thread ended", m_cameraIndex + 1);
}

static enum AVPixelFormat get_hw_format(AVCodecContext* ctx, const enum AVPixelFormat* pix_fmts) {
    (void)ctx;
    const enum AVPixelFormat* p;
    for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
        if (*p == AV_PIX_FMT_CUDA) {
            return *p;
        }
    }
    return pix_fmts[0];
}

bool CameraStream::initDecoder(const std::string& url) {
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = CameraState::Connecting;
    }
    
    m_formatCtx = avformat_alloc_context();
    if (!m_formatCtx) {
        spdlog::error("Camera {} failed to allocate format context", m_cameraIndex + 1);
        return false;
    }
    
    // Set RTSP options for absolute minimum latency
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
    
    spdlog::info("Camera {} connecting to: {}", m_cameraIndex + 1, url);
    
    int ret = avformat_open_input(&m_formatCtx, url.c_str(), nullptr, &opts);
    av_dict_free(&opts);
    
    if (ret < 0) {
        char errBuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errBuf, sizeof(errBuf));
        spdlog::error("Camera {} failed to open stream: {}", m_cameraIndex + 1, errBuf);
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
        spdlog::error("Camera {} failed to find stream info", m_cameraIndex + 1);
        avformat_close_input(&m_formatCtx);
        return false;
    }
    
    m_videoStreamIndex = -1;
    for (unsigned int i = 0; i < m_formatCtx->nb_streams; i++) {
        if (m_formatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            m_videoStreamIndex = i;
            break;
        }
    }
    
    if (m_videoStreamIndex == -1) {
        spdlog::error("Camera {} no video stream found", m_cameraIndex + 1);
        avformat_close_input(&m_formatCtx);
        return false;
    }
    
    const AVCodec* codec = nullptr;
    const AVCodecID cid = m_formatCtx->streams[m_videoStreamIndex]->codecpar->codec_id;
    if (m_decodeMode == DecodeMode::GPU) {
        const char* name = nullptr;
        switch (cid) {
            case AV_CODEC_ID_H264: name = "h264_cuvid"; break;
            case AV_CODEC_ID_HEVC: name = "hevc_cuvid"; break;
            case AV_CODEC_ID_MPEG2VIDEO: name = "mpeg2_cuvid"; break;
            case AV_CODEC_ID_MPEG4: name = "mpeg4_cuvid"; break;
            case AV_CODEC_ID_VP8: name = "vp8_cuvid"; break;
            case AV_CODEC_ID_VP9: name = "vp9_cuvid"; break;
            case AV_CODEC_ID_AV1: name = "av1_cuvid"; break;
            default: break;
        }
        if (name) {
            codec = avcodec_find_decoder_by_name(name);
            if (!codec) {
                spdlog::warn("Camera {} GPU decoder {} not found, falling back to CPU", m_cameraIndex + 1, name);
            }
        }
    }
    if (!codec) {
        codec = avcodec_find_decoder(cid);
    }
    if (!codec) {
        spdlog::error("Camera {} unsupported codec", m_cameraIndex + 1);
        avformat_close_input(&m_formatCtx);
        return false;
    }
    
    m_codecCtx = avcodec_alloc_context3(codec);
    if (!m_codecCtx) {
        spdlog::error("Camera {} failed to allocate codec context", m_cameraIndex + 1);
        avformat_close_input(&m_formatCtx);
        return false;
    }
    
    ret = avcodec_parameters_to_context(m_codecCtx, 
        m_formatCtx->streams[m_videoStreamIndex]->codecpar);
    if (ret < 0) {
        spdlog::error("Camera {} failed to copy codec parameters", m_cameraIndex + 1);
        avcodec_free_context(&m_codecCtx);
        avformat_close_input(&m_formatCtx);
        return false;
    }
    
    if (m_decodeMode == DecodeMode::GPU) {
        if (av_hwdevice_ctx_create(&m_hwDeviceCtx, AV_HWDEVICE_TYPE_CUDA, nullptr, nullptr, 0) < 0) {
            spdlog::warn("Camera {} failed to create CUDA hwdevice, falling back to CPU", m_cameraIndex + 1);
            m_decodeMode = DecodeMode::CPU;
        } else {
            m_codecCtx->get_format = get_hw_format;
            m_codecCtx->hw_device_ctx = av_buffer_ref(m_hwDeviceCtx);
        }
    }

    m_codecCtx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    m_codecCtx->flags |= AV_CODEC_FLAG_OUTPUT_CORRUPT;
    m_codecCtx->flags2 |= AV_CODEC_FLAG2_FAST;
    m_codecCtx->flags2 |= AV_CODEC_FLAG2_CHUNKS;
    m_codecCtx->thread_count = 1;
    m_codecCtx->thread_type = 0;
    m_codecCtx->delay = 0;
    m_codecCtx->has_b_frames = 0;
    m_codecCtx->skip_loop_filter = AVDISCARD_ALL;
    m_codecCtx->skip_idct = AVDISCARD_NONKEY;
    m_codecCtx->skip_frame = AVDISCARD_DEFAULT;
    m_codecCtx->err_recognition = 0;
    m_codecCtx->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
    
    AVDictionary* codecOpts = nullptr;
    av_dict_set(&codecOpts, "threads", "1", 0);
    ret = avcodec_open2(m_codecCtx, codec, &codecOpts);
    av_dict_free(&codecOpts);
    if (ret < 0) {
        spdlog::error("Camera {} failed to open codec", m_cameraIndex + 1);
        avcodec_free_context(&m_codecCtx);
        avformat_close_input(&m_formatCtx);
        if (m_hwDeviceCtx) {
            av_buffer_unref(&m_hwDeviceCtx);
            m_hwDeviceCtx = nullptr;
        }
        return false;
    }
    
    m_frame = av_frame_alloc();
    m_frameRGB = av_frame_alloc();
    m_packet = av_packet_alloc();
    
    if (!m_frame || !m_frameRGB || !m_packet) {
        spdlog::error("Camera {} failed to allocate frame/packet", m_cameraIndex + 1);
        cleanupDecoder();
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.frame_width = m_codecCtx->width;
        m_stats.frame_height = m_codecCtx->height;
    }
    
    spdlog::info("Camera {} decoder initialized: {}x{} @ {}", 
                 m_cameraIndex + 1, m_codecCtx->width, m_codecCtx->height,
                 avcodec_get_name(codec->id));
    
    return true;
}

void CameraStream::cleanupDecoder() {
    if (m_swsCtx) {
        sws_freeContext(m_swsCtx);
        m_swsCtx = nullptr;
    }

    if (m_hwDeviceCtx) {
        av_buffer_unref(&m_hwDeviceCtx);
        m_hwDeviceCtx = nullptr;
    }
    
    if (m_packet) {
        av_packet_free(&m_packet);
        m_packet = nullptr;
    }
    
    if (m_frameRGB) {
        if (m_frameRGB->data[0]) {
            av_freep(&m_frameRGB->data[0]);
        }
        av_frame_free(&m_frameRGB);
        m_frameRGB = nullptr;
    }
    
    if (m_frame) {
        av_frame_free(&m_frame);
        m_frame = nullptr;
    }
    
    if (m_codecCtx) {
        avcodec_free_context(&m_codecCtx);
        m_codecCtx = nullptr;
    }
    
    if (m_formatCtx) {
        avformat_close_input(&m_formatCtx);
        m_formatCtx = nullptr;
    }
    
    m_videoStreamIndex = -1;
    m_formatLogged = false;
}

bool CameraStream::decodeFrame() {
    int ret = av_read_frame(m_formatCtx, m_packet);
    if (ret < 0) {
        if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN)) {
            return false;
        }
        char errBuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errBuf, sizeof(errBuf));
        spdlog::warn("Camera {} read frame error: {}", m_cameraIndex + 1, errBuf);
        return false;
    }
    
    if (m_packet->stream_index != m_videoStreamIndex) {
        av_packet_unref(m_packet);
        return true;
    }
    
    ret = avcodec_send_packet(m_codecCtx, m_packet);
    av_packet_unref(m_packet);
    
    if (ret < 0) {
        spdlog::warn("Camera {} send packet error", m_cameraIndex + 1);
        return ret == AVERROR(EAGAIN);
    }
    
    bool gotFrame = false;
    while (true) {
        ret = avcodec_receive_frame(m_codecCtx, m_frame);
        if (ret < 0) {
            break;
        }
        gotFrame = true;

        AVFrame* usableFrame = m_frame;
        AVFrame* swFrame = nullptr;
        if (m_decodeMode == DecodeMode::GPU && m_frame->format == AV_PIX_FMT_CUDA) {
            swFrame = av_frame_alloc();
            if (swFrame && av_hwframe_transfer_data(swFrame, m_frame, 0) == 0) {
                usableFrame = swFrame;
            } else {
                spdlog::warn("Camera {} failed to transfer hw frame, skipping", m_cameraIndex + 1);
                if (swFrame) av_frame_free(&swFrame);
                av_frame_unref(m_frame);
                continue;
            }
        }
        
        if (!updateTexture(usableFrame)) {
            std::lock_guard<std::mutex> lock(m_statsMutex);
            m_stats.dropped_frames++;
        } else {
            auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(m_statsMutex);
                m_stats.total_frames++;
                m_stats.last_frame_time = now;
                m_frameCountSinceLastUpdate++;
                
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - m_lastFpsUpdate);
                if (elapsed.count() >= 1000) {
                    m_stats.current_fps = m_frameCountSinceLastUpdate * 1000.0f / elapsed.count();
                    m_frameCountSinceLastUpdate = 0;
                    m_lastFpsUpdate = now;
                }
            }
            
            {
                std::lock_guard<std::mutex> lock(m_callbackMutex);
                if (m_frameCallback) {
                    std::lock_guard<std::mutex> texLock(m_textureMutex);
                    m_frameCallback(m_cameraIndex, m_texture, getStats());
                }
            }
        }
        
        if (swFrame) {
            av_frame_free(&swFrame);
        }
        av_frame_unref(m_frame);
    }
    
    if (!gotFrame && ret != AVERROR(EAGAIN)) {
        return false;
    }
    return true;
}

bool CameraStream::updateTexture(AVFrame* frame) {
    if (!frame || frame->width == 0 || frame->height == 0) {
        spdlog::warn("Camera {} updateTexture: invalid frame", m_cameraIndex + 1);
        return false;
    }
    
    AVPixelFormat srcFormat = static_cast<AVPixelFormat>(frame->format);
    
    if (!m_formatLogged) {
        spdlog::info("Camera {} FRAME: format={} size={}x{}", 
                     m_cameraIndex + 1, av_get_pix_fmt_name(srcFormat),
                     frame->width, frame->height);
        m_formatLogged = true;
    }
    
    const AVPixelFormat targetFormat = AV_PIX_FMT_BGRA;
    
    if (!m_swsCtx) {
        m_swsCtx = sws_getContext(
            frame->width, frame->height, srcFormat,
            frame->width, frame->height, targetFormat,
            SWS_POINT, nullptr, nullptr, nullptr);
        
        if (!m_swsCtx) {
            spdlog::error("Camera {} failed to create swscale context", m_cameraIndex + 1);
            return false;
        }
        
        int bufSize = av_image_get_buffer_size(targetFormat, frame->width, frame->height, 1);
        uint8_t* buffer = static_cast<uint8_t*>(av_malloc(bufSize));
        if (!buffer) {
            spdlog::error("Camera {} failed to allocate BGRA buffer", m_cameraIndex + 1);
            sws_freeContext(m_swsCtx);
            m_swsCtx = nullptr;
            return false;
        }
        
        av_image_fill_arrays(m_frameRGB->data, m_frameRGB->linesize, buffer,
                            targetFormat, frame->width, frame->height, 1);
        m_frameRGB->width = frame->width;
        m_frameRGB->height = frame->height;
        m_frameRGB->format = targetFormat;
    }
    
    sws_scale(m_swsCtx, 
              frame->data, frame->linesize, 0, frame->height,
              m_frameRGB->data, m_frameRGB->linesize);
    
    // Store frame data for main thread to update texture
    {
        std::lock_guard<std::mutex> lock(m_pendingFrameMutex);
        int dataSize = m_frameRGB->linesize[0] * frame->height;
        m_pendingFrameData.resize(dataSize);
        memcpy(m_pendingFrameData.data(), m_frameRGB->data[0], dataSize);
        m_pendingFrameWidth = frame->width;
        m_pendingFrameHeight = frame->height;
        m_pendingFramePitch = m_frameRGB->linesize[0];
        m_hasPendingFrame.store(true);
    }
    
    return true;
}

bool CameraStream::updateTextureFromMainThread() {
    if (!m_hasPendingFrame.load()) {
        return false;
    }
    
    std::lock_guard<std::mutex> frameLock(m_pendingFrameMutex);
    std::lock_guard<std::mutex> texLock(m_textureMutex);
    
    if (m_pendingFrameData.empty()) {
        return false;
    }
    
    if (!m_texture) {
        m_texture = SDL_CreateTexture(
            m_renderer,
            SDL_PIXELFORMAT_ARGB8888,
            SDL_TEXTUREACCESS_STREAMING,
            m_pendingFrameWidth,
            m_pendingFrameHeight
        );
        
        if (!m_texture) {
            spdlog::error("Camera {} failed to create texture: {}", 
                         m_cameraIndex + 1, SDL_GetError());
            return false;
        }
    }
    
    void* texPixels = nullptr;
    int texPitch = 0;
    if (SDL_LockTexture(m_texture, nullptr, &texPixels, &texPitch) == 0) {
        if (texPitch == m_pendingFramePitch) {
            memcpy(texPixels, m_pendingFrameData.data(), m_pendingFramePitch * m_pendingFrameHeight);
        } else {
            const uint8_t* src = m_pendingFrameData.data();
            uint8_t* dst = static_cast<uint8_t*>(texPixels);
            int rowBytes = std::min(texPitch, m_pendingFramePitch);
            for (int row = 0; row < m_pendingFrameHeight; ++row) {
                memcpy(dst, src, rowBytes);
                src += m_pendingFramePitch;
                dst += texPitch;
            }
        }
        SDL_UnlockTexture(m_texture);
    } else {
        SDL_UpdateTexture(m_texture, nullptr, m_pendingFrameData.data(), m_pendingFramePitch);
    }
    
    m_hasPendingFrame.store(false);
    return true;
}

void CameraStream::attemptReconnect() {
    m_reconnectAttempts++;
    
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = CameraState::Reconnecting;
        m_stats.reconnect_count++;
    }
    
    if (m_reconnectAttempts > MAX_RECONNECT_ATTEMPTS) {
        spdlog::error("Camera {} max reconnect attempts reached, giving up", m_cameraIndex + 1);
        {
            std::lock_guard<std::mutex> lock(m_statsMutex);
            m_stats.state = CameraState::Error;
        }
        m_stopRequested = true;
        return;
    }
    
    spdlog::warn("Camera {} reconnecting (attempt {}/{})", 
                 m_cameraIndex + 1, m_reconnectAttempts, MAX_RECONNECT_ATTEMPTS);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
}

} // namespace camera_viewer
