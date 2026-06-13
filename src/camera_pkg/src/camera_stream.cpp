#include "camera_pkg/camera_stream.h"
#include <spdlog/spdlog.h>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/hwcontext.h>
#include <libavdevice/avdevice.h> 
}

#include <jpeglib.h>
#include <setjmp.h>

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
    m_formatCtx = nullptr;
    m_codecCtx = nullptr;
    m_hwDeviceCtx = nullptr;
    m_swsCtx = nullptr;
    m_frame = nullptr;
    m_frameRGB = nullptr;
    m_packet = nullptr;
    m_texture = nullptr;
    m_formatLogged = false;
    
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
    if (!m_running.load() || !m_texture) {
        return nullptr; // Clean fallback protection
    }
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

    int ret = 0;
    bool is_v4l2 = (url.find("/dev/video") == 0);

    if (is_v4l2) {
        // --- 1. LOCAL USB CAMERA (Lowest Latency MJPEG Pipeline) ---
        static std::once_flag device_init_flag;
                std::call_once(device_init_flag, []() {
                    avdevice_register_all(); 
                });
                const AVInputFormat* iformat = av_find_input_format("video4linux2");
        
        if (!iformat) {
            spdlog::error("video4linux2 driver not found in FFmpeg!");
            avformat_free_context(m_formatCtx);
            m_formatCtx = nullptr;
            return false;
        }

        AVDictionary* opts = nullptr;
        av_dict_set(&opts, "input_format", "mjpeg", 0);
        av_dict_set(&opts, "video_size", "1920x1080", 0);
        av_dict_set(&opts, "framerate", "30", 0);
        av_dict_set(&opts, "fflags", "nobuffer+flush_packets", 0);
        av_dict_set(&opts, "flags", "low_delay", 0);

        spdlog::info("Camera {} capturing local hardware device: {}", m_cameraIndex + 1, url);
        ret = avformat_open_input(&m_formatCtx, url.c_str(), const_cast<AVInputFormat*>(iformat), &opts);
        av_dict_free(&opts);

} else {
        // --- CÁMARA DE RED (JETSON VIA RTP/SDP) ---
        AVDictionary* opts = nullptr;
        
        // 1. Permisos: Permitir leer el archivo SDP y abrir la red
        av_dict_set(&opts, "protocol_whitelist", "file,udp,rtp", 0);
        
        // 2. Cero Latencia: Obligar a FFmpeg a imitar 'sync=false'
        av_dict_set(&opts, "fflags", "nobuffer+discardcorrupt", 0);
        av_dict_set(&opts, "flags", "low_delay", 0);
        // Increase probe sizes for RTP/MJPEG so FFmpeg can discover codec params
        // Default tiny probe values caused "unspecified size" for MJPEG RTP streams.
        av_dict_set(&opts, "analyzeduration", "10000000", 0); // 10s in microseconds
        av_dict_set(&opts, "probesize", "32M", 0); // allow large probe to collect JPEG headers
        
        spdlog::info("Camera {} connecting to Jetson via SDP: {}", m_cameraIndex + 1, url);

        // 3. Abrir stream
        ret = avformat_open_input(&m_formatCtx, url.c_str(), nullptr, &opts);

// If SDP failed to open (often due to probe/port binding issues), try a UDP fallback
if (ret < 0) {

 char errBuf[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errBuf, sizeof(errBuf));


     if (m_formatCtx) {
        avformat_close_input(&m_formatCtx);
    }

    if (m_formatCtx) {
        avformat_free_context(m_formatCtx);
        m_formatCtx = nullptr;
    }


    spdlog::error(
        "Camera {} SDP open failed: {}",
        m_cameraIndex + 1,
        errBuf
    );


    if (m_formatCtx != nullptr) {
        avformat_close_input(&m_formatCtx); 
    }

    // Only attempt fallback for local SDP files
    std::string lower = url;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    if (lower.size() >= 4 && lower.substr(lower.size()-4) == ".sdp") {
        spdlog::warn("Camera {} SDP open failed; attempting UDP fallback", m_cameraIndex + 1);
        // parse port from SDP 'm=' line if possible, default to 5000
        int fallback_port = 5000;
        // simple parse: look for 'm=video <port>' in the file
        try {
            std::ifstream sdpfile(url);
            std::string line;
            while (std::getline(sdpfile, line)) {
                if (line.rfind("m=video", 0) == 0) {
                    std::istringstream iss(line);
                    std::string m, proto; int p; int pt;
                    if (iss >> m >> p >> proto >> pt) {
                        fallback_port = p;
                    }
                    break;
                }
            }
        } catch (...) {}

        av_dict_free(&opts);
        std::string udp_url = std::string("udp://0.0.0.0:") + std::to_string(fallback_port);
        spdlog::info("Camera {} attempting UDP open: {}", m_cameraIndex + 1, udp_url);
        
        // prepare opts for UDP open
        AVDictionary* udp_opts = nullptr;

av_dict_set(&udp_opts, "protocol_whitelist", "file,udp,rtp", 0);

av_dict_set(&udp_opts, "buffer_size", "8388608", 0);

av_dict_set(&udp_opts, "fflags", "discardcorrupt", 0);

av_dict_set(&udp_opts, "flags", "low_delay", 0);

av_dict_set(&udp_opts, "analyzeduration", "100000", 0);

        av_dict_set(&udp_opts, "probesize", "32M", 0);
        
        // 2. ATTEMPT UDP FALLBACK WITH CLEAN CONTEXT
        ret = avformat_open_input(&m_formatCtx, udp_url.c_str(), nullptr, &udp_opts);
        
        av_dict_free(&udp_opts);
        if (ret >= 0) {
            spdlog::info("Camera {} UDP fallback succeeded", m_cameraIndex + 1);
        }
    } else {
        av_dict_free(&opts);
    }
} else {
    av_dict_free(&opts);
}
}

    if (ret < 0) {
        char errBuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errBuf, sizeof(errBuf));
        spdlog::error("Camera {} failed to open stream: {}", m_cameraIndex + 1, errBuf);
        avformat_free_context(m_formatCtx);
        m_formatCtx = nullptr;
        return false;
    }
    
    m_formatCtx->max_analyze_duration = 0;
    m_formatCtx->fps_probe_size = 0;
    
    // For local v4l2 devices we avoid calling avformat_find_stream_info
    // because it may probe/dispatch into codec paths that can crash on
    // certain system-specific MJPEG v4l2 drivers. Use the existing
    // stream list / codec parameters directly for v4l2 inputs.
    if (!is_v4l2) {
        ret = avformat_find_stream_info(m_formatCtx, nullptr);
        if (ret < 0) {
            spdlog::error("Camera {} failed to find stream info", m_cameraIndex + 1);
            avformat_close_input(&m_formatCtx);
            m_formatCtx = nullptr;
            return false;
        }
    } else {
        spdlog::debug("Camera {} v4l2 input - skipping avformat_find_stream_info probe", m_cameraIndex + 1);
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
        m_formatCtx = nullptr;
        return false;
    }
    
    const AVCodecID cid = m_formatCtx->streams[m_videoStreamIndex]->codecpar->codec_id;
    const AVCodec* codec = nullptr;

    // --- CRITICAL HARDWARE VALIDATION FIX ---
    // Check if CUDA is physically viable BEFORE choosing a hardware codec string
    if (m_decodeMode == DecodeMode::GPU && !is_v4l2) {
        AVBufferRef* test_hw_ctx = nullptr;
        if (av_hwdevice_ctx_create(&test_hw_ctx, AV_HWDEVICE_TYPE_CUDA, nullptr, nullptr, 0) < 0) {
            spdlog::warn("Camera {} CUDA hardware unavailable (No NVIDIA GPU detected). Falling back to CPU.", m_cameraIndex + 1);
            m_decodeMode = DecodeMode::CPU;
        } else {
            // NVIDIA Hardware exists! safe to match cuvid wrappers
            const char* name = nullptr;
            switch (cid) {
                case AV_CODEC_ID_H264: name = "h264_cuvid"; break;
                case AV_CODEC_ID_HEVC: name = "hevc_cuvid"; break;
                case AV_CODEC_ID_MJPEG: name = "mjpeg_cuvid"; break;
                default: break;
            }
            if (name) {
                codec = avcodec_find_decoder_by_name(name);
                if (codec) {
                    m_hwDeviceCtx = test_hw_ctx; // Bind verified hardware context reference
                } else {
                    spdlog::warn("Camera {} GPU codec {} not found in FFmpeg build. Falling back to CPU.", m_cameraIndex + 1, name);
                    av_buffer_unref(&test_hw_ctx);
                    m_decodeMode = DecodeMode::CPU;
                }
            } else {
                av_buffer_unref(&test_hw_ctx);
                m_decodeMode = DecodeMode::CPU;
            }
        }
    } else {
        m_decodeMode = DecodeMode::CPU;
    }

    // Fall back to native software CPU codec if no hardware path was activated
    if (!codec) {
        codec = avcodec_find_decoder(cid);
    }

    if (!codec) {
        spdlog::error("Camera {} unsupported codec type", m_cameraIndex + 1);
        if (m_hwDeviceCtx) { av_buffer_unref(&m_hwDeviceCtx); m_hwDeviceCtx = nullptr; }
        avformat_close_input(&m_formatCtx);
        m_formatCtx = nullptr;
        return false;
    }
    
    m_codecCtx = avcodec_alloc_context3(codec);
    if (!m_codecCtx) {
        spdlog::error("Camera {} failed to allocate codec context", m_cameraIndex + 1);
        if (m_hwDeviceCtx) { av_buffer_unref(&m_hwDeviceCtx); m_hwDeviceCtx = nullptr; }
        avformat_close_input(&m_formatCtx);
        m_formatCtx = nullptr;
        return false;
    }
    
    ret = avcodec_parameters_to_context(m_codecCtx, m_formatCtx->streams[m_videoStreamIndex]->codecpar);
    if (ret < 0) {
        spdlog::error("Camera {} failed to copy codec parameters", m_cameraIndex + 1);
        avcodec_free_context(&m_codecCtx);
        m_codecCtx = nullptr;
        if (m_hwDeviceCtx) { av_buffer_unref(&m_hwDeviceCtx); m_hwDeviceCtx = nullptr; }
        avformat_close_input(&m_formatCtx);
        m_formatCtx = nullptr;
        return false;
    }
    
    // Bind hardware setup context ONLY if running a verified GPU pipeline
    if (m_decodeMode == DecodeMode::GPU && m_hwDeviceCtx) {
        m_codecCtx->get_format = get_hw_format;
        m_codecCtx->hw_device_ctx = av_buffer_ref(m_hwDeviceCtx);
    } else {
        m_codecCtx->get_format = nullptr; // Ensure standard software lookup handles formatting
    }

    m_codecCtx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    m_codecCtx->flags |= AV_CODEC_FLAG_OUTPUT_CORRUPT;
    m_codecCtx->flags2 |= AV_CODEC_FLAG2_FAST;
    m_codecCtx->thread_count = 1; 
    m_codecCtx->delay = 0;
    
    ret = avcodec_open2(m_codecCtx, codec, nullptr);
    if (ret < 0) {
        spdlog::error("Camera {} failed to open codec structure", m_cameraIndex + 1);
        avcodec_free_context(&m_codecCtx);
        m_codecCtx = nullptr;
        if (m_hwDeviceCtx) { av_buffer_unref(&m_hwDeviceCtx); m_hwDeviceCtx = nullptr; }
        avformat_close_input(&m_formatCtx);
        m_formatCtx = nullptr;
        return false;
    }
    
    m_frame = av_frame_alloc();
    m_frameRGB = av_frame_alloc();
    m_packet = av_packet_alloc();

    // Validate core allocations to avoid crashes in decode loop
    if (!m_frame || !m_frameRGB || !m_packet) {
        spdlog::error("Camera {} failed to allocate frame/packet structures", m_cameraIndex + 1);
        cleanupDecoder();
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.frame_width = m_codecCtx->width;
        m_stats.frame_height = m_codecCtx->height;
    }
    
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
    if (!m_formatCtx || !m_packet) {
        spdlog::warn("Camera {} decodeFrame: invalid format context or packet", m_cameraIndex + 1);
        return false;
    }

    int ret = av_read_frame(m_formatCtx, m_packet);
    if (ret < 0) {
        if (ret == AVERROR_EOF) {
            // Stream ended - trigger reconnect
            return false;
        }
        if (ret == AVERROR(EAGAIN)) {
            // Transient no-data condition (network jitter). Wait briefly
            // and continue rather than tearing down the decoder which
            // causes 1s reconnect gaps.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            return true;
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
    
    if (!m_codecCtx) {
        av_packet_unref(m_packet);
        spdlog::warn("Camera {} decodeFrame: codec context is null", m_cameraIndex + 1);
        return false;
    }

    // Defensive logging: validate packet before sending to codec (helps debug crashes in libavcodec)
    if (!m_packet->data || m_packet->size == 0) {
        spdlog::warn("Camera {} decodeFrame: empty packet (size={}) - skipping", m_cameraIndex + 1, m_packet->size);
        av_packet_unref(m_packet);
        // Skip empty packet but keep decoder alive
        return true;
    }

    if (m_codecCtx && m_codecCtx->codec) {
        spdlog::debug("Camera {} send_packet: codec={} pkt_size={} stream_index={}",
                      m_cameraIndex + 1,
                      m_codecCtx->codec->name ? m_codecCtx->codec->name : "<unknown>",
                      m_packet->size, m_packet->stream_index);
    }
    // If this is a local v4l2 MJPEG stream and FFmpeg send_packet is unstable
    // on this system, use libjpeg fallback to decode MJPEG frames directly
    // If the incoming stream codec is MJPEG, prefer our libjpeg fallback.
    // Some system FFmpeg/libavcodec MJPEG decoders can crash on certain
    // RTP/SDP/v4l2 inputs — use the stable libjpeg path for MJPEG packets.
    if (m_formatCtx && m_formatCtx->streams[m_videoStreamIndex]->codecpar->codec_id == AV_CODEC_ID_MJPEG) {
        spdlog::debug("Camera {} using libjpeg MJPEG fallback", m_cameraIndex + 1);
        bool ok = decodeMJPEGPacketWithLibjpeg();
        av_packet_unref(m_packet);
        return ok;
    }

    ret = avcodec_send_packet(m_codecCtx, m_packet);

    if (!m_codecCtx) {
    spdlog::error("m_codecCtx is NULL");
    return false;
}

if (!m_packet) {
    spdlog::error("m_packet is NULL");
    return false;
}

if (!m_frame) {
    spdlog::error("m_frame is NULL");
    return false;
}

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

bool CameraStream::decodeMJPEGPacketWithLibjpeg() {
    if (!m_packet || !m_packet->data || m_packet->size == 0) {
        return false;
    }

    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, m_packet->data, m_packet->size);
    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK && jpeg_read_header(&cinfo, TRUE) <= 0) {
        // Try to proceed anyway; if header fails, abort
        jpeg_destroy_decompress(&cinfo);
        return false;
    }
    cinfo.out_color_space = JCS_RGB;
    if (!jpeg_start_decompress(&cinfo)) {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    const int width = cinfo.output_width;
    const int height = cinfo.output_height;
    const int comps = cinfo.output_components; // expect 3
    if (width <= 0 || height <= 0 || comps < 3) {
        jpeg_finish_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    std::vector<uint8_t> rowbuf(width * comps);
    std::vector<uint8_t> outbuf(static_cast<size_t>(width) * height * 4);

    int y = 0;
    while (cinfo.output_scanline < cinfo.output_height) {
        JSAMPROW rowptr[1];
        rowptr[0] = rowbuf.data();
        int read = jpeg_read_scanlines(&cinfo, rowptr, 1);
        if (read <= 0) break;

        uint8_t* dst = outbuf.data() + (size_t)y * width * 4;
        for (int x = 0; x < width; ++x) {
            const uint8_t r = rowbuf[x * comps + 0];
            const uint8_t g = rowbuf[x * comps + 1];
            const uint8_t b = rowbuf[x * comps + 2];
            dst[x * 4 + 0] = b;
            dst[x * 4 + 1] = g;
            dst[x * 4 + 2] = r;
            dst[x * 4 + 3] = 0xFF;
        }
        ++y;
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    // Push decoded BGRA buffer to pending frame for main thread texture update
    {
        std::lock_guard<std::mutex> lock(m_pendingFrameMutex);
        m_pendingFrameData.swap(outbuf);
        m_pendingFrameWidth = width;
        m_pendingFrameHeight = height;
        m_pendingFramePitch = width * 4;
        m_hasPendingFrame.store(true);
    }

    return true;
}

bool CameraStream::updateTextureFromMainThread() {
    if (!m_running.load() || !m_hasPendingFrame.load()) {
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