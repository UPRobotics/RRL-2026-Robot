#include "osmo_camera_pkg/jpeg_decoder.hpp"
#include <spdlog/spdlog.h>
#include <cstring>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace osmo {

JpegDecoder::JpegDecoder() = default;

JpegDecoder::~JpegDecoder() {
#if HAS_NVJPEG
    if (m_nvjpegReady) {
        nvjpegJpegStateDestroy(m_nvjpegState);
        nvjpegDestroy(m_nvjpegHandle);
        cudaStreamDestroy(m_cudaStream);
    }
#endif
    if (m_swsCtx)   { sws_freeContext(m_swsCtx); }
    if (m_frame)    { av_frame_free(&m_frame); }
    if (m_pkt)      { av_packet_free(&m_pkt); }
    if (m_codecCtx) { avcodec_free_context(&m_codecCtx); }
}

bool JpegDecoder::initialize() {
#if HAS_NVJPEG
    if (cudaStreamCreate(&m_cudaStream) != cudaSuccess) {
        spdlog::warn("JpegDecoder: cudaStreamCreate failed, using FFmpeg fallback");
        goto ffmpeg_init;
    }
    if (nvjpegCreate(NVJPEG_BACKEND_DEFAULT, nullptr, &m_nvjpegHandle) != NVJPEG_STATUS_SUCCESS) {
        spdlog::warn("JpegDecoder: nvjpegCreate failed, using FFmpeg fallback");
        cudaStreamDestroy(m_cudaStream);
        goto ffmpeg_init;
    }
    if (nvjpegJpegStateCreate(m_nvjpegHandle, &m_nvjpegState) != NVJPEG_STATUS_SUCCESS) {
        spdlog::warn("JpegDecoder: nvjpegJpegStateCreate failed, using FFmpeg fallback");
        nvjpegDestroy(m_nvjpegHandle);
        cudaStreamDestroy(m_cudaStream);
        goto ffmpeg_init;
    }
    m_nvjpegReady = true;
    spdlog::info("JpegDecoder: NVJPEG hardware decoder ready");
    return initFfmpegFallback();

ffmpeg_init:
#endif
    return initFfmpegFallback();
}

bool JpegDecoder::initFfmpegFallback() {
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (!codec) {
        spdlog::error("JpegDecoder: MJPEG codec not found");
        return false;
    }
    m_codecCtx = avcodec_alloc_context3(codec);
    m_codecCtx->flags  |= AV_CODEC_FLAG_LOW_DELAY;
    m_codecCtx->flags2 |= AV_CODEC_FLAG2_FAST;
    m_codecCtx->thread_count = 1;
    if (avcodec_open2(m_codecCtx, codec, nullptr) < 0) {
        spdlog::error("JpegDecoder: avcodec_open2 failed");
        avcodec_free_context(&m_codecCtx);
        return false;
    }
    m_frame = av_frame_alloc();
    m_pkt   = av_packet_alloc();
    m_ffmpegReady = true;
    spdlog::info("JpegDecoder: FFmpeg MJPEG software decoder ready");
    return true;
}

bool JpegDecoder::decode(const uint8_t* jpegData, int size, DecodedFrame& out) {
#if HAS_NVJPEG
    if (m_nvjpegReady) {
        if (decodeNvjpeg(jpegData, size, out)) return true;
        spdlog::warn("JpegDecoder: NVJPEG decode failed, falling back to FFmpeg");
    }
#endif
    if (m_ffmpegReady) return decodeFfmpeg(jpegData, size, out);
    return false;
}

#if HAS_NVJPEG
bool JpegDecoder::decodeNvjpeg(const uint8_t* data, int size, DecodedFrame& out) {
    int nComponents = 0;
    nvjpegChromaSubsampling_t subsampling;
    int widths[NVJPEG_MAX_COMPONENT]  = {};
    int heights[NVJPEG_MAX_COMPONENT] = {};

    if (nvjpegGetImageInfo(m_nvjpegHandle, data, static_cast<size_t>(size),
                           &nComponents, &subsampling, widths, heights)
        != NVJPEG_STATUS_SUCCESS) return false;

    const int W = widths[0];
    const int H = heights[0];
    if (W <= 0 || H <= 0) return false;

    uint8_t* d_buf = nullptr;
    const size_t bufSz = static_cast<size_t>(W) * H * 4;
    if (cudaMalloc(&d_buf, bufSz) != cudaSuccess) return false;

    nvjpegImage_t img{};
    img.channel[0] = d_buf;
    img.pitch[0]   = static_cast<unsigned int>(W * 4);

    const nvjpegStatus_t st = nvjpegDecode(
        m_nvjpegHandle, m_nvjpegState,
        data, static_cast<size_t>(size),
        NVJPEG_OUTPUT_BGRI, &img, m_cudaStream);
    cudaStreamSynchronize(m_cudaStream);

    bool ok = (st == NVJPEG_STATUS_SUCCESS);
    if (ok) {
        out.bgra.resize(bufSz);
        out.width  = W;
        out.height = H;
        cudaMemcpy(out.bgra.data(), d_buf, bufSz, cudaMemcpyDeviceToHost);
    }
    cudaFree(d_buf);
    return ok;
}
#endif

bool JpegDecoder::decodeFfmpeg(const uint8_t* data, int size, DecodedFrame& out) {
    m_pkt->data = const_cast<uint8_t*>(data);
    m_pkt->size = size;

    if (avcodec_send_packet(m_codecCtx, m_pkt) < 0) return false;
    if (avcodec_receive_frame(m_codecCtx, m_frame) < 0) return false;

    const int W = m_frame->width;
    const int H = m_frame->height;

    if (m_swsCtx == nullptr || m_swsWidth != W || m_swsHeight != H) {
        if (m_swsCtx) sws_freeContext(m_swsCtx);
        m_swsCtx = sws_getContext(
            W, H, static_cast<AVPixelFormat>(m_frame->format),
            W, H, AV_PIX_FMT_BGRA,
            SWS_BILINEAR, nullptr, nullptr, nullptr);
        m_swsWidth  = W;
        m_swsHeight = H;
    }
    if (!m_swsCtx) return false;

    const size_t bufSz = static_cast<size_t>(W) * H * 4;
    out.bgra.resize(bufSz);
    out.width  = W;
    out.height = H;

    uint8_t* dstPlanes[1]   = { out.bgra.data() };
    int      dstLinesize[1] = { W * 4 };
    sws_scale(m_swsCtx,
              m_frame->data, m_frame->linesize, 0, H,
              dstPlanes, dstLinesize);

    av_frame_unref(m_frame);
    return true;
}

}  // namespace osmo
