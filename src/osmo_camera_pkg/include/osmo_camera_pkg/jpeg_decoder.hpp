#pragma once
#include <cstdint>
#include <vector>

#if HAS_NVJPEG
#include <nvjpeg.h>
#include <cuda_runtime.h>
#endif

extern "C" {
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;
}

namespace osmo {

struct DecodedFrame {
    std::vector<uint8_t> bgra;
    int width  = 0;
    int height = 0;
};

class JpegDecoder {
public:
    JpegDecoder();
    ~JpegDecoder();

    JpegDecoder(const JpegDecoder&)            = delete;
    JpegDecoder& operator=(const JpegDecoder&) = delete;

    bool initialize();
    bool decode(const uint8_t* jpegData, int size, DecodedFrame& out);

private:
#if HAS_NVJPEG
    bool decodeNvjpeg(const uint8_t* data, int size, DecodedFrame& out);

    nvjpegHandle_t    m_nvjpegHandle{};
    nvjpegJpegState_t m_nvjpegState{};
    cudaStream_t      m_cudaStream{};
    bool              m_nvjpegReady = false;
#endif

    bool initFfmpegFallback();
    bool decodeFfmpeg(const uint8_t* data, int size, DecodedFrame& out);

    AVCodecContext* m_codecCtx  = nullptr;
    AVFrame*        m_frame     = nullptr;
    AVPacket*       m_pkt       = nullptr;
    SwsContext*     m_swsCtx    = nullptr;
    int             m_swsWidth  = 0;
    int             m_swsHeight = 0;
    bool            m_ffmpegReady = false;
};

}  // namespace osmo
