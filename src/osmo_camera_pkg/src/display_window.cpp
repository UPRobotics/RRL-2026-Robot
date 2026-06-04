#include "osmo_camera_pkg/display_window.hpp"
#include <spdlog/spdlog.h>
#include <cstring>

namespace osmo {

DisplayWindow::DisplayWindow(const DisplayConfig& cfg, int numCameras)
    : m_cfg(cfg), m_numCameras(numCameras)
{}

DisplayWindow::~DisplayWindow() {
    shutdown();
}

bool DisplayWindow::initialize() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        spdlog::error("DisplayWindow: SDL_Init failed: {}", SDL_GetError());
        return false;
    }

    m_window = SDL_CreateWindow(
        "OSMO Camera Monitor",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        m_cfg.window_width, m_cfg.window_height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    if (!m_window) {
        spdlog::error("DisplayWindow: SDL_CreateWindow failed: {}", SDL_GetError());
        return false;
    }

    m_renderer = SDL_CreateRenderer(
        m_window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    if (!m_renderer) {
        spdlog::warn("DisplayWindow: accelerated renderer unavailable, trying software");
        m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_SOFTWARE);
    }
    if (!m_renderer) {
        spdlog::error("DisplayWindow: SDL_CreateRenderer failed: {}", SDL_GetError());
        return false;
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");

    m_textures.resize(m_numCameras, nullptr);
    m_pending.resize(m_numCameras);
    m_decoders.resize(m_numCameras);

    for (int i = 0; i < m_numCameras; ++i) {
        m_decoders[i] = std::make_unique<JpegDecoder>();
        if (!m_decoders[i]->initialize()) {
            spdlog::error("DisplayWindow: JpegDecoder[{}] init failed", i);
            return false;
        }
    }

    spdlog::info("DisplayWindow: initialized {}x{} for {} cameras",
        m_cfg.window_width, m_cfg.window_height, m_numCameras);
    return true;
}

void DisplayWindow::shutdown() {
    for (auto* tex : m_textures) {
        if (tex) SDL_DestroyTexture(tex);
    }
    m_textures.clear();

    if (m_renderer) { SDL_DestroyRenderer(m_renderer); m_renderer = nullptr; }
    if (m_window)   { SDL_DestroyWindow(m_window);     m_window   = nullptr; }

    SDL_Quit();
}

void DisplayWindow::submitJpegFrame(int cameraIdx, const uint8_t* data, int size) {
    if (cameraIdx < 0 || cameraIdx >= m_numCameras) return;

    DecodedFrame df;
    if (!m_decoders[cameraIdx]->decode(data, size, df)) return;

    auto& p    = m_pending[cameraIdx];
    p.bgra     = std::move(df.bgra);
    p.width    = df.width;
    p.height   = df.height;
    p.ready    = true;
}

void DisplayWindow::uploadPendingFrames() {
    for (int i = 0; i < m_numCameras; ++i) {
        auto& p = m_pending[i];
        if (!p.ready || p.width <= 0 || p.height <= 0) continue;

        if (!m_textures[i]) {
            m_textures[i] = SDL_CreateTexture(
                m_renderer,
                SDL_PIXELFORMAT_BGRA32,
                SDL_TEXTUREACCESS_STREAMING,
                p.width, p.height);
        }

        if (m_textures[i]) {
            void* pixels = nullptr;
            int   pitch  = 0;
            if (SDL_LockTexture(m_textures[i], nullptr, &pixels, &pitch) == 0) {
                const uint8_t* src = p.bgra.data();
                auto* dst = static_cast<uint8_t*>(pixels);
                const int rowBytes = p.width * 4;
                for (int row = 0; row < p.height; ++row) {
                    memcpy(dst, src, rowBytes);
                    src += rowBytes;
                    dst += pitch;
                }
                SDL_UnlockTexture(m_textures[i]);
            }
        }
        p.ready = false;
    }
}

void DisplayWindow::render() {
    SDL_SetRenderDrawColor(m_renderer, 20, 20, 25, 255);
    SDL_RenderClear(m_renderer);

    int winW = m_cfg.window_width;
    int winH = m_cfg.window_height;
    SDL_GetWindowSize(m_window, &winW, &winH);

    const int slotW = (m_numCameras > 0) ? winW / m_numCameras : winW;

    for (int i = 0; i < m_numCameras; ++i) {
        SDL_Rect dst{ i * slotW, 0, slotW, winH };
        if (m_textures[i]) {
            SDL_RenderCopy(m_renderer, m_textures[i], nullptr, &dst);
        } else {
            SDL_SetRenderDrawColor(m_renderer, 40, 40, 45, 255);
            SDL_RenderFillRect(m_renderer, &dst);
        }
    }

    SDL_RenderPresent(m_renderer);
}

bool DisplayWindow::spinOnce() {
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) {
            m_quit = true;
        } else if (ev.type == SDL_KEYDOWN) {
            if (ev.key.keysym.sym == SDLK_q ||
                ev.key.keysym.sym == SDLK_ESCAPE) {
                m_quit = true;
            }
        }
    }
    if (m_quit) return false;

    uploadPendingFrames();
    render();
    return true;
}

}  // namespace osmo
