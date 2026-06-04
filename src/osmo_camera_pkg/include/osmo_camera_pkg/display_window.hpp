#pragma once
#include "osmo_camera_pkg/types.hpp"
#include "osmo_camera_pkg/jpeg_decoder.hpp"
#include <SDL2/SDL.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace osmo {

struct PendingFrame {
    std::vector<uint8_t> bgra;
    int  width  = 0;
    int  height = 0;
    bool ready  = false;
};

class DisplayWindow {
public:
    DisplayWindow(const DisplayConfig& cfg, int numCameras);
    ~DisplayWindow();

    DisplayWindow(const DisplayWindow&)            = delete;
    DisplayWindow& operator=(const DisplayWindow&) = delete;

    bool initialize();
    void shutdown();

    void submitJpegFrame(int cameraIdx, const uint8_t* data, int size);

    bool spinOnce();

private:
    void uploadPendingFrames();
    void render();

    DisplayConfig m_cfg;
    int           m_numCameras;

    SDL_Window*   m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;

    std::vector<SDL_Texture*>                   m_textures;
    std::vector<PendingFrame>                   m_pending;
    std::vector<std::unique_ptr<JpegDecoder>>   m_decoders;

    bool m_quit = false;
};

}  // namespace osmo
