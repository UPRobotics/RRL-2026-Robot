#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace telemetry_ui {

// -------------------------------------------------------
// Color palette — matches camera_pkg / detections_pkg
// -------------------------------------------------------
namespace Colors {
    constexpr SDL_Color BACKGROUND = {30,  30,  35,  255};
    constexpr SDL_Color FOOTER_BG  = {40,  40,  45,  255};
    constexpr SDL_Color BORDER     = {60,  60,  65,  255};
    constexpr SDL_Color TEXT       = {220, 220, 220, 255};
    constexpr SDL_Color STAT_LABEL = {180, 180, 180, 255};
    constexpr SDL_Color STAT_GOOD  = {40,  180, 99,  255};
    constexpr SDL_Color STAT_WARN  = {241, 196, 15,  255};
    constexpr SDL_Color STAT_CRIT  = {231, 76,  60,  255};
}

class MainWindow {
public:
    MainWindow(const std::string& title, int width, int height);
    ~MainWindow();

    MainWindow(const MainWindow&)            = delete;
    MainWindow& operator=(const MainWindow&) = delete;

    bool initialize();
    bool spinOnce();
    void shutdown();

    bool isRunning() const { return m_running; }

private:
    // Event & render
    void handleEvents();
    void render();
    void renderMainArea(int winW, int winH);
    void renderFooter(int winW, int winH);

    // Telemetry background thread
    void  startTelemetry();
    void  stopTelemetry();
    void  telemetryLoop();
    float sampleCpuPercent();
    float sampleRamPercent();
    void  sampleGpuStats();

    // Drawing helper
    SDL_Color getStatColor(float pct) const;
    void drawText(int x, int y, const std::string& text, SDL_Color color, TTF_Font* fnt);

    // Window properties
    int           m_width;
    int           m_height;
    bool          m_running     = false;
    bool          m_initialized = false;

    // SDL objects
    SDL_Window*   m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;

    // Fonts — FreeMono.ttf, matching camera_pkg / detections_pkg
    TTF_Font* m_font10 = nullptr;
    TTF_Font* m_font12 = nullptr;
    TTF_Font* m_font14 = nullptr;
    TTF_Font* m_font16 = nullptr;

    // Layout constants (same as camera_pkg statsbar height)
    static constexpr int FOOTER_H    = 60;
    static constexpr int STAT_PAD    = 15;

    // Telemetry thread
    std::thread        m_telemetryThread;
    std::atomic<bool>  m_telemetryRunning{false};
    std::atomic<float> m_cpu{0.0f};
    std::atomic<float> m_ram{0.0f};
    std::atomic<float> m_gpu{0.0f};
    std::atomic<float> m_vramUsedMb{0.0f};
    std::atomic<float> m_vramTotalMb{0.0f};

    // CPU delta tracking (/proc/stat)
    uint64_t m_prevTotalJiffies  = 0;
    uint64_t m_prevActiveJiffies = 0;
    bool     m_hasPrevCpu        = false;
};

} // namespace telemetry_ui
