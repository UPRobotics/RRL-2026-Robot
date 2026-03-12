#include "telemetry_ui_pkg/main_window.h"

#include <spdlog/spdlog.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace telemetry_ui {

// -------------------------------------------------------
// Construction / destruction
// -------------------------------------------------------

MainWindow::MainWindow(const std::string& /*title*/, int width, int height)
    : m_width(width), m_height(height)
{
}

MainWindow::~MainWindow()
{
    shutdown();
}

// -------------------------------------------------------
// Lifecycle
// -------------------------------------------------------

bool MainWindow::initialize()
{
    if (m_initialized) return true;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        spdlog::error("SDL_Init failed: {}", SDL_GetError());
        return false;
    }

    if (TTF_Init() == -1) {
        spdlog::error("TTF_Init failed: {}", TTF_GetError());
        SDL_Quit();
        return false;
    }

    // FreeMono.ttf — same font used by camera_pkg and detections_pkg
    const char* fontPath = "/usr/share/fonts/truetype/freefont/FreeMono.ttf";
    m_font10 = TTF_OpenFont(fontPath, 10);
    m_font12 = TTF_OpenFont(fontPath, 12);
    m_font14 = TTF_OpenFont(fontPath, 14);
    m_font16 = TTF_OpenFont(fontPath, 16);

    if (!m_font12) {
        spdlog::error("Failed to load font '{}': {}", fontPath, TTF_GetError());
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    m_window = SDL_CreateWindow(
        "Telemetry Dashboard",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        m_width, m_height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    if (!m_window) {
        spdlog::error("SDL_CreateWindow failed: {}", SDL_GetError());
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED);
    if (!m_renderer) {
        spdlog::error("SDL_CreateRenderer failed: {}", SDL_GetError());
        SDL_DestroyWindow(m_window);
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    m_initialized = true;
    m_running     = true;

    startTelemetry();
    spdlog::info("TelemetryUI initialized ({}x{})", m_width, m_height);
    return true;
}

bool MainWindow::spinOnce()
{
    if (!m_initialized) return false;
    handleEvents();
    render();
    SDL_Delay(4); // cap at ~240 FPS to minimise display lag
    return m_running;
}

void MainWindow::shutdown()
{
    if (!m_initialized) return;

    stopTelemetry();

    if (m_font10) { TTF_CloseFont(m_font10); m_font10 = nullptr; }
    if (m_font12) { TTF_CloseFont(m_font12); m_font12 = nullptr; }
    if (m_font14) { TTF_CloseFont(m_font14); m_font14 = nullptr; }
    if (m_font16) { TTF_CloseFont(m_font16); m_font16 = nullptr; }

    if (m_renderer) { SDL_DestroyRenderer(m_renderer); m_renderer = nullptr; }
    if (m_window)   { SDL_DestroyWindow(m_window);     m_window   = nullptr; }

    TTF_Quit();
    SDL_Quit();
    m_running     = false;
    m_initialized = false;
    spdlog::info("TelemetryUI shutdown complete");
}

// -------------------------------------------------------
// Event handling
// -------------------------------------------------------

void MainWindow::handleEvents()
{
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        switch (e.type) {
            case SDL_QUIT:
                m_running = false;
                break;
            case SDL_KEYDOWN:
                if (e.key.keysym.sym == SDLK_ESCAPE ||
                    e.key.keysym.sym == SDLK_q) {
                    m_running = false;
                }
                break;
            default:
                break;
        }
    }
}

// -------------------------------------------------------
// Rendering — queries window size each frame so that every
// layout calculation adjusts automatically on resize.
// -------------------------------------------------------

void MainWindow::render()
{
    int winW, winH;
    SDL_GetWindowSize(m_window, &winW, &winH);

    SDL_SetRenderDrawColor(m_renderer,
        Colors::BACKGROUND.r, Colors::BACKGROUND.g,
        Colors::BACKGROUND.b, Colors::BACKGROUND.a);
    SDL_RenderClear(m_renderer);

    renderMainArea(winW, winH);
    renderFooter(winW, winH);

    SDL_RenderPresent(m_renderer);
}

void MainWindow::renderMainArea(int winW, int winH)
{
    int y = 0;
    int h = winH - FOOTER_H;
    if (h <= 0) return;

    SDL_Rect rect = {0, y, winW, h};
    SDL_SetRenderDrawColor(m_renderer,
        Colors::BACKGROUND.r, Colors::BACKGROUND.g,
        Colors::BACKGROUND.b, Colors::BACKGROUND.a);
    SDL_RenderFillRect(m_renderer, &rect);
}

void MainWindow::renderFooter(int winW, int winH)
{
    int footerY = winH - FOOTER_H;

    // Background bar
    SDL_Rect rect = {0, footerY, winW, FOOTER_H};
    SDL_SetRenderDrawColor(m_renderer,
        Colors::FOOTER_BG.r, Colors::FOOTER_BG.g,
        Colors::FOOTER_BG.b, Colors::FOOTER_BG.a);
    SDL_RenderFillRect(m_renderer, &rect);

    // Top border line
    SDL_SetRenderDrawColor(m_renderer,
        Colors::BORDER.r, Colors::BORDER.g,
        Colors::BORDER.b, Colors::BORDER.a);
    SDL_RenderDrawLine(m_renderer, 0, footerY, winW, footerY);

    // Stat values from background thread
    const float cpu = m_cpu.load();
    const float ram = m_ram.load();
    const float gpu = m_gpu.load();
    const float vramUsed  = m_vramUsedMb.load();
    const float vramTotal = m_vramTotalMb.load();

    // Three equally-wide columns
    const int secW   = winW / 3;
    const int labelY = footerY + 8;
    const int valueY = footerY + 28;

    char buf[48];

    // ---- CPU ----
    drawText(STAT_PAD, labelY,
             "CPU Usage", Colors::STAT_LABEL, m_font12);
    std::snprintf(buf, sizeof(buf), "%.1f%%", cpu);
    drawText(STAT_PAD, valueY,
             buf, getStatColor(cpu), m_font16);

    // ---- RAM ----
    drawText(secW + STAT_PAD, labelY,
             "RAM Usage", Colors::STAT_LABEL, m_font12);
    std::snprintf(buf, sizeof(buf), "%.1f%%", ram);
    drawText(secW + STAT_PAD, valueY,
             buf, getStatColor(ram), m_font16);

    // ---- GPU ----
    const int col3 = secW * 2 + STAT_PAD;
    if (vramTotal > 0.0f) {
        drawText(col3, labelY,
                 "GPU Core / VRAM", Colors::STAT_LABEL, m_font12);
        std::snprintf(buf, sizeof(buf), "%.0f%%  %.0f/%.0f MB",
                      gpu, vramUsed, vramTotal);
    } else {
        drawText(col3, labelY,
                 "GPU Usage", Colors::STAT_LABEL, m_font12);
        std::snprintf(buf, sizeof(buf), "%.0f%%", gpu);
    }
    drawText(col3, valueY, buf, getStatColor(gpu), m_font16);
}

// -------------------------------------------------------
// Drawing helpers
// -------------------------------------------------------

SDL_Color MainWindow::getStatColor(float pct) const
{
    if (pct < 50.0f) return Colors::STAT_GOOD;
    if (pct < 80.0f) return Colors::STAT_WARN;
    return Colors::STAT_CRIT;
}

void MainWindow::drawText(int x, int y, const std::string& text,
                          SDL_Color color, TTF_Font* fnt)
{
    if (!fnt || text.empty()) return;

    SDL_Surface* surf = TTF_RenderText_Blended(fnt, text.c_str(), color);
    if (!surf) return;

    SDL_Texture* tex = SDL_CreateTextureFromSurface(m_renderer, surf);
    if (tex) {
        SDL_Rect dst = {x, y, surf->w, surf->h};
        SDL_RenderCopy(m_renderer, tex, nullptr, &dst);
        SDL_DestroyTexture(tex);
    }
    SDL_FreeSurface(surf);
}

// -------------------------------------------------------
// Telemetry background thread
// -------------------------------------------------------

void MainWindow::startTelemetry()
{
    if (m_telemetryRunning) return;
    m_telemetryRunning = true;
    m_telemetryThread  = std::thread(&MainWindow::telemetryLoop, this);
}

void MainWindow::stopTelemetry()
{
    m_telemetryRunning = false;
    if (m_telemetryThread.joinable())
        m_telemetryThread.join();
}

void MainWindow::telemetryLoop()
{
    using namespace std::chrono_literals;
    while (m_telemetryRunning) {
        m_cpu.store(sampleCpuPercent());
        m_ram.store(sampleRamPercent());
        sampleGpuStats();

        // Sleep in 100 ms chunks so the thread responds promptly to shutdown
        for (int i = 0; i < 10 && m_telemetryRunning; ++i)
            std::this_thread::sleep_for(100ms);
    }
}

// System-wide CPU usage via /proc/stat delta between two samples.
float MainWindow::sampleCpuPercent()
{
    std::ifstream f("/proc/stat");
    if (!f.is_open()) return 0.0f;

    std::string line;
    std::getline(f, line);

    std::istringstream ss(line);
    std::string label;
    ss >> label; // "cpu"

    uint64_t user = 0, nice = 0, sys = 0, idle = 0,
             iowait = 0, irq = 0, softirq = 0, steal = 0;
    ss >> user >> nice >> sys >> idle >> iowait >> irq >> softirq >> steal;

    const uint64_t total  = user + nice + sys + idle + iowait + irq + softirq + steal;
    const uint64_t active = total - idle - iowait;

    float pct = 0.0f;
    if (m_hasPrevCpu) {
        const uint64_t dt = total  - m_prevTotalJiffies;
        const uint64_t da = active - m_prevActiveJiffies;
        if (dt > 0)
            pct = static_cast<float>(da) / static_cast<float>(dt) * 100.0f;
    }

    m_prevTotalJiffies  = total;
    m_prevActiveJiffies = active;
    m_hasPrevCpu        = true;
    return pct;
}

// System-wide RAM usage via /proc/meminfo (MemAvailable method).
float MainWindow::sampleRamPercent()
{
    std::ifstream f("/proc/meminfo");
    if (!f.is_open()) return 0.0f;

    uint64_t memTotal     = 0;
    uint64_t memAvailable = 0;

    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("MemTotal:", 0) == 0)
            std::sscanf(line.c_str(), "MemTotal: %lu kB", &memTotal);
        else if (line.rfind("MemAvailable:", 0) == 0)
            std::sscanf(line.c_str(), "MemAvailable: %lu kB", &memAvailable);

        if (memTotal && memAvailable) break;
    }

    if (!memTotal) return 0.0f;
    return static_cast<float>(memTotal - memAvailable) /
           static_cast<float>(memTotal) * 100.0f;
}

// GPU core utilisation + VRAM via nvidia-smi (same approach as detections_pkg).
void MainWindow::sampleGpuStats()
{
    const char* cmd =
        "timeout 3 nvidia-smi "
        "--query-gpu=utilization.gpu,memory.used,memory.total "
        "--format=csv,noheader,nounits -i 0";

    FILE* pipe = popen(cmd, "r");
    if (!pipe) return;

    char buf[256] = {};
    if (fgets(buf, sizeof(buf), pipe)) {
        std::istringstream ss(buf);
        std::string tok;
        float vals[3] = {0.0f, 0.0f, 0.0f};

        for (int i = 0; i < 3 && std::getline(ss, tok, ','); ++i) {
            // trim whitespace
            const auto first = tok.find_first_not_of(" \t\n\r");
            const auto last  = tok.find_last_not_of(" \t\n\r");
            if (first == std::string::npos) continue;
            tok = tok.substr(first, last - first + 1);
            try { vals[i] = std::stof(tok); } catch (...) {}
        }

        m_gpu.store(vals[0]);
        m_vramUsedMb.store(vals[1]);
        m_vramTotalMb.store(vals[2]);
    }

    pclose(pipe);
}

} // namespace telemetry_ui
