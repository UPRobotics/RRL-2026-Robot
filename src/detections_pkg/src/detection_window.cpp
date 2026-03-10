#include "detections_pkg/detection_window.h"
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cstdio>
#include <fstream>

namespace detections {

// Colors matching camera_pkg catppuccin theme
namespace WinColors {
    const SDL_Color BG          = {30, 30, 35, 255};
    const SDL_Color CAMERA_BG   = {50, 50, 55, 255};
    const SDL_Color TEXT        = {220, 220, 220, 255};
    const SDL_Color SUBTEXT     = {166, 173, 200, 255};
    const SDL_Color GREEN       = {40, 180, 99, 255};
    const SDL_Color RED         = {231, 76, 60, 255};
    const SDL_Color YELLOW      = {241, 196, 15, 255};
    const SDL_Color QR_COLOR    = {100, 160, 255, 255};  // Bright blue for QR
    const SDL_Color MOTION_COLOR= {0, 255, 0, 255};      // Green for motion
    const SDL_Color FPS_COLOR   = {255, 255, 0, 255};
    const SDL_Color PANEL_BG    = {30, 30, 46, 255};
    const SDL_Color PANEL_BORDER= {69, 71, 90, 255};
    const SDL_Color BTN_BG      = {69, 71, 90, 255};
    const SDL_Color BTN_HOVER   = {88, 91, 112, 255};
    const SDL_Color BTN_TEXT    = {205, 214, 244, 255};
    const SDL_Color LABEL_BG    = {20, 20, 30, 200};
}

DetectionWindow::DetectionWindow(const DetectionConfig& config,
                                 rclcpp::Node::SharedPtr node)
    : m_config(config)
    , m_node(node)
{}

DetectionWindow::~DetectionWindow() {
    shutdown();
}

bool DetectionWindow::initFonts() {
    if (TTF_Init() == -1) {
        spdlog::error("TTF_Init failed: {}", TTF_GetError());
        return false;
    }
    const char* fontPath = "/usr/share/fonts/truetype/freefont/FreeMono.ttf";
    m_font12 = TTF_OpenFont(fontPath, 12);
    m_font14 = TTF_OpenFont(fontPath, 14);
    m_font16 = TTF_OpenFont(fontPath, 16);
    m_font18 = TTF_OpenFont(fontPath, 18);
    m_font24 = TTF_OpenFont(fontPath, 24);
    if (!m_font12) {
        spdlog::error("Failed to load font: {}", TTF_GetError());
        return false;
    }
    return true;
}

bool DetectionWindow::initialize() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        spdlog::error("SDL_Init failed: {}", SDL_GetError());
        return false;
    }

    if (!initFonts()) return false;

    m_window = SDL_CreateWindow(
        "Detection Monitor - ROS2",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        m_config.window_width, m_config.window_height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!m_window) {
        spdlog::error("SDL_CreateWindow failed: {}", SDL_GetError());
        return false;
    }

    m_renderer = SDL_CreateRenderer(
        m_window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!m_renderer) {
        spdlog::error("SDL_CreateRenderer failed: {}", SDL_GetError());
        return false;
    }

    // Create subsystems
    m_stream = std::make_unique<DetectionStream>(m_config, m_renderer);
    m_magPanel = std::make_unique<MagnetometerPanel>(m_node);

    // Start the camera stream
    if (!m_stream->start()) {
        spdlog::error("Failed to start detection stream");
        return false;
    }

    spdlog::info("DetectionWindow initialized {}x{}",
                 m_config.window_width, m_config.window_height);
    return true;
}

void DetectionWindow::shutdown() {
    if (m_stream) m_stream->stop();
    m_stream.reset();
    m_magPanel.reset();

    if (m_font12) TTF_CloseFont(m_font12);
    if (m_font14) TTF_CloseFont(m_font14);
    if (m_font16) TTF_CloseFont(m_font16);
    if (m_font18) TTF_CloseFont(m_font18);
    if (m_font24) TTF_CloseFont(m_font24);
    m_font12 = m_font14 = m_font16 = m_font18 = m_font24 = nullptr;
    TTF_Quit();

    if (m_renderer) { SDL_DestroyRenderer(m_renderer); m_renderer = nullptr; }
    if (m_window)   { SDL_DestroyWindow(m_window);     m_window   = nullptr; }
    SDL_Quit();
}

bool DetectionWindow::spinOnce() {
    processEvents();
    if (m_quit) return false;

    // Upload pending frame to texture
    m_stream->updateTextureFromMainThread();

    // Grab latest BGR for detections
    cv::Mat bgr;
    if (m_stream->getLatestBGRFrame(bgr)) {
        m_frameW = bgr.cols;
        m_frameH = bgr.rows;

        // Run detections
        m_qrResults = m_qrDetector.detect(
            bgr, m_config.qr_detect_interval);
        m_motionResults = m_motionDetector.detect(
            bgr,
            m_config.motion_skip_frames,
            m_config.motion_min_area,
            m_config.motion_threshold,
            m_config.motion_accumulate_weight);
    }

    render();

    // Small delay to cap CPU
    SDL_Delay(1);
    return true;
}

void DetectionWindow::processEvents() {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        switch (e.type) {
            case SDL_QUIT:
                m_quit = true;
                break;
            case SDL_KEYDOWN:
                if (e.key.keysym.sym == SDLK_q || e.key.keysym.sym == SDLK_ESCAPE)
                    m_quit = true;
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (e.button.button == SDL_BUTTON_LEFT)
                    handleMouseClick(e.button.x, e.button.y);
                break;
        }
    }
}

void DetectionWindow::drawText(SDL_Renderer* r, int x, int y,
                               const std::string& text, SDL_Color color,
                               TTF_Font* font) {
    if (!font) font = m_font12;
    if (!font || text.empty()) return;
    SDL_Surface* surf = TTF_RenderText_Blended(font, text.c_str(), color);
    if (!surf) return;
    SDL_Texture* tex = SDL_CreateTextureFromSurface(r, surf);
    if (tex) {
        SDL_Rect dst = {x, y, surf->w, surf->h};
        SDL_RenderCopy(r, tex, nullptr, &dst);
        SDL_DestroyTexture(tex);
    }
    SDL_FreeSurface(surf);
}

void DetectionWindow::render() {
    int winW, winH;
    SDL_GetWindowSize(m_window, &winW, &winH);

    // Clear background
    SDL_SetRenderDrawColor(m_renderer, WinColors::BG.r, WinColors::BG.g,
                           WinColors::BG.b, WinColors::BG.a);
    SDL_RenderClear(m_renderer);

    int panelW = m_config.mag_panel_width;
    int videoAreaW = winW - panelW;

    // Split the right panel: top half magnetometer, bottom half settings
    int magH = winH / 2;
    int settingsH = winH - magH;

    // ---- Video area ----
    SDL_Texture* tex = m_stream->getTexture();
    SDL_Rect videoRect = {0, 0, videoAreaW, winH};

    if (tex) {
        // Fit video maintaining aspect ratio
        int texW = 0, texH = 0;
        SDL_QueryTexture(tex, nullptr, nullptr, &texW, &texH);

        float scaleX = static_cast<float>(videoAreaW) / texW;
        float scaleY = static_cast<float>(winH) / texH;
        float scale  = std::min(scaleX, scaleY);

        int dstW = static_cast<int>(texW * scale);
        int dstH = static_cast<int>(texH * scale);
        int dstX = (videoAreaW - dstW) / 2;
        int dstY = (winH - dstH) / 2;

        SDL_Rect dst = {dstX, dstY, dstW, dstH};
        SDL_RenderCopy(m_renderer, tex, nullptr, &dst);

        // Draw detection overlays (scaled to dst rect)
        drawDetections(m_renderer, dst, texW, texH);

        // FPS overlay
        auto stats = m_stream->getStats();
        char fpsBuf[32];
        std::snprintf(fpsBuf, sizeof(fpsBuf), "FPS: %.1f", stats.current_fps);
        drawText(m_renderer, dstX + 8, dstY + 6, fpsBuf, WinColors::FPS_COLOR, m_font14);

        // Stream state
        const char* stateStr = "";
        SDL_Color stateColor = WinColors::GREEN;
        switch (stats.state) {
            case StreamState::Connected:    stateStr = "LIVE"; break;
            case StreamState::Connecting:   stateStr = "CONNECTING..."; stateColor = WinColors::YELLOW; break;
            case StreamState::Reconnecting: stateStr = "RECONNECTING..."; stateColor = WinColors::YELLOW; break;
            case StreamState::Error:        stateStr = "ERROR"; stateColor = WinColors::RED; break;
            default:                        stateStr = "DISCONNECTED"; stateColor = WinColors::RED; break;
        }
        drawText(m_renderer, dstX + 8, dstY + 24, stateStr, stateColor, m_font12);
    } else {
        // No video – draw placeholder
        SDL_SetRenderDrawColor(m_renderer, WinColors::CAMERA_BG.r, WinColors::CAMERA_BG.g,
                               WinColors::CAMERA_BG.b, WinColors::CAMERA_BG.a);
        SDL_RenderFillRect(m_renderer, &videoRect);
        drawText(m_renderer, videoAreaW / 2 - 60, winH / 2 - 8,
                 "Connecting...", WinColors::YELLOW, m_font16);
    }

    // ---- Magnetometer panel (top-right) ----
    SDL_Rect magRect = {videoAreaW, 0, panelW, magH};
    m_magPanel->render(m_renderer, magRect);

    // ---- Settings panel (bottom-right) ----
    SDL_Rect settingsRect = {videoAreaW, magH, panelW, settingsH};
    renderSettingsPanel(m_renderer, settingsRect);

    SDL_RenderPresent(m_renderer);
}

void DetectionWindow::drawDetections(SDL_Renderer* r, SDL_Rect dst,
                                     int srcW, int srcH) {
    if (srcW <= 0 || srcH <= 0) return;

    float sx = static_cast<float>(dst.w) / srcW;
    float sy = static_cast<float>(dst.h) / srcH;

    // Motion boxes (green, 3px thick)
    SDL_SetRenderDrawColor(r, WinColors::MOTION_COLOR.r, WinColors::MOTION_COLOR.g,
                           WinColors::MOTION_COLOR.b, WinColors::MOTION_COLOR.a);
    for (auto& mb : m_motionResults) {
        SDL_Rect box = {
            dst.x + static_cast<int>(mb.x * sx),
            dst.y + static_cast<int>(mb.y * sy),
            static_cast<int>(mb.w * sx),
            static_cast<int>(mb.h * sy)
        };
        for (int t = 0; t < 3; t++) {
            SDL_Rect b = {box.x - t, box.y - t, box.w + 2*t, box.h + 2*t};
            SDL_RenderDrawRect(r, &b);
        }
        // Label with background
        int lx = box.x;
        int ly = box.y - 20;
        if (ly < dst.y) ly = dst.y;
        drawText(r, lx, ly, "Motion", WinColors::MOTION_COLOR, m_font14);
    }

    // QR bounding boxes (blue, 4px thick) with decoded data label
    for (auto& qr : m_qrResults) {
        if (qr.points.size() < 4) continue;

        SDL_SetRenderDrawColor(r, WinColors::QR_COLOR.r, WinColors::QR_COLOR.g,
                               WinColors::QR_COLOR.b, WinColors::QR_COLOR.a);
        for (size_t i = 0; i < qr.points.size(); i++) {
            size_t j = (i + 1) % qr.points.size();
            int x1 = dst.x + static_cast<int>(qr.points[i].x * sx);
            int y1 = dst.y + static_cast<int>(qr.points[i].y * sy);
            int x2 = dst.x + static_cast<int>(qr.points[j].x * sx);
            int y2 = dst.y + static_cast<int>(qr.points[j].y * sy);
            // Draw 4px thick lines
            for (int t = -1; t <= 2; t++) {
                SDL_RenderDrawLine(r, x1 + t, y1, x2 + t, y2);
                SDL_RenderDrawLine(r, x1, y1 + t, x2, y2 + t);
            }
        }

        // Decoded data label with dark background
        std::string label = "QR: " + qr.data;

        // Find top-left corner of QR
        int minX = dst.x + static_cast<int>(qr.points[0].x * sx);
        int minY = dst.y + static_cast<int>(qr.points[0].y * sy);
        for (auto& p : qr.points) {
            int px = dst.x + static_cast<int>(p.x * sx);
            int py = dst.y + static_cast<int>(p.y * sy);
            if (py < minY || (py == minY && px < minX)) { minX = px; minY = py; }
        }

        int labelY = minY - 26;
        if (labelY < dst.y) labelY = dst.y;

        // Measure text width for background
        int tw = 0, th = 0;
        if (m_font14) TTF_SizeText(m_font14, label.c_str(), &tw, &th);
        SDL_Rect labelBg = {minX - 2, labelY - 2, tw + 8, th + 4};
        SDL_SetRenderDrawColor(r, WinColors::LABEL_BG.r, WinColors::LABEL_BG.g,
                               WinColors::LABEL_BG.b, WinColors::LABEL_BG.a);
        SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
        SDL_RenderFillRect(r, &labelBg);
        SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_NONE);

        drawText(r, minX, labelY, label, WinColors::QR_COLOR, m_font14);
    }
}

// ----------------------------------------------------------------
// Settings panel
// ----------------------------------------------------------------

void DetectionWindow::renderSettingsPanel(SDL_Renderer* r, SDL_Rect area) {
    // Background
    SDL_SetRenderDrawColor(r, WinColors::PANEL_BG.r, WinColors::PANEL_BG.g,
                           WinColors::PANEL_BG.b, WinColors::PANEL_BG.a);
    SDL_RenderFillRect(r, &area);

    // Top border
    SDL_SetRenderDrawColor(r, WinColors::PANEL_BORDER.r, WinColors::PANEL_BORDER.g,
                           WinColors::PANEL_BORDER.b, WinColors::PANEL_BORDER.a);
    SDL_RenderDrawLine(r, area.x, area.y, area.x + area.w, area.y);
    // Left border
    SDL_RenderDrawLine(r, area.x, area.y, area.x, area.y + area.h);

    int px = area.x + 12;
    int py = area.y + 10;
    int btnSize = 24;
    int rowH = 36;
    int valW = area.w - 24 - btnSize * 2 - 12;

    drawText(r, px, py, "DETECTION SETTINGS", WinColors::TEXT, m_font14);
    py += 28;

    // Helper lambda to render one setting row with +/- buttons
    auto renderRow = [&](const char* name, const std::string& value,
                         SettingRow& row) {
        drawText(r, px, py, name, WinColors::SUBTEXT, m_font12);
        py += 16;

        // [-] button
        row.minus = {px, py, btnSize, btnSize};
        SDL_SetRenderDrawColor(r, WinColors::BTN_BG.r, WinColors::BTN_BG.g,
                               WinColors::BTN_BG.b, WinColors::BTN_BG.a);
        SDL_RenderFillRect(r, &row.minus);
        SDL_SetRenderDrawColor(r, WinColors::PANEL_BORDER.r, WinColors::PANEL_BORDER.g,
                               WinColors::PANEL_BORDER.b, WinColors::PANEL_BORDER.a);
        SDL_RenderDrawRect(r, &row.minus);
        drawText(r, px + 7, py + 3, "-", WinColors::BTN_TEXT, m_font16);

        // Value
        int valX = px + btnSize + 6;
        drawText(r, valX, py + 4, value, WinColors::TEXT, m_font14);

        // [+] button
        row.plus = {px + btnSize + valW, py, btnSize, btnSize};
        SDL_SetRenderDrawColor(r, WinColors::BTN_BG.r, WinColors::BTN_BG.g,
                               WinColors::BTN_BG.b, WinColors::BTN_BG.a);
        SDL_RenderFillRect(r, &row.plus);
        SDL_SetRenderDrawColor(r, WinColors::PANEL_BORDER.r, WinColors::PANEL_BORDER.g,
                               WinColors::PANEL_BORDER.b, WinColors::PANEL_BORDER.a);
        SDL_RenderDrawRect(r, &row.plus);
        drawText(r, px + btnSize + valW + 5, py + 3, "+", WinColors::BTN_TEXT, m_font16);

        py += rowH;
    };

    char buf[32];

    std::snprintf(buf, sizeof(buf), "%d", m_config.motion_min_area);
    renderRow("Min Area", buf, m_btnMinArea);

    std::snprintf(buf, sizeof(buf), "%.0f", m_config.motion_threshold);
    renderRow("Threshold", buf, m_btnThreshold);

    std::snprintf(buf, sizeof(buf), "%.2f", m_config.motion_accumulate_weight);
    renderRow("Adapt Weight", buf, m_btnWeight);

    std::snprintf(buf, sizeof(buf), "%d", m_config.motion_skip_frames);
    renderRow("Skip Frames", buf, m_btnSkipFrames);

    std::snprintf(buf, sizeof(buf), "%d", m_config.qr_detect_interval);
    renderRow("QR Interval", buf, m_btnQRInterval);
}

void DetectionWindow::handleMouseClick(int mx, int my) {
    bool changed = false;

    // Min Area: step 500
    if (inRect(mx, my, m_btnMinArea.minus)) {
        m_config.motion_min_area = std::max(0, m_config.motion_min_area - 500);
        changed = true;
    } else if (inRect(mx, my, m_btnMinArea.plus)) {
        m_config.motion_min_area += 500;
        changed = true;
    }
    // Threshold: step 5
    else if (inRect(mx, my, m_btnThreshold.minus)) {
        m_config.motion_threshold = std::max(5.0, m_config.motion_threshold - 5.0);
        changed = true;
    } else if (inRect(mx, my, m_btnThreshold.plus)) {
        m_config.motion_threshold += 5.0;
        changed = true;
    }
    // Weight: step 0.01
    else if (inRect(mx, my, m_btnWeight.minus)) {
        m_config.motion_accumulate_weight = std::max(0.01, m_config.motion_accumulate_weight - 0.01);
        changed = true;
    } else if (inRect(mx, my, m_btnWeight.plus)) {
        m_config.motion_accumulate_weight = std::min(1.0, m_config.motion_accumulate_weight + 0.01);
        changed = true;
    }
    // Skip Frames: step 1
    else if (inRect(mx, my, m_btnSkipFrames.minus)) {
        m_config.motion_skip_frames = std::max(1, m_config.motion_skip_frames - 1);
        changed = true;
    } else if (inRect(mx, my, m_btnSkipFrames.plus)) {
        m_config.motion_skip_frames += 1;
        changed = true;
    }
    // QR Interval: step 1
    else if (inRect(mx, my, m_btnQRInterval.minus)) {
        m_config.qr_detect_interval = std::max(1, m_config.qr_detect_interval - 1);
        changed = true;
    } else if (inRect(mx, my, m_btnQRInterval.plus)) {
        m_config.qr_detect_interval += 1;
        changed = true;
    }

    if (changed) saveConfig();
}

void DetectionWindow::saveConfig() {
    if (m_config.config_path.empty()) return;

    try {
        // Read existing JSON to preserve non-detection fields
        nlohmann::json j;
        {
            std::ifstream f(m_config.config_path);
            if (f.is_open()) j = nlohmann::json::parse(f);
        }

        j["detection"]["qr_detect_interval"]       = m_config.qr_detect_interval;
        j["detection"]["motion_min_area"]           = m_config.motion_min_area;
        j["detection"]["motion_skip_frames"]        = m_config.motion_skip_frames;
        j["detection"]["motion_threshold"]          = m_config.motion_threshold;
        j["detection"]["motion_accumulate_weight"]  = m_config.motion_accumulate_weight;

        std::ofstream out(m_config.config_path);
        out << j.dump(2) << std::endl;
        spdlog::info("Settings saved to {}", m_config.config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to save config: {}", e.what());
    }
}

} // namespace detections
