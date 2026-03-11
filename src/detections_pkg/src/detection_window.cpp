#include "detections_pkg/detection_window.h"
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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
    const SDL_Color HAZMAT_COLOR= {255, 100, 50, 255};   // Orange-red for hazmat
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

    // Initialize hazmat detector (non-fatal if models not found)
    if (m_config.enable_hazmat &&
        !m_config.hazmat_yolo_model.empty() &&
        !m_config.hazmat_resnet_model.empty()) {
        if (!m_hazmatDetector.initialize(
                m_config.hazmat_yolo_model,
                m_config.hazmat_resnet_model,
                m_config.hazmat_use_cuda)) {
            spdlog::warn("Hazmat detector failed to initialize, disabling");
            m_config.enable_hazmat = false;
        }
    } else if (m_config.enable_hazmat) {
        spdlog::warn("Hazmat model paths not configured, disabling");
        m_config.enable_hazmat = false;
    }

    spdlog::info("DetectionWindow initialized {}x{}",
                 m_config.window_width, m_config.window_height);
    return true;
}

void DetectionWindow::shutdown() {
    if (m_stream) m_stream->stop();
    m_stream.reset();
    m_magPanel.reset();

    if (m_qrCropTex) { SDL_DestroyTexture(m_qrCropTex); m_qrCropTex = nullptr; }
    if (m_hazmatCropTex) { SDL_DestroyTexture(m_hazmatCropTex); m_hazmatCropTex = nullptr; }

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
        // Apply rotation
        rotateFrame(bgr);

        m_frameW = bgr.cols;
        m_frameH = bgr.rows;
        m_lastFrame = bgr.clone();

        // Run detections (only if enabled)
        if (m_config.enable_qr) {
            m_qrResults = m_qrDetector.detect(
                bgr, m_config.qr_detect_interval);
        } else {
            m_qrResults.clear();
        }
        if (m_config.enable_motion) {
            m_motionResults = m_motionDetector.detect(
                bgr,
                m_config.motion_skip_frames,
                m_config.motion_min_area,
                m_config.motion_threshold,
                m_config.motion_accumulate_weight);
        } else {
            m_motionResults.clear();
        }

        if (m_config.enable_hazmat && m_hazmatDetector.isLoaded()) {
            m_hazmatResults = m_hazmatDetector.detect(
                bgr,
                m_config.hazmat_detect_interval,
                m_config.hazmat_conf_threshold,
                m_config.hazmat_nms_threshold);
        } else {
            m_hazmatResults.clear();
        }

        // If QR detected, save frame and create crop texture
        if (!m_qrResults.empty()) {
            saveQRFrame(bgr, m_qrResults[0]);
        }

        // If hazmat detected, save crop and create texture
        if (!m_hazmatResults.empty()) {
            saveHazmatCrop(bgr, m_hazmatResults[0]);
        }
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
                else if (e.key.keysym.sym == SDLK_r) {
                    m_config.rotation = (m_config.rotation + 90) % 360;
                    spdlog::info("Rotation set to {}°", m_config.rotation);
                    m_motionDetector.reset();
                    saveConfig();
                }
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

    // ---- Toggle bar (top of video area) ----
    static constexpr int TOGGLE_BAR_H = 32;
    SDL_Rect toggleBarRect = {0, 0, videoAreaW, TOGGLE_BAR_H};
    renderToggleBar(m_renderer, toggleBarRect);

    // ---- Video area (below toggle bar) ----
    SDL_Texture* tex = m_stream->getTexture();
    int videoY = TOGGLE_BAR_H;
    int videoH = winH - TOGGLE_BAR_H;
    SDL_Rect videoRect = {0, videoY, videoAreaW, videoH};

    double renderAngle = static_cast<double>(m_config.rotation);
    bool rotated90 = (m_config.rotation == 90 || m_config.rotation == 270);

    if (tex) {
        // Fit video maintaining aspect ratio
        int texW = 0, texH = 0;
        SDL_QueryTexture(tex, nullptr, nullptr, &texW, &texH);

        // When rotated 90/270, swap effective dimensions for aspect ratio calc
        int effectiveW = rotated90 ? texH : texW;
        int effectiveH = rotated90 ? texW : texH;

        float scaleX = static_cast<float>(videoAreaW) / effectiveW;
        float scaleY = static_cast<float>(videoH) / effectiveH;
        float scale  = std::min(scaleX, scaleY);

        int dstW = static_cast<int>(effectiveW * scale);
        int dstH = static_cast<int>(effectiveH * scale);
        int dstX = (videoAreaW - dstW) / 2;
        int dstY = videoY + (videoH - dstH) / 2;

        // For SDL_RenderCopyEx with rotation, we need the dest rect to be
        // the size of the *rotated* output
        SDL_Rect dst;
        if (rotated90) {
            // Render into a rect sized for the rotated image
            dst = {dstX + (dstW - dstH) / 2, dstY + (dstH - dstW) / 2, dstH, dstW};
        } else {
            dst = {dstX, dstY, dstW, dstH};
        }
        SDL_RenderCopyEx(m_renderer, tex, nullptr, &dst, renderAngle, nullptr, SDL_FLIP_NONE);

        // The actual visible rect for overlay drawing
        SDL_Rect visibleRect = {dstX, dstY, dstW, dstH};

        // Draw detection overlays (scaled to visible rect, using rotated frame dims)
        drawDetections(m_renderer, visibleRect, m_frameW, m_frameH);

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

        // Draw QR crop overlay if available
        renderQRCrop(m_renderer, visibleRect);

        // Draw hazmat crop overlay if available
        renderHazmatCrop(m_renderer, visibleRect);
    } else {
        // No video – draw placeholder
        SDL_SetRenderDrawColor(m_renderer, WinColors::CAMERA_BG.r, WinColors::CAMERA_BG.g,
                               WinColors::CAMERA_BG.b, WinColors::CAMERA_BG.a);
        SDL_RenderFillRect(m_renderer, &videoRect);
        drawText(m_renderer, videoAreaW / 2 - 60, videoY + videoH / 2 - 8,
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

void DetectionWindow::renderToggleBar(SDL_Renderer* r, SDL_Rect area) {
    // Toolbar background (matches camera_pkg TOOLBAR_BG)
    SDL_SetRenderDrawColor(r, 40, 40, 45, 255);
    SDL_RenderFillRect(r, &area);

    // Bottom border
    SDL_SetRenderDrawColor(r, 60, 60, 65, 255);
    SDL_RenderDrawLine(r, area.x, area.y + area.h - 1, area.x + area.w, area.y + area.h - 1);

    int btnW = 130;
    int btnH = 28;
    int spacing = 12;
    int x = area.x + 12;
    int y = area.y + (area.h - btnH) / 2;

    bool allOn = m_config.enable_qr && m_config.enable_motion && m_config.enable_hazmat;

    // Colors matching camera_pkg palette
    static const SDL_Color activeColor   = {70, 130, 180, 255};  // BUTTON_NORMAL (steel blue)
    static const SDL_Color inactiveColor = {80, 80, 85, 255};    // BUTTON_DISABLED
    static const SDL_Color textColor     = {255, 255, 255, 255}; // BUTTON_TEXT (white)
    static const SDL_Color disTextColor  = {150, 150, 150, 255}; // DISABLED_TEXT
    static const SDL_Color borderColor   = {50, 50, 55, 255};    // BUTTON_BORDER

    struct ToggleDef {
        const char* label;
        bool active;
        SDL_Rect* rect;
    };
    ToggleDef toggles[] = {
        { allOn ? "ALL ON" : "ALL OFF", allOn, &m_toggleAll },
        { "QR Code",  m_config.enable_qr,     &m_toggleQR },
        { "Motion",   m_config.enable_motion,  &m_toggleMotion },
        { "Hazmat",   m_config.enable_hazmat,  &m_toggleHazmat },
    };

    for (auto& tg : toggles) {
        *tg.rect = {x, y, btnW, btnH};

        // Button fill
        auto& bg = tg.active ? activeColor : inactiveColor;
        SDL_SetRenderDrawColor(r, bg.r, bg.g, bg.b, 255);
        SDL_RenderFillRect(r, tg.rect);

        // Border
        SDL_SetRenderDrawColor(r, borderColor.r, borderColor.g, borderColor.b, 255);
        SDL_RenderDrawRect(r, tg.rect);

        // Center label text
        auto& tc = tg.active ? textColor : disTextColor;
        int tw = 0, th = 0;
        if (m_font14) TTF_SizeText(m_font14, tg.label, &tw, &th);
        int tx = x + (btnW - tw) / 2;
        int ty = y + (btnH - th) / 2;
        drawText(r, tx, ty, tg.label, tc, m_font14);

        x += btnW + spacing;
    }
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

    // Hazmat bounding boxes (orange-red, 3px thick) with class labels
    for (auto& hz : m_hazmatResults) {
        SDL_Rect box = {
            dst.x + static_cast<int>(hz.x * sx),
            dst.y + static_cast<int>(hz.y * sy),
            static_cast<int>(hz.w * sx),
            static_cast<int>(hz.h * sy)
        };

        // Draw box (orange-red)
        SDL_SetRenderDrawColor(r, WinColors::HAZMAT_COLOR.r, WinColors::HAZMAT_COLOR.g,
                               WinColors::HAZMAT_COLOR.b, WinColors::HAZMAT_COLOR.a);
        for (int t = 0; t < 3; t++) {
            SDL_Rect b = {box.x - t, box.y - t, box.w + 2*t, box.h + 2*t};
            SDL_RenderDrawRect(r, &b);
        }

        // Build label: "ResNet: Class XX% | YOLO: Class XX%"
        char labelBuf[128];
        std::snprintf(labelBuf, sizeof(labelBuf), "%s %.0f%%",
                      hz.resnet_class.empty() ? hz.yolo_class.c_str() : hz.resnet_class.c_str(),
                      (hz.resnet_class.empty() ? hz.yolo_confidence : hz.resnet_confidence) * 100.0f);

        int lx = box.x;
        int ly = box.y - 22;
        if (ly < dst.y) ly = dst.y;

        // Label background
        int tw = 0, th = 0;
        if (m_font14) TTF_SizeText(m_font14, labelBuf, &tw, &th);
        SDL_Rect labelBg = {lx - 2, ly - 2, tw + 8, th + 4};
        SDL_SetRenderDrawColor(r, WinColors::LABEL_BG.r, WinColors::LABEL_BG.g,
                               WinColors::LABEL_BG.b, WinColors::LABEL_BG.a);
        SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
        SDL_RenderFillRect(r, &labelBg);
        SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_NONE);

        drawText(r, lx, ly, labelBuf, WinColors::HAZMAT_COLOR, m_font14);
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

    // Hazmat settings separator
    py += 6;
    SDL_SetRenderDrawColor(r, WinColors::PANEL_BORDER.r, WinColors::PANEL_BORDER.g,
                           WinColors::PANEL_BORDER.b, WinColors::PANEL_BORDER.a);
    SDL_RenderDrawLine(r, px, py, px + area.w - 24, py);
    py += 8;
    drawText(r, px, py, "HAZMAT SETTINGS", WinColors::TEXT, m_font14);
    py += 28;

    std::snprintf(buf, sizeof(buf), "%d", m_config.hazmat_detect_interval);
    renderRow("Hazmat Interval", buf, m_btnHazmatInterval);

    std::snprintf(buf, sizeof(buf), "%.2f", m_config.hazmat_conf_threshold);
    renderRow("Hazmat Conf", buf, m_btnHazmatConf);

    std::snprintf(buf, sizeof(buf), "%.2f", m_config.hazmat_nms_threshold);
    renderRow("Hazmat NMS", buf, m_btnHazmatNms);

}

void DetectionWindow::handleMouseClick(int mx, int my) {
    bool changed = false;

    // Toggle bar clicks
    if (inRect(mx, my, m_toggleAll)) {
        bool allOn = m_config.enable_qr && m_config.enable_motion && m_config.enable_hazmat;
        m_config.enable_qr = !allOn;
        m_config.enable_motion = !allOn;
        m_config.enable_hazmat = !allOn && m_hazmatDetector.isLoaded();
        if (!m_config.enable_motion) m_motionDetector.reset();
        changed = true;
    } else if (inRect(mx, my, m_toggleQR)) {
        m_config.enable_qr = !m_config.enable_qr;
        changed = true;
    } else if (inRect(mx, my, m_toggleMotion)) {
        m_config.enable_motion = !m_config.enable_motion;
        if (!m_config.enable_motion) m_motionDetector.reset();
        changed = true;
    } else if (inRect(mx, my, m_toggleHazmat)) {
        if (m_hazmatDetector.isLoaded()) {
            m_config.enable_hazmat = !m_config.enable_hazmat;
            changed = true;
        }
    }

    // Min Area: step 500
    else if (inRect(mx, my, m_btnMinArea.minus)) {
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
    // Hazmat Interval: step 1
    else if (inRect(mx, my, m_btnHazmatInterval.minus)) {
        m_config.hazmat_detect_interval = std::max(1, m_config.hazmat_detect_interval - 1);
        changed = true;
    } else if (inRect(mx, my, m_btnHazmatInterval.plus)) {
        m_config.hazmat_detect_interval += 1;
        changed = true;
    }
    // Hazmat Confidence: step 0.05
    else if (inRect(mx, my, m_btnHazmatConf.minus)) {
        m_config.hazmat_conf_threshold = std::max(0.05f, m_config.hazmat_conf_threshold - 0.05f);
        changed = true;
    } else if (inRect(mx, my, m_btnHazmatConf.plus)) {
        m_config.hazmat_conf_threshold = std::min(0.95f, m_config.hazmat_conf_threshold + 0.05f);
        changed = true;
    }
    // Hazmat NMS: step 0.05
    else if (inRect(mx, my, m_btnHazmatNms.minus)) {
        m_config.hazmat_nms_threshold = std::max(0.05f, m_config.hazmat_nms_threshold - 0.05f);
        changed = true;
    } else if (inRect(mx, my, m_btnHazmatNms.plus)) {
        m_config.hazmat_nms_threshold = std::min(0.95f, m_config.hazmat_nms_threshold + 0.05f);
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

        j["camera"]["rotation"] = m_config.rotation;

        j["hazmat"]["detect_interval"]       = m_config.hazmat_detect_interval;
        j["hazmat"]["confidence_threshold"]  = m_config.hazmat_conf_threshold;
        j["hazmat"]["nms_threshold"]         = m_config.hazmat_nms_threshold;

        std::ofstream out(m_config.config_path);
        out << j.dump(2) << std::endl;
        spdlog::info("Settings saved to {}", m_config.config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to save config: {}", e.what());
    }
}

void DetectionWindow::rotateFrame(cv::Mat& frame) {
    if (frame.empty()) return;
    switch (m_config.rotation) {
        case 90:
            cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE);
            break;
        case 180:
            cv::rotate(frame, frame, cv::ROTATE_180);
            break;
        case 270:
            cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE);
            break;
        default:
            break;
    }
}

void DetectionWindow::saveQRFrame(const cv::Mat& frame, const QRDetection& qr) {
    if (frame.empty() || qr.points.size() < 4) return;

    // Compute bounding rect of QR points with padding
    int minX = frame.cols, minY = frame.rows, maxX = 0, maxY = 0;
    for (auto& p : qr.points) {
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
    }
    int pad = 20;
    minX = std::max(0, minX - pad);
    minY = std::max(0, minY - pad);
    maxX = std::min(frame.cols, maxX + pad);
    maxY = std::min(frame.rows, maxY + pad);

    if (maxX <= minX || maxY <= minY) return;

    cv::Rect roi(minX, minY, maxX - minX, maxY - minY);
    cv::Mat crop = frame(roi).clone();

    // Draw bounding box on crop (offset points)
    for (size_t i = 0; i < qr.points.size(); i++) {
        size_t j = (i + 1) % qr.points.size();
        cv::Point p1(qr.points[i].x - minX, qr.points[i].y - minY);
        cv::Point p2(qr.points[j].x - minX, qr.points[j].y - minY);
        cv::line(crop, p1, p2, cv::Scalar(255, 160, 100), 3);
    }

    // Draw data label on the crop
    cv::putText(crop, qr.data, cv::Point(4, crop.rows - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 160, 100), 1);

    // Only save to disk if this is new QR data
    if (m_seenQRData.find(qr.data) == m_seenQRData.end()) {
        m_seenQRData.insert(qr.data);

        // Save to workspace/detections/qr/
        std::string saveDir = std::string(getenv("HOME") ? getenv("HOME") : ".") + "/roboticaWS/detections/qr";
        // Create directory if needed
        std::string mkdirCmd = "mkdir -p " + saveDir;
        (void)system(mkdirCmd.c_str());

        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        char filename[128];
        std::strftime(filename, sizeof(filename), "qr_%Y%m%d_%H%M%S.png", std::localtime(&t));

        std::string savePath = saveDir + "/" + filename;
        cv::imwrite(savePath, frame);
        spdlog::info("QR frame saved: {}", savePath);
    }

    // Convert crop to BGRA for SDL texture
    cv::Mat bgra;
    cv::cvtColor(crop, bgra, cv::COLOR_BGR2BGRA);

    if (m_qrCropTex) { SDL_DestroyTexture(m_qrCropTex); m_qrCropTex = nullptr; }

    m_qrCropTex = SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_ARGB8888,
                                     SDL_TEXTUREACCESS_STREAMING,
                                     bgra.cols, bgra.rows);
    if (m_qrCropTex) {
        void* pixels; int pitch;
        if (SDL_LockTexture(m_qrCropTex, nullptr, &pixels, &pitch) == 0) {
            for (int row = 0; row < bgra.rows; row++) {
                memcpy(static_cast<uint8_t*>(pixels) + row * pitch,
                       bgra.ptr(row), bgra.cols * 4);
            }
            SDL_UnlockTexture(m_qrCropTex);
        }
        m_qrCropW = bgra.cols;
        m_qrCropH = bgra.rows;
        m_qrCropData = qr.data;
        m_qrCropTime = std::chrono::steady_clock::now();
    }
}

void DetectionWindow::renderQRCrop(SDL_Renderer* r, SDL_Rect videoArea) {
    if (!m_qrCropTex) return;

    // Show the crop for 10 seconds after last detection
    auto elapsed = std::chrono::steady_clock::now() - m_qrCropTime;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 10) {
        SDL_DestroyTexture(m_qrCropTex);
        m_qrCropTex = nullptr;
        return;
    }

    // Render in bottom-left corner of video area, max 250px wide
    int maxW = 250;
    float scale = 1.0f;
    if (m_qrCropW > maxW) scale = static_cast<float>(maxW) / m_qrCropW;
    int drawW = static_cast<int>(m_qrCropW * scale);
    int drawH = static_cast<int>(m_qrCropH * scale);

    int margin = 10;
    int dx = videoArea.x + margin;
    int dy = videoArea.y + videoArea.h - drawH - margin - 20;

    // Background
    SDL_Rect bgRect = {dx - 4, dy - 4, drawW + 8, drawH + 28};
    SDL_SetRenderDrawColor(r, 20, 20, 30, 220);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_RenderFillRect(r, &bgRect);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_NONE);

    // Border
    SDL_SetRenderDrawColor(r, 100, 160, 255, 255);
    SDL_RenderDrawRect(r, &bgRect);

    // Crop image
    SDL_Rect cropDst = {dx, dy, drawW, drawH};
    SDL_RenderCopy(r, m_qrCropTex, nullptr, &cropDst);

    // Data label below crop
    drawText(r, dx, dy + drawH + 2, "QR: " + m_qrCropData, WinColors::QR_COLOR, m_font12);
}

void DetectionWindow::saveHazmatCrop(const cv::Mat& frame, const HazmatDetection& hz) {
    if (frame.empty() || hz.w <= 0 || hz.h <= 0) return;

    // Crop region with padding
    int pad = 20;
    int x1 = std::max(0, hz.x - pad);
    int y1 = std::max(0, hz.y - pad);
    int x2 = std::min(frame.cols, hz.x + hz.w + pad);
    int y2 = std::min(frame.rows, hz.y + hz.h + pad);
    if (x2 <= x1 || y2 <= y1) return;

    cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
    cv::Mat crop = frame(roi).clone();

    // Draw bounding box on crop
    cv::Scalar color(50, 100, 255); // BGR orange-red
    cv::rectangle(crop, cv::Point(hz.x - x1, hz.y - y1),
                  cv::Point(hz.x + hz.w - x1, hz.y + hz.h - y1), color, 2);

    // Label on crop
    std::string label = hz.resnet_class.empty() ? hz.yolo_class : hz.resnet_class;
    cv::putText(crop, label, cv::Point(4, crop.rows - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

    // Build unique key from class name
    std::string key = label;

    // Save to disk if this class hasn't been saved yet
    if (m_seenHazmatClasses.find(key) == m_seenHazmatClasses.end()) {
        m_seenHazmatClasses.insert(key);

        std::string saveDir = std::string(getenv("HOME") ? getenv("HOME") : ".") + "/roboticaWS/detections/hazmat";
        std::filesystem::create_directories(saveDir);

        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        char filename[128];
        std::strftime(filename, sizeof(filename), "hazmat_%Y%m%d_%H%M%S", std::localtime(&t));

        std::string savePath = saveDir + "/" + filename + "_" + label + ".png";
        cv::imwrite(savePath, crop);
        spdlog::info("Hazmat crop saved: {}", savePath);
    }

    // Convert crop to BGRA for SDL texture
    cv::Mat bgra;
    cv::cvtColor(crop, bgra, cv::COLOR_BGR2BGRA);

    if (m_hazmatCropTex) { SDL_DestroyTexture(m_hazmatCropTex); m_hazmatCropTex = nullptr; }

    m_hazmatCropTex = SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_ARGB8888,
                                        SDL_TEXTUREACCESS_STREAMING,
                                        bgra.cols, bgra.rows);
    if (m_hazmatCropTex) {
        void* pixels; int pitch;
        if (SDL_LockTexture(m_hazmatCropTex, nullptr, &pixels, &pitch) == 0) {
            for (int row = 0; row < bgra.rows; row++) {
                memcpy(static_cast<uint8_t*>(pixels) + row * pitch,
                       bgra.ptr(row), bgra.cols * 4);
            }
            SDL_UnlockTexture(m_hazmatCropTex);
        }
        m_hazmatCropW = bgra.cols;
        m_hazmatCropH = bgra.rows;
        m_hazmatCropLabel = label;
        m_hazmatCropTime = std::chrono::steady_clock::now();
    }
}

void DetectionWindow::renderHazmatCrop(SDL_Renderer* r, SDL_Rect videoArea) {
    if (!m_hazmatCropTex) return;

    // Show the crop for 10 seconds after last detection
    auto elapsed = std::chrono::steady_clock::now() - m_hazmatCropTime;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 10) {
        SDL_DestroyTexture(m_hazmatCropTex);
        m_hazmatCropTex = nullptr;
        return;
    }

    // Render in bottom-right corner of video area, max 250px wide
    int maxW = 250;
    float scale = 1.0f;
    if (m_hazmatCropW > maxW) scale = static_cast<float>(maxW) / m_hazmatCropW;
    int drawW = static_cast<int>(m_hazmatCropW * scale);
    int drawH = static_cast<int>(m_hazmatCropH * scale);

    int margin = 10;
    int dx = videoArea.x + videoArea.w - drawW - margin;
    int dy = videoArea.y + videoArea.h - drawH - margin - 20;

    // Background
    SDL_Rect bgRect = {dx - 4, dy - 4, drawW + 8, drawH + 28};
    SDL_SetRenderDrawColor(r, 20, 20, 30, 220);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_RenderFillRect(r, &bgRect);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_NONE);

    // Border (orange-red to match hazmat color)
    SDL_SetRenderDrawColor(r, WinColors::HAZMAT_COLOR.r, WinColors::HAZMAT_COLOR.g,
                           WinColors::HAZMAT_COLOR.b, WinColors::HAZMAT_COLOR.a);
    SDL_RenderDrawRect(r, &bgRect);

    // Crop image
    SDL_Rect cropDst = {dx, dy, drawW, drawH};
    SDL_RenderCopy(r, m_hazmatCropTex, nullptr, &cropDst);

    // Label below crop
    drawText(r, dx, dy + drawH + 2, "Hazmat: " + m_hazmatCropLabel,
             WinColors::HAZMAT_COLOR, m_font12);
}

} // namespace detections
