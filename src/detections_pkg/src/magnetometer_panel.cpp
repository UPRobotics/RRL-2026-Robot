#include "detections_pkg/magnetometer_panel.h"
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace detections {

// Catppuccin-inspired dark palette
namespace PanelColors {
    const SDL_Color BG           = {30, 30, 46, 255};
    const SDL_Color BORDER       = {69, 71, 90, 255};
    const SDL_Color TEXT         = {205, 214, 244, 255};
    const SDL_Color SUBTEXT      = {166, 173, 200, 255};
    const SDL_Color GREEN        = {166, 227, 161, 255};
    const SDL_Color YELLOW       = {249, 226, 175, 255};
    const SDL_Color RED          = {243, 139, 168, 255};
    const SDL_Color BLUE         = {137, 180, 250, 255};
    const SDL_Color GRAPH_LINE   = {137, 180, 250, 255};
    const SDL_Color GRAPH_BG     = {24, 24, 37, 255};
    const SDL_Color GRAPH_GRID   = {49, 50, 68, 255};
    const SDL_Color BAR_BG       = {49, 50, 68, 255};
}

MagnetometerPanel::MagnetometerPanel(rclcpp::Node::SharedPtr node) {
    m_sub = node->create_subscription<std_msgs::msg::String>(
        "magnetometer_data", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { onMagData(msg); });
    m_lastReceived = std::chrono::steady_clock::now();
}

void MagnetometerPanel::onMagData(const std_msgs::msg::String::SharedPtr msg) {
    try {
        auto j = nlohmann::json::parse(msg->data);
        MagReading r;
        r.x = j.value("x", 0.0);
        r.y = j.value("y", 0.0);
        r.z = j.value("z", 0.0);
        r.magnitude = j.value("magnitude", 0.0);
        r.unit = j.value("unit", "uT");
        r.timestamp = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(m_dataMutex);
        m_latest = r;
        m_hasData = true;
        m_lastReceived = r.timestamp;
        m_history.push_back(r);
        if (m_history.size() > MAX_HISTORY) m_history.pop_front();
    } catch (const std::exception& e) {
        spdlog::warn("Magnetometer JSON parse error: {}", e.what());
    }
}

// Helper to render text via TTF (uses the window's font ptrs passed through renderer user data)
static void renderText(SDL_Renderer* renderer, TTF_Font* font,
                       int x, int y, const std::string& text, SDL_Color color) {
    if (!font || text.empty()) return;
    SDL_Surface* surf = TTF_RenderText_Blended(font, text.c_str(), color);
    if (!surf) return;
    SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
    if (tex) {
        SDL_Rect dst = {x, y, surf->w, surf->h};
        SDL_RenderCopy(renderer, tex, nullptr, &dst);
        SDL_DestroyTexture(tex);
    }
    SDL_FreeSurface(surf);
}

void MagnetometerPanel::render(SDL_Renderer* renderer, SDL_Rect area) {
    // Background
    SDL_SetRenderDrawColor(renderer, PanelColors::BG.r, PanelColors::BG.g,
                           PanelColors::BG.b, PanelColors::BG.a);
    SDL_RenderFillRect(renderer, &area);

    // Border left edge
    SDL_SetRenderDrawColor(renderer, PanelColors::BORDER.r, PanelColors::BORDER.g,
                           PanelColors::BORDER.b, PanelColors::BORDER.a);
    SDL_RenderDrawLine(renderer, area.x, area.y, area.x, area.y + area.h);

    // Load fonts via TTF directly (open once, cache)
    static TTF_Font* fontLarge  = nullptr;
    static TTF_Font* fontMedium = nullptr;
    static TTF_Font* fontSmall  = nullptr;
    if (!fontLarge) {
        const char* path = "/usr/share/fonts/truetype/freefont/FreeMono.ttf";
        fontLarge  = TTF_OpenFont(path, 28);
        fontMedium = TTF_OpenFont(path, 16);
        fontSmall  = TTF_OpenFont(path, 12);
    }

    int px = area.x + 12;
    int py = area.y + 10;

    // Title
    renderText(renderer, fontMedium, px, py, "MAGNETOMETER", PanelColors::TEXT);
    py += 28;

    std::lock_guard<std::mutex> lock(m_dataMutex);

    // Status indicator
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - m_lastReceived).count();
    SDL_Color statusColor = (m_hasData && elapsed < 3) ? PanelColors::GREEN : PanelColors::RED;
    SDL_Rect dot = {px, py + 2, 10, 10};
    SDL_SetRenderDrawColor(renderer, statusColor.r, statusColor.g,
                           statusColor.b, statusColor.a);
    SDL_RenderFillRect(renderer, &dot);
    const char* statusText = (m_hasData && elapsed < 3) ? "Receiving" : "No Data";
    renderText(renderer, fontSmall, px + 16, py, statusText, PanelColors::SUBTEXT);
    py += 24;

    if (!m_hasData) return;

    // Magnitude display
    char magBuf[32];
    std::snprintf(magBuf, sizeof(magBuf), "%.2f", m_latest.magnitude);
    SDL_Color magColor;
    if (m_latest.magnitude < 25.0) magColor = PanelColors::GREEN;
    else if (m_latest.magnitude < 55.0) magColor = PanelColors::YELLOW;
    else magColor = PanelColors::RED;

    renderText(renderer, fontLarge, px, py, magBuf, magColor);
    py += 36;
    renderText(renderer, fontSmall, px, py, m_latest.unit, PanelColors::SUBTEXT);
    py += 20;

    // Intensity bar
    int barW = area.w - 24;
    int barH = 14;
    SDL_Rect barBg = {px, py, barW, barH};
    SDL_SetRenderDrawColor(renderer, PanelColors::BAR_BG.r, PanelColors::BAR_BG.g,
                           PanelColors::BAR_BG.b, PanelColors::BAR_BG.a);
    SDL_RenderFillRect(renderer, &barBg);

    double maxMag = 100.0;
    int fillW = std::min(barW, static_cast<int>(barW * (m_latest.magnitude / maxMag)));
    SDL_Rect barFill = {px, py, fillW, barH};
    SDL_SetRenderDrawColor(renderer, magColor.r, magColor.g, magColor.b, magColor.a);
    SDL_RenderFillRect(renderer, &barFill);
    py += barH + 14;

    // X / Y / Z
    char xyzBuf[64];
    std::snprintf(xyzBuf, sizeof(xyzBuf), "X: %.1f", m_latest.x);
    renderText(renderer, fontSmall, px, py, xyzBuf, PanelColors::SUBTEXT);
    py += 16;
    std::snprintf(xyzBuf, sizeof(xyzBuf), "Y: %.1f", m_latest.y);
    renderText(renderer, fontSmall, px, py, xyzBuf, PanelColors::SUBTEXT);
    py += 16;
    std::snprintf(xyzBuf, sizeof(xyzBuf), "Z: %.1f", m_latest.z);
    renderText(renderer, fontSmall, px, py, xyzBuf, PanelColors::SUBTEXT);
    py += 26;

    // ---- Graph ----
    renderText(renderer, fontSmall, px, py, "History (60s)", PanelColors::SUBTEXT);
    py += 18;

    int graphX = px;
    int graphW = area.w - 24;
    int graphH = area.y + area.h - py - 10;
    if (graphH < 40) return;

    SDL_Rect graphBg = {graphX, py, graphW, graphH};
    SDL_SetRenderDrawColor(renderer, PanelColors::GRAPH_BG.r, PanelColors::GRAPH_BG.g,
                           PanelColors::GRAPH_BG.b, PanelColors::GRAPH_BG.a);
    SDL_RenderFillRect(renderer, &graphBg);

    // Grid lines
    SDL_SetRenderDrawColor(renderer, PanelColors::GRAPH_GRID.r, PanelColors::GRAPH_GRID.g,
                           PanelColors::GRAPH_GRID.b, PanelColors::GRAPH_GRID.a);
    for (int i = 1; i < 4; i++) {
        int gy = py + (graphH * i / 4);
        SDL_RenderDrawLine(renderer, graphX, gy, graphX + graphW, gy);
    }

    if (m_history.size() < 2) return;

    // Find range
    double minV = m_history[0].magnitude, maxV = m_history[0].magnitude;
    for (auto& r : m_history) {
        if (r.magnitude < minV) minV = r.magnitude;
        if (r.magnitude > maxV) maxV = r.magnitude;
    }
    double range = maxV - minV;
    if (range < 1.0) { range = 1.0; minV -= 0.5; }
    double padding = range * 0.1;
    minV -= padding;
    maxV += padding;
    range = maxV - minV;

    // Draw line graph
    SDL_SetRenderDrawColor(renderer, PanelColors::GRAPH_LINE.r, PanelColors::GRAPH_LINE.g,
                           PanelColors::GRAPH_LINE.b, PanelColors::GRAPH_LINE.a);
    int n = static_cast<int>(m_history.size());
    for (int i = 1; i < n; i++) {
        int x1 = graphX + static_cast<int>((i - 1) * graphW / (MAX_HISTORY - 1));
        int x2 = graphX + static_cast<int>(i * graphW / (MAX_HISTORY - 1));
        int y1 = py + graphH - static_cast<int>((m_history[i-1].magnitude - minV) / range * graphH);
        int y2 = py + graphH - static_cast<int>((m_history[i].magnitude - minV) / range * graphH);
        y1 = std::clamp(y1, py, py + graphH);
        y2 = std::clamp(y2, py, py + graphH);
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    }

    // Min/max labels
    char label[16];
    std::snprintf(label, sizeof(label), "%.1f", maxV);
    renderText(renderer, fontSmall, graphX + 2, py + 2, label, PanelColors::SUBTEXT);
    std::snprintf(label, sizeof(label), "%.1f", minV);
    renderText(renderer, fontSmall, graphX + 2, py + graphH - 14, label, PanelColors::SUBTEXT);
}

} // namespace detections
