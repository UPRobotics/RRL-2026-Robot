#include "detections_pkg/thermal_panel.h"
#include <algorithm>
#include <cstring>
#include <spdlog/spdlog.h>

namespace detections {

// ---------------------------------------------------------------------------
// Inferno colormap – 256 entries precomputed from matplotlib's inferno LUT.
// Index 0 = coldest (dark purple), 255 = hottest (pale yellow).
// ---------------------------------------------------------------------------
static const SDL_Color INFERNO[256] = {
    {0,0,4,255},{1,0,5,255},{1,1,6,255},{1,1,8,255},{2,1,9,255},{2,2,11,255},{2,2,13,255},{3,3,15,255},
    {3,3,18,255},{4,4,20,255},{5,4,22,255},{6,5,24,255},{6,5,26,255},{7,6,28,255},{8,7,30,255},{9,7,32,255},
    {10,8,34,255},{11,9,36,255},{12,9,38,255},{13,10,41,255},{14,11,43,255},{16,11,45,255},{17,12,47,255},{18,13,49,255},
    {19,13,52,255},{20,14,54,255},{21,14,56,255},{22,15,59,255},{24,15,61,255},{25,16,63,255},{26,16,66,255},{28,16,68,255},
    {29,17,71,255},{30,17,73,255},{32,17,75,255},{33,17,78,255},{34,17,80,255},{36,18,83,255},{37,18,85,255},{39,18,88,255},
    {41,17,90,255},{42,17,92,255},{44,17,95,255},{45,17,97,255},{47,17,99,255},{49,17,101,255},{51,16,103,255},{52,16,105,255},
    {54,16,107,255},{56,15,109,255},{57,15,111,255},{59,15,113,255},{61,15,114,255},{63,14,116,255},{64,14,117,255},{66,13,119,255},
    {68,13,120,255},{69,13,121,255},{71,12,122,255},{73,12,123,255},{74,12,124,255},{76,12,125,255},{77,11,126,255},{79,11,127,255},
    {81,11,128,255},{82,11,129,255},{84,11,130,255},{85,11,131,255},{87,11,131,255},{89,10,132,255},{90,10,133,255},{92,10,134,255},
    {94,10,134,255},{95,10,135,255},{97,10,136,255},{99,10,136,255},{100,10,137,255},{102,10,137,255},{103,10,138,255},{105,10,138,255},
    {107,10,139,255},{108,10,139,255},{110,10,140,255},{112,10,140,255},{113,10,141,255},{115,10,141,255},{116,10,141,255},{118,10,142,255},
    {120,10,142,255},{121,10,142,255},{123,10,143,255},{125,10,143,255},{126,10,143,255},{128,10,143,255},{130,10,144,255},{131,10,144,255},
    {133,10,144,255},{135,10,144,255},{136,10,144,255},{138,10,144,255},{140,10,144,255},{141,10,144,255},{143,10,144,255},{145,10,144,255},
    {146,10,144,255},{148,11,144,255},{150,11,144,255},{151,11,144,255},{153,11,143,255},{155,11,143,255},{156,11,143,255},{158,12,142,255},
    {160,12,142,255},{161,12,142,255},{163,13,141,255},{165,13,141,255},{166,13,140,255},{168,14,140,255},{170,14,139,255},{171,14,139,255},
    {173,15,138,255},{175,15,138,255},{176,16,137,255},{178,16,136,255},{180,17,136,255},{181,17,135,255},{183,18,134,255},{185,19,134,255},
    {186,19,133,255},{188,20,132,255},{190,21,131,255},{191,21,130,255},{193,22,129,255},{195,23,128,255},{196,24,127,255},{198,24,126,255},
    {200,25,125,255},{201,26,124,255},{203,27,123,255},{205,28,122,255},{206,29,121,255},{208,30,120,255},{210,31,118,255},{211,32,117,255},
    {213,33,116,255},{215,34,115,255},{216,35,113,255},{218,37,112,255},{220,38,111,255},{221,39,109,255},{223,40,108,255},{225,42,107,255},
    {226,43,105,255},{228,44,104,255},{230,46,102,255},{231,47,101,255},{233,49,99,255},{235,50,98,255},{236,52,96,255},{238,53,95,255},
    {240,55,93,255},{241,57,91,255},{243,58,90,255},{245,60,88,255},{246,62,86,255},{248,64,85,255},{249,65,83,255},{251,67,81,255},
    {252,69,79,255},{253,71,78,255},{253,73,76,255},{254,75,74,255},{254,77,72,255},{254,79,71,255},{254,81,69,255},{254,83,67,255},
    {254,85,65,255},{254,87,63,255},{254,89,61,255},{254,92,59,255},{254,94,57,255},{254,96,55,255},{254,98,53,255},{254,101,51,255},
    {254,103,49,255},{254,105,47,255},{254,108,45,255},{254,110,43,255},{254,112,41,255},{254,115,39,255},{254,117,37,255},{254,119,35,255},
    {254,122,33,255},{254,124,31,255},{254,127,28,255},{254,129,26,255},{254,132,24,255},{253,134,22,255},{253,137,20,255},{253,139,18,255},
    {253,142,16,255},{252,144,14,255},{252,147,12,255},{252,150,10,255},{252,152,8,255},{251,155,7,255},{251,158,6,255},{250,160,6,255},
    {250,163,6,255},{249,166,7,255},{249,168,8,255},{248,171,10,255},{248,174,12,255},{247,177,14,255},{247,179,16,255},{246,182,19,255},
    {246,185,22,255},{245,188,25,255},{245,191,28,255},{244,193,32,255},{244,196,35,255},{243,199,39,255},{243,202,43,255},{242,205,47,255},
    {241,208,51,255},{241,211,55,255},{240,213,59,255},{240,216,64,255},{239,219,68,255},{238,222,73,255},{238,225,78,255},{237,228,83,255},
    {236,230,88,255},{236,233,93,255},{235,236,98,255},{234,239,103,255},{234,242,109,255},{233,245,114,255},{232,247,120,255},{232,250,125,255},
    {231,253,131,255},{231,254,137,255},{232,254,142,255},{233,254,148,255},{234,254,153,255},{235,254,159,255},{236,254,164,255},{252,255,164,255}
};

// ---------------------------------------------------------------------------

ThermalPanel::ThermalPanel(rclcpp::Node::SharedPtr node)
{
    m_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/thermal_data", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            onThermalData(msg);
        });
}

ThermalPanel::~ThermalPanel()
{
    if (m_tex) {
        SDL_DestroyTexture(m_tex);
        m_tex = nullptr;
    }
}

void ThermalPanel::onThermalData(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 768) return;
    std::lock_guard<std::mutex> lk(m_mutex);
    m_data.assign(msg->data.begin(), msg->data.end());
    m_hasData = true;
    m_dirty   = true;
    m_lastUpdate = std::chrono::steady_clock::now();
}

void ThermalPanel::render(SDL_Renderer* r, SDL_Rect area, TTF_Font* font12, TTF_Font* font14)
{
    // Panel background
    SDL_SetRenderDrawColor(r, 30, 30, 46, 255);
    SDL_RenderFillRect(r, &area);

    // Left border separator
    SDL_SetRenderDrawColor(r, 69, 71, 90, 255);
    SDL_RenderDrawLine(r, area.x, area.y, area.x, area.y + area.h);

    // Title
    if (font12) {
        SDL_Color titleColor = {166, 173, 200, 255};
        SDL_Surface* surf = TTF_RenderText_Blended(font12, "THERMAL", titleColor);
        if (surf) {
            SDL_Texture* tex = SDL_CreateTextureFromSurface(r, surf);
            SDL_Rect dst = {area.x + 8, area.y + 6, surf->w, surf->h};
            SDL_RenderCopy(r, tex, nullptr, &dst);
            SDL_DestroyTexture(tex);
            SDL_FreeSurface(surf);
        }
    }

    std::unique_lock<std::mutex> lk(m_mutex);
    if (!m_hasData) {
        lk.unlock();
        if (font14) {
            SDL_Color c = {100, 100, 120, 255};
            SDL_Surface* surf = TTF_RenderText_Blended(font14, "NO DATA", c);
            if (surf) {
                SDL_Texture* tex = SDL_CreateTextureFromSurface(r, surf);
                SDL_Rect dst = {area.x + area.w / 2 - surf->w / 2,
                                area.y + area.h / 2 - surf->h / 2,
                                surf->w, surf->h};
                SDL_RenderCopy(r, tex, nullptr, &dst);
                SDL_DestroyTexture(tex);
                SDL_FreeSurface(surf);
            }
        }
        return;
    }

    // Status dot: green if data within 3 seconds
    auto age = std::chrono::steady_clock::now() - m_lastUpdate;
    bool fresh = age < std::chrono::seconds(3);
    SDL_Color dotColor = fresh ? SDL_Color{40, 180, 99, 255} : SDL_Color{231, 76, 60, 255};

    // Copy data for processing
    std::vector<float> data = m_data;
    bool needsRebuild = m_dirty;
    m_dirty = false;
    lk.unlock();

    // ---- Build texture if needed ----
    if (needsRebuild) {
        // Percentile normalisation: find 2nd and 98th percentile
        std::vector<float> sorted = data;
        std::sort(sorted.begin(), sorted.end());
        float lo = sorted[static_cast<int>(sorted.size() * 0.02f)];
        float hi = sorted[static_cast<int>(sorted.size() * 0.98f)];
        if (hi <= lo) hi = lo + 1.0f;

        // Build 32×24 RGBA pixel buffer
        std::vector<uint32_t> pixels(768);
        for (int i = 0; i < 768; i++) {
            float norm = (data[i] - lo) / (hi - lo);
            int idx = static_cast<int>(std::clamp(norm, 0.0f, 1.0f) * 255.0f);
            const SDL_Color& c = INFERNO[idx];
            // SDL_PIXELFORMAT_RGBA8888: R in high byte
            pixels[i] = (static_cast<uint32_t>(c.r) << 24) |
                        (static_cast<uint32_t>(c.g) << 16) |
                        (static_cast<uint32_t>(c.b) <<  8) |
                        0xFF;
        }

        if (!m_tex) {
            m_tex = SDL_CreateTexture(r,
                SDL_PIXELFORMAT_RGBA8888,
                SDL_TEXTUREACCESS_STREAMING,
                32, 24);
        }
        if (m_tex) {
            SDL_UpdateTexture(m_tex, nullptr, pixels.data(), 32 * sizeof(uint32_t));
        }
    }

    // ---- Render texture scaled to panel area ----
    if (m_tex) {
        int titleH = 22;
        int labelH = font12 ? (TTF_FontHeight(font12) + 4) : 18;
        SDL_Rect imgArea = {
            area.x + 1,
            area.y + titleH,
            area.w - 1,
            area.h - titleH - labelH * 2
        };
        SDL_RenderCopy(r, m_tex, nullptr, &imgArea);

        // Min / max temperature labels underneath the image
        float minT = *std::min_element(data.begin(), data.end());
        float maxT = *std::max_element(data.begin(), data.end());
        int labelY = imgArea.y + imgArea.h + 2;

        if (font12) {
            char buf[48];
            // Min label (cyan)
            std::snprintf(buf, sizeof(buf), "Min: %.1f\xC2\xB0""C", minT);
            SDL_Surface* s = TTF_RenderText_Blended(font12, buf, {0, 180, 200, 255});
            if (s) {
                SDL_Texture* t = SDL_CreateTextureFromSurface(r, s);
                SDL_Rect d = {area.x + 6, labelY, s->w, s->h};
                SDL_RenderCopy(r, t, nullptr, &d);
                SDL_DestroyTexture(t); SDL_FreeSurface(s);
            }
            // Max label (orange-red)
            std::snprintf(buf, sizeof(buf), "Max: %.1f\xC2\xB0""C", maxT);
            s = TTF_RenderText_Blended(font12, buf, {255, 100, 50, 255});
            if (s) {
                SDL_Texture* t = SDL_CreateTextureFromSurface(r, s);
                SDL_Rect d = {area.x + area.w - s->w - 6, labelY, s->w, s->h};
                SDL_RenderCopy(r, t, nullptr, &d);
                SDL_DestroyTexture(t); SDL_FreeSurface(s);
            }
        }
    }

    // Status dot (top-right of title row)
    SDL_Rect dot = {area.x + area.w - 14, area.y + 8, 8, 8};
    SDL_SetRenderDrawColor(r, dotColor.r, dotColor.g, dotColor.b, 255);
    SDL_RenderFillRect(r, &dot);
}

} // namespace detections
