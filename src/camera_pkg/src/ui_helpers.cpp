#include "camera_pkg/ui_helpers.h"
#include "camera_pkg/main_window.h"
#include <spdlog/spdlog.h>
#include <algorithm>

namespace camera_viewer {

// FontManager implementation
FontManager& FontManager::instance() {
    static FontManager instance;
    return instance;
}

bool FontManager::initialize() {
    if (m_initialized) return true;
    
    if (TTF_Init() == -1) {
        spdlog::error("TTF_Init failed: {}", TTF_GetError());
        return false;
    }
    
    const char* fontPaths[] = {
        "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
    };
    
    const char* selectedFont = nullptr;
    for (const char* path : fontPaths) {
        SDL_RWops* test = SDL_RWFromFile(path, "rb");
        if (test) {
            SDL_RWclose(test);
            selectedFont = path;
            break;
        }
    }
    
    if (!selectedFont) {
        spdlog::error("Could not find any system font");
        return false;
    }
    
    spdlog::info("Using font: {}", selectedFont);
    
    m_font10 = TTF_OpenFont(selectedFont, 10);
    m_font12 = TTF_OpenFont(selectedFont, 12);
    m_font14 = TTF_OpenFont(selectedFont, 14);
    m_font16 = TTF_OpenFont(selectedFont, 16);
    m_font18 = TTF_OpenFont(selectedFont, 18);
    
    if (!m_font12) {
        spdlog::error("Failed to load font: {}", TTF_GetError());
        return false;
    }
    
    m_initialized = true;
    spdlog::info("FontManager initialized");
    return true;
}

void FontManager::shutdown() {
    if (!m_initialized) return;
    
    if (m_font10) TTF_CloseFont(m_font10);
    if (m_font12) TTF_CloseFont(m_font12);
    if (m_font14) TTF_CloseFont(m_font14);
    if (m_font16) TTF_CloseFont(m_font16);
    if (m_font18) TTF_CloseFont(m_font18);
    
    m_font10 = m_font12 = m_font14 = m_font16 = m_font18 = nullptr;
    
    TTF_Quit();
    m_initialized = false;
}

TTF_Font* FontManager::getFont(int size) {
    switch (size) {
        case 10: return m_font10;
        case 11: return m_font10;  // fall back to closest
        case 12: return m_font12;
        case 14: return m_font14;
        case 16: return m_font16;
        case 18: return m_font18;
        default: return m_font12;
    }
}

namespace UIHelpers {

void drawRoundedRect(SDL_Renderer* renderer, SDL_Rect rect, int /*radius*/, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderFillRect(renderer, &rect);
}

void drawButton(SDL_Renderer* renderer, SDL_Rect rect, const std::string& label,
               bool hovered, bool pressed, bool enabled) {
    SDL_Color buttonColor;
    if (!enabled) {
        buttonColor = Colors::BUTTON_DISABLED;
    } else if (pressed) {
        buttonColor = Colors::BUTTON_PRESSED;
    } else if (hovered) {
        buttonColor = Colors::BUTTON_HOVER;
    } else {
        buttonColor = Colors::BUTTON_NORMAL;
    }

    drawRoundedRect(renderer, rect, 4, buttonColor);

    SDL_SetRenderDrawColor(renderer, 50, 50, 55, 255);
    SDL_RenderDrawRect(renderer, &rect);

    SDL_Color textColor = enabled ? Colors::BUTTON_TEXT : SDL_Color{150, 150, 150, 255};
    
    TTF_Font* font = FontManager::instance().getFont(12);
    if (font) {
        SDL_Surface* textSurface = TTF_RenderText_Blended(font, label.c_str(), textColor);
        if (textSurface) {
            SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
            if (textTexture) {
                int textW = textSurface->w;
                int textH = textSurface->h;
                SDL_Rect textRect = {
                    rect.x + (rect.w - textW) / 2,
                    rect.y + (rect.h - textH) / 2,
                    textW,
                    textH
                };
                SDL_RenderCopy(renderer, textTexture, nullptr, &textRect);
                SDL_DestroyTexture(textTexture);
            }
            SDL_FreeSurface(textSurface);
        }
    }
}

void drawText(SDL_Renderer* renderer, int x, int y, const std::string& text,
             SDL_Color color, int size) {
    TTF_Font* font = FontManager::instance().getFont(size);
    if (!font || text.empty()) return;
    
    SDL_Surface* textSurface = TTF_RenderText_Blended(font, text.c_str(), color);
    if (textSurface) {
        SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
        if (textTexture) {
            SDL_Rect textRect = {x, y, textSurface->w, textSurface->h};
            SDL_RenderCopy(renderer, textTexture, nullptr, &textRect);
            SDL_DestroyTexture(textTexture);
        }
        SDL_FreeSurface(textSurface);
    }
}

bool pointInRect(int x, int y, SDL_Rect rect) {
    return (x >= rect.x && x <= rect.x + rect.w &&
            y >= rect.y && y <= rect.y + rect.h);
}

SDL_Color getStatColor(float percentage) {
    if (percentage < 60.0f) {
        return Colors::STAT_VALUE_GOOD;
    } else if (percentage < 80.0f) {
        return Colors::STAT_VALUE_WARNING;
    } else {
        return Colors::STAT_VALUE_CRITICAL;
    }
}

} // namespace UIHelpers
} // namespace camera_viewer
