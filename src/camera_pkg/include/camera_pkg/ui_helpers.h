#pragma once

#include <SDL2/SDL.h>
#include <string>

namespace camera_viewer {

/**
 * @brief Colors used throughout the application
 */
namespace Colors {
    // Main UI colors
    const SDL_Color BACKGROUND = {30, 30, 35, 255};
    const SDL_Color TOOLBAR_BG = {40, 40, 45, 255};
    const SDL_Color STATSBAR_BG = {40, 40, 45, 255};
    const SDL_Color BORDER = {60, 60, 65, 255};
    
    // Button colors
    const SDL_Color BUTTON_NORMAL = {70, 130, 180, 255};
    const SDL_Color BUTTON_HOVER = {100, 160, 210, 255};
    const SDL_Color BUTTON_PRESSED = {50, 100, 150, 255};
    const SDL_Color BUTTON_DISABLED = {80, 80, 85, 255};
    const SDL_Color BUTTON_TEXT = {255, 255, 255, 255};
    
    // Camera grid colors
    const SDL_Color CAMERA_BG = {50, 50, 55, 255};
    const SDL_Color CAMERA_BORDER = {70, 70, 75, 255};
    const SDL_Color CAMERA_ACTIVE = {40, 180, 99, 255};
    const SDL_Color CAMERA_INACTIVE = {231, 76, 60, 255};
    const SDL_Color CAMERA_TEXT = {220, 220, 220, 255};
    
    // Stats colors
    const SDL_Color STAT_LABEL = {180, 180, 180, 255};
    const SDL_Color STAT_VALUE_GOOD = {40, 180, 99, 255};
    const SDL_Color STAT_VALUE_WARNING = {241, 196, 15, 255};
    const SDL_Color STAT_VALUE_CRITICAL = {231, 76, 60, 255};
    
    // Console colors
    const SDL_Color CONSOLE_BG = {20, 20, 25, 255};
    const SDL_Color CONSOLE_TEXT = {200, 200, 200, 255};
    const SDL_Color CONSOLE_DEBUG = {140, 140, 140, 255};
    const SDL_Color CONSOLE_INFO = {100, 160, 210, 255};
    const SDL_Color CONSOLE_WARNING = {241, 196, 15, 255};
    const SDL_Color CONSOLE_ERROR = {231, 76, 60, 255};
}

/**
 * @brief UI drawing helper functions
 */
namespace UIHelpers {
    /**
     * @brief Draw a filled rectangle with rounded corners
     */
    void drawRoundedRect(SDL_Renderer* renderer, SDL_Rect rect, int radius, SDL_Color color);
    
    /**
     * @brief Draw a button
     */
    void drawButton(SDL_Renderer* renderer, SDL_Rect rect, const std::string& label, 
                   bool hovered, bool pressed, bool enabled);
    
    /**
     * @brief Draw text (simplified without TTF for now)
     */
    void drawText(SDL_Renderer* renderer, int x, int y, const std::string& text, 
                 SDL_Color color, int size = 12);
    
    /**
     * @brief Check if point is inside rectangle
     */
    bool pointInRect(int x, int y, SDL_Rect rect);
    
    /**
     * @brief Get color based on percentage value (for stats)
     */
    SDL_Color getStatColor(float percentage);
}

} // namespace camera_viewer
