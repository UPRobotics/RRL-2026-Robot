#pragma once

#include <SDL2/SDL.h>
#include <string>
#include <vector>

namespace camera_viewer {

/**
 * @brief Console window for displaying logs, warnings, and errors
 */
class ConsoleWindow {
public:
    ConsoleWindow();
    ~ConsoleWindow();

    /**
     * @brief Create and show the console window
     */
    bool create(int x, int y, int width, int height);

    /**
     * @brief Show or hide the console window
     */
    void setVisible(bool visible);
    bool isVisible() const { return m_visible; }

    /**
     * @brief Add a log message to the console
     */
    void addLog(const std::string& message, SDL_Color color);

    /**
     * @brief Render the console window
     */
    void render();

    /**
     * @brief Handle window events
     */
    void handleEvent(const SDL_Event& event);

private:
    SDL_Window* m_window;
    SDL_Renderer* m_renderer;
    bool m_visible;
    int m_width;
    int m_height;

    struct LogEntry {
        std::string message;
        SDL_Color color;
    };
    std::vector<LogEntry> m_logMessages;
    static constexpr size_t MAX_LOG_ENTRIES = 1000;
};

} // namespace camera_viewer
