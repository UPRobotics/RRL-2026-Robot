#pragma once

#include <SDL2/SDL.h>

namespace camera_viewer {

/**
 * @brief Stats panel displaying CPU, RAM, and latency information
 */
class StatsPanel {
public:
    StatsPanel();
    ~StatsPanel() = default;

    /**
     * @brief Update statistics values
     */
    void updateCpuUsage(float percentage);
    void updateRamUsage(float percentage);
    void updateLatency(float milliseconds);
    void updateGpuUsage(float percentage);

    /**
     * @brief Render the stats panel
     */
    void render(SDL_Renderer* renderer, int x, int y, int width, int height);

private:
    void renderStatItem(SDL_Renderer* renderer, int x, int y, int width, 
                       const char* label, const char* value, SDL_Color valueColor);

    float m_cpuUsage;    // Percentage (0-100)
    float m_ramUsage;    // Percentage (0-100)
    float m_latency;     // Milliseconds
    float m_gpuUsage;    // Percentage (0-100)

    static constexpr int STAT_PADDING = 15;
    static constexpr int TEXT_HEIGHT = 20;
};

} // namespace camera_viewer
