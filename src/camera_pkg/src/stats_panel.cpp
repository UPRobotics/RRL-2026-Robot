#include "camera_pkg/stats_panel.h"
#include "camera_pkg/ui_helpers.h"
#include <cstdio>

namespace camera_viewer {

StatsPanel::StatsPanel()
    : m_cpuUsage(0.0f)
    , m_ramUsage(0.0f)
    , m_latency(0.0f)
    , m_gpuUsage(0.0f)
{
}

void StatsPanel::updateCpuUsage(float percentage) {
    m_cpuUsage = percentage;
}

void StatsPanel::updateRamUsage(float percentage) {
    m_ramUsage = percentage;
}

void StatsPanel::updateLatency(float milliseconds) {
    m_latency = milliseconds;
}

void StatsPanel::updateGpuUsage(float percentage) {
    m_gpuUsage = percentage;
}

void StatsPanel::render(SDL_Renderer* renderer, int x, int y, int width, int height) {
    // Calculate section widths
    int sectionWidth = width / 4;

    // Format values
    char cpuText[32], ramText[32], gpuText[32], latencyText[32];
    snprintf(cpuText, sizeof(cpuText), "%.1f%%", m_cpuUsage);
    snprintf(ramText, sizeof(ramText), "%.1f%%", m_ramUsage);
    snprintf(gpuText, sizeof(gpuText), "%.1f%%", m_gpuUsage);
    snprintf(latencyText, sizeof(latencyText), "%.1f ms", m_latency);

    // Render CPU stat
    SDL_Color cpuColor = UIHelpers::getStatColor(m_cpuUsage);
    renderStatItem(renderer, x + STAT_PADDING, y + STAT_PADDING, 
                  sectionWidth - STAT_PADDING * 2, "CPU Usage", cpuText, cpuColor);

    // Render RAM stat
    SDL_Color ramColor = UIHelpers::getStatColor(m_ramUsage);
    renderStatItem(renderer, x + sectionWidth + STAT_PADDING, y + STAT_PADDING,
                  sectionWidth - STAT_PADDING * 2, "RAM Usage", ramText, ramColor);

    // Render GPU stat
    SDL_Color gpuColor = UIHelpers::getStatColor(m_gpuUsage);
    renderStatItem(renderer, x + sectionWidth * 2 + STAT_PADDING, y + STAT_PADDING,
                  sectionWidth - STAT_PADDING * 2, "GPU Usage", gpuText, gpuColor);

    // Render Latency stat (different color scheme - lower is better)
    SDL_Color latencyColor;
    if (m_latency < 50.0f) {
        latencyColor = Colors::STAT_VALUE_GOOD;
    } else if (m_latency < 100.0f) {
        latencyColor = Colors::STAT_VALUE_WARNING;
    } else {
        latencyColor = Colors::STAT_VALUE_CRITICAL;
    }
    renderStatItem(renderer, x + sectionWidth * 3 + STAT_PADDING, y + STAT_PADDING,
                  sectionWidth - STAT_PADDING * 2, "Camera Latency", latencyText, latencyColor);
}

void StatsPanel::renderStatItem(SDL_Renderer* renderer, int x, int y, int width,
                                const char* label, const char* value, SDL_Color valueColor) {
    // Draw label
    UIHelpers::drawText(renderer, x, y, label, Colors::STAT_LABEL, 11);

    // Draw value below label
    UIHelpers::drawText(renderer, x, y + TEXT_HEIGHT, value, valueColor, 16);
}

} // namespace camera_viewer
