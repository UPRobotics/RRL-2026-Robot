#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include "camera_pkg/types.h"

namespace camera_viewer {

// Forward declarations
class ConsoleWindow;
class StatsPanel;
class CameraGrid;
class CameraManager;
class ToolbarButton;

/**
 * @brief Font manager for the application
 */
class FontManager {
public:
    static FontManager& instance();
    
    bool initialize();
    void shutdown();
    
    TTF_Font* getFont(int size);
    TTF_Font* getDefaultFont() { return getFont(12); }
    TTF_Font* getLargeFont() { return getFont(16); }
    TTF_Font* getSmallFont() { return getFont(10); }
    
private:
    FontManager() = default;
    ~FontManager() { shutdown(); }
    
    FontManager(const FontManager&) = delete;
    FontManager& operator=(const FontManager&) = delete;
    
    TTF_Font* m_font10 = nullptr;
    TTF_Font* m_font12 = nullptr;
    TTF_Font* m_font14 = nullptr;
    TTF_Font* m_font16 = nullptr;
    TTF_Font* m_font18 = nullptr;
    bool m_initialized = false;
};

enum class ViewMode {
    FULLSCREEN,  // Single camera fullscreen
    GRID_2X2,    // 2x2 grid (4 cameras)
    GRID_NXN     // NxN grid (auto-layout based on camera count)
};

/**
 * @brief Main application window managing the entire UI
 */
class MainWindow {
public:
    MainWindow(const std::string& title, int width, int height, DecodeMode decodeMode = DecodeMode::GPU);
    ~MainWindow();

    // Delete copy constructor and assignment
    MainWindow(const MainWindow&) = delete;
    MainWindow& operator=(const MainWindow&) = delete;

    /**
     * @brief Initialize the window and all UI components
     * @return true if initialization was successful
     */
    bool initialize();

    /**
     * @brief Run a single iteration of the main loop (non-blocking)
     * @return true if the application should keep running
     */
    bool spinOnce();

    /**
     * @brief Shutdown and cleanup resources
     */
    void shutdown();
    
    /**
     * @brief Check if the window is still running
     */
    bool isRunning() const { return m_running; }

    /**
     * @brief Access the camera manager (e.g. to inject ROS node)
     */
    CameraManager* getCameraManager() const { return m_cameraManager.get(); }

private:
    // Event handling
    void handleEvents();
    void handleMouseClick(int x, int y);
    void handleResize(int width, int height);
    void handleKeyPress(SDL_Keycode key);

    // Rendering
    void render();
    void renderToolbar();
    void renderMainArea();
    void renderStatsBar();

    // UI Actions
    void onStartCamerasClicked();
    void onStopCamerasClicked();
    void onRestartCamerasClicked();
    void onToggleConsoleClicked();
    void onViewModeChanged(ViewMode mode);
    void toggleFullscreen();
    
    // Telemetry
    void startTelemetry();
    void stopTelemetry();
    void telemetryLoop();
    float sampleProcessCpuPercent();
    float sampleProcessRamPercent();
    float pingCameraMs();
    float sampleGpuUsagePercent();

    // Window properties
    std::string m_title;
    int m_windowWidth;
    int m_windowHeight;
    bool m_running;
    bool m_isInitialized;

    // SDL objects
    SDL_Window* m_window;
    SDL_Renderer* m_renderer;

    // UI Layout dimensions
    static constexpr int TOOLBAR_HEIGHT = 50;
    static constexpr int STATSBAR_HEIGHT = 60;
    static constexpr int BUTTON_WIDTH = 120;
    static constexpr int BUTTON_HEIGHT = 35;
    static constexpr int BUTTON_MARGIN = 10;

    // UI State
    ViewMode m_currentViewMode;
    bool m_consolVisible;
    bool m_isFullscreen;
    int m_activeCameraCount;
    // UI Components
    std::unique_ptr<ConsoleWindow> m_consoleWindow;
    std::unique_ptr<StatsPanel> m_statsPanel;
    std::unique_ptr<CameraGrid> m_cameraGrid;
    std::unique_ptr<CameraManager> m_cameraManager;

    // Button states (for hover effects)
    struct ButtonRect {
        SDL_Rect rect;
        std::string label;
        bool hovered;
        bool enabled;
    };
    std::vector<ButtonRect> m_toolbarButtons;
    int m_mainAreaY;
    int m_mainAreaHeight;

    // Telemetry state
    std::thread m_telemetryThread;
    std::atomic<bool> m_telemetryRunning{false};
    std::atomic<float> m_cpuUsageAtomic{0.0f};
    std::atomic<float> m_ramUsageAtomic{0.0f};
    std::atomic<float> m_latencyAtomic{0.0f};
    std::atomic<float> m_gpuUsageAtomic{0.0f};
    std::string m_pingCameraIp;
    uint64_t m_prevProcJiffies = 0;
    uint64_t m_prevTotalJiffies = 0;
    bool m_hasPrevCpuSample = false;
    DecodeMode m_decodeMode;
};

} // namespace camera_viewer
