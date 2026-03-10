#pragma once

#include "detections_pkg/types.h"
#include "detections_pkg/detection_stream.h"
#include "detections_pkg/qr_detector.h"
#include "detections_pkg/motion_detector.h"
#include "detections_pkg/magnetometer_panel.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace detections {

/// Main SDL2 window: renders camera feed with detection overlays
/// and magnetometer side panel.
class DetectionWindow {
public:
    DetectionWindow(const DetectionConfig& config, rclcpp::Node::SharedPtr node);
    ~DetectionWindow();

    bool initialize();
    void shutdown();

    /// Run one iteration of the event loop.  Returns false when the window
    /// should close.
    bool spinOnce();

private:
    void processEvents();
    void render();
    void drawDetections(SDL_Renderer* r, SDL_Rect videoRect,
                        int srcW, int srcH);
    void renderSettingsPanel(SDL_Renderer* r, SDL_Rect area);
    void handleMouseClick(int mx, int my);
    void saveConfig();

    // Returns true if (mx,my) is inside rect
    static bool inRect(int mx, int my, SDL_Rect r) {
        return mx >= r.x && mx < r.x + r.w && my >= r.y && my < r.y + r.h;
    }

    DetectionConfig m_config;
    rclcpp::Node::SharedPtr m_node;

    // Settings button rects (populated during render)
    struct SettingRow {
        SDL_Rect minus;
        SDL_Rect plus;
    };
    SettingRow m_btnMinArea{};
    SettingRow m_btnThreshold{};
    SettingRow m_btnWeight{};
    SettingRow m_btnSkipFrames{};
    SettingRow m_btnQRInterval{};

    // SDL
    SDL_Window*   m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;

    // Subsystems
    std::unique_ptr<DetectionStream>   m_stream;
    std::unique_ptr<MagnetometerPanel> m_magPanel;
    QRDetector      m_qrDetector;
    MotionDetector  m_motionDetector;

    // Detection results (updated each frame from the latest BGR grab)
    std::vector<QRDetection> m_qrResults;
    std::vector<MotionBox>   m_motionResults;

    // Frame dimensions from the stream
    int m_frameW = 0;
    int m_frameH = 0;

    bool m_quit = false;

    // Font
    TTF_Font* m_font12 = nullptr;
    TTF_Font* m_font14 = nullptr;
    TTF_Font* m_font16 = nullptr;
    TTF_Font* m_font18 = nullptr;
    TTF_Font* m_font24 = nullptr;
    bool initFonts();
    void drawText(SDL_Renderer* r, int x, int y,
                  const std::string& text, SDL_Color color,
                  TTF_Font* font = nullptr);
};

} // namespace detections
