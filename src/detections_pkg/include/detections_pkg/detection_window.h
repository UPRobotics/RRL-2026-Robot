#pragma once

#include "detections_pkg/types.h"
#include "detections_pkg/detection_stream.h"
#include "detections_pkg/qr_detector.h"
#include "detections_pkg/motion_detector.h"
#include "detections_pkg/hazmat_detector.h"
#include "detections_pkg/magnetometer_panel.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>
#include <set>
#include <thread>
#include <atomic>

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
    void renderToggleBar(SDL_Renderer* r, SDL_Rect area);
    void renderSettingsPanel(SDL_Renderer* r, SDL_Rect area);
    void handleMouseClick(int mx, int my);
    void saveConfig();
    void rotateFrame(cv::Mat& frame);
    void saveQRFrame(const cv::Mat& frame, const QRDetection& qr);
    void renderQRCrop(SDL_Renderer* r, SDL_Rect videoArea);
    void saveHazmatCrop(const cv::Mat& frame, const HazmatDetection& hz);
    void renderHazmatCrop(SDL_Renderer* r, SDL_Rect videoArea);
    void renderFooter(SDL_Renderer* r, SDL_Rect area);

    // Telemetry
    void startTelemetry();
    void stopTelemetry();
    void telemetryLoop();
    float sampleCpuPercent();
    float sampleRamPercent();
    void  sampleGpuStats();
    float pingCameraMs();

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
    SettingRow m_btnHazmatInterval{};
    SettingRow m_btnHazmatConf{};
    SettingRow m_btnHazmatNms{};

    // Toggle bar button rects
    SDL_Rect m_toggleAll{};
    SDL_Rect m_toggleQR{};
    SDL_Rect m_toggleMotion{};
    SDL_Rect m_toggleHazmat{};

    // SDL
    SDL_Window*   m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;

    // Subsystems
    std::unique_ptr<DetectionStream>   m_stream;
    std::unique_ptr<MagnetometerPanel> m_magPanel;
    QRDetector      m_qrDetector;
    MotionDetector  m_motionDetector;
    HazmatDetector  m_hazmatDetector;

    // Detection results (updated each frame from the latest BGR grab)
    std::vector<QRDetection> m_qrResults;
    std::vector<MotionBox>   m_motionResults;
    std::vector<HazmatDetection> m_hazmatResults;

    // Frame dimensions from the stream
    int m_frameW = 0;
    int m_frameH = 0;

    bool m_quit = false;

    // QR crop display
    SDL_Texture* m_qrCropTex = nullptr;
    int m_qrCropW = 0;
    int m_qrCropH = 0;
    std::string m_qrCropData;
    std::chrono::steady_clock::time_point m_qrCropTime;

    // Last full frame for QR save
    cv::Mat m_lastFrame;
    std::set<std::string> m_seenQRData;

    // Hazmat crop display
    SDL_Texture* m_hazmatCropTex = nullptr;
    int m_hazmatCropW = 0;
    int m_hazmatCropH = 0;
    std::string m_hazmatCropLabel;
    std::chrono::steady_clock::time_point m_hazmatCropTime;
    std::set<std::string> m_seenHazmatClasses;

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

    // Telemetry state
    std::thread m_telemetryThread;
    std::atomic<bool> m_telemetryRunning{false};
    std::atomic<float> m_cpuUsage{0.0f};
    std::atomic<float> m_ramUsage{0.0f};
    std::atomic<float> m_gpuUsage{0.0f};
    std::atomic<float> m_vramUsedMb{0.0f};
    std::atomic<float> m_vramTotalMb{0.0f};
    std::atomic<float> m_latency{0.0f};
    uint64_t m_prevProcJiffies = 0;
    uint64_t m_prevTotalJiffies = 0;
    bool m_hasPrevCpuSample = false;
    std::string m_pingCameraIp;
};

} // namespace detections
