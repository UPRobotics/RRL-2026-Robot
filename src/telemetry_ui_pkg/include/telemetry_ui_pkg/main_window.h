#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_msgs/msg/motor_config.hpp>
#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace telemetry_ui {

// -------------------------------------------------------
// Color palette — matches camera_pkg / detections_pkg
// -------------------------------------------------------
namespace Colors {
    constexpr SDL_Color BACKGROUND  = {30,  30,  35,  255};
    constexpr SDL_Color FOOTER_BG   = {40,  40,  45,  255};
    constexpr SDL_Color BORDER      = {60,  60,  65,  255};
    constexpr SDL_Color TEXT        = {220, 220, 220, 255};
    constexpr SDL_Color STAT_LABEL  = {180, 180, 180, 255};
    constexpr SDL_Color STAT_GOOD   = {40,  180, 99,  255};
    constexpr SDL_Color STAT_WARN   = {241, 196, 15,  255};
    constexpr SDL_Color STAT_CRIT   = {231, 76,  60,  255};
    constexpr SDL_Color CARD_BG     = {45,  45,  50,  255};
    constexpr SDL_Color CARD_SELECT = {55,  55,  70,  255};
    constexpr SDL_Color SIDEBAR_BG  = {35,  35,  40,  255};
    constexpr SDL_Color ACCENT_BLUE = {70,  130, 180, 255};
    constexpr SDL_Color BUTTON_BG   = {55,  55,  60,  255};
    constexpr SDL_Color BUTTON_HOV  = {70,  70,  80,  255};
}

// -------------------------------------------------------
// Motor telemetry data (read from aggregated JSON)
// -------------------------------------------------------
static constexpr int NUM_MOTORS = 10;

struct MotorData {
    uint8_t     motor_id     = 0;
    std::string motor_name;
    int         config_index = -1;
    int32_t     rpm          = 0;
    float       duty_cycle   = 0.0f;
    float       voltage      = 0.0f;
    uint8_t     control_mode = 0;
    bool        inverted     = false;
    bool        received     = false;
};

// -------------------------------------------------------
// MainWindow
// -------------------------------------------------------
class MainWindow {
public:
    MainWindow(const std::string& title, int width, int height);
    ~MainWindow();

    MainWindow(const MainWindow&)            = delete;
    MainWindow& operator=(const MainWindow&) = delete;

    bool initialize();
    bool spinOnce();
    void shutdown();
    void setRosNode(rclcpp::Node::SharedPtr node);

    bool isRunning() const { return m_running; }

private:
    // Event & render
    void handleEvents();
    void render();
    void renderMainArea(int winW, int winH);
    void renderFooter(int winW, int winH);
    void renderMotorGrid(int x, int y, int w, int h);
    void renderMotorCard(int x, int y, int w, int h,
                         const MotorData& motor, bool selected);
    void renderSidebar(int x, int y, int w, int h);
    void renderEditPanel(int x, int y, int w, int h);

    // ROS2 callbacks
    void onTelemetryReceived(const std_msgs::msg::String::SharedPtr msg);
    void publishConfigUpdate(int configIndex);

    // Mouse handling
    void handleMouseClick(int mx, int my);

    // Telemetry background thread (system stats)
    void  startTelemetry();
    void  stopTelemetry();
    void  telemetryLoop();
    float sampleCpuPercent();
    float sampleRamPercent();
    void  sampleGpuStats();

    // Drawing helpers
    SDL_Color getStatColor(float pct) const;
    void drawText(int x, int y, const std::string& text,
                  SDL_Color color, TTF_Font* fnt);
    void drawFilledRect(int x, int y, int w, int h, SDL_Color color);
    void drawBorderRect(int x, int y, int w, int h, SDL_Color color);
    bool pointInRect(int px, int py, int rx, int ry, int rw, int rh) const;
    void drawButton(int x, int y, int w, int h,
                    const std::string& label, SDL_Color bg, TTF_Font* fnt);

    // Window properties
    int           m_width;
    int           m_height;
    bool          m_running     = false;
    bool          m_initialized = false;

    // SDL objects
    SDL_Window*   m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;

    // Fonts
    TTF_Font* m_font10 = nullptr;
    TTF_Font* m_font12 = nullptr;
    TTF_Font* m_font14 = nullptr;
    TTF_Font* m_font16 = nullptr;

    // Layout constants
    static constexpr int FOOTER_H    = 60;
    static constexpr int STAT_PAD    = 15;
    static constexpr int SIDEBAR_W   = 220;
    static constexpr int EDIT_PANEL_H = 130;
    static constexpr int CARD_PAD    = 6;

    // System telemetry thread
    std::thread        m_telemetryThread;
    std::atomic<bool>  m_telemetryRunning{false};
    std::atomic<float> m_cpu{0.0f};
    std::atomic<float> m_ram{0.0f};
    std::atomic<float> m_gpu{0.0f};
    std::atomic<float> m_vramUsedMb{0.0f};
    std::atomic<float> m_vramTotalMb{0.0f};

    // CPU delta tracking (/proc/stat)
    uint64_t m_prevTotalJiffies  = 0;
    uint64_t m_prevActiveJiffies = 0;
    bool     m_hasPrevCpu        = false;

    // ROS2 integration
    rclcpp::Node::SharedPtr m_rosNode;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_telemSub;
    rclcpp::Publisher<robot_msgs::msg::MotorConfig>::SharedPtr m_configPub;

    // Motor data (written by ROS callback, read by render)
    std::mutex m_motorMutex;
    std::array<MotorData, NUM_MOTORS> m_motors;

    // UI state
    int  m_selectedMotor   = -1;   // config_index of selected motor (-1 = none)
    bool m_editPanelOpen   = false;
    int  m_lastClickX      = 0;
    int  m_lastClickY      = 0;

    // Edit panel temporary state
    float   m_editRpmLimit       = 0.0f;
    float   m_editDutyCycleLimit = 0.0f;
    uint8_t m_editControlMode    = 0;
    bool    m_editInverted       = false;

    // Cached layout rects for hit testing (updated each frame)
    struct CardRect { int x, y, w, h; int configIndex; };
    std::vector<CardRect> m_cardRects;

    struct ButtonRect { int x, y, w, h; int action; };
    std::vector<ButtonRect> m_editButtons;
};

} // namespace telemetry_ui
