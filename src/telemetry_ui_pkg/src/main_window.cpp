#include "telemetry_ui_pkg/main_window.h"
#include "telemetry_ui_pkg/json.hpp"
#include <std_msgs/msg/bool.hpp>
#include <robot_msgs/msg/d_pad_config.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <thread>
#include <unistd.h>

using json = nlohmann::json;

namespace telemetry_ui {

// Edit-panel button action IDs
enum EditAction {
    ACT_MODE_TOGGLE = 0,
    ACT_INV_TOGGLE,
    ACT_ENABLED_TOGGLE,
    ACT_APPLY,
};

// Footer button actions
static const int ACT_SET_AUTO   = 100;
static const int ACT_SET_TELEOP = 101;

// Motor group definitions — config_index values that belong together
static const std::array<GroupDef, 2> MOTOR_GROUPS = {{
    { "Flippers", {0, 3} },   // Flipper Trasero (0), Flipper Delantero (3)
    { "Tracks",   {1, 2} },   // Track Izquierdo (1), Track Derecho (2)
}};

// -------------------------------------------------------
// Construction / destruction
// -------------------------------------------------------

MainWindow::MainWindow(const std::string& /*title*/, int width, int height)
    : m_width(width), m_height(height)
{
}

MainWindow::~MainWindow()
{
    shutdown();
}

// -------------------------------------------------------
// Lifecycle
// -------------------------------------------------------

bool MainWindow::initialize()
{
    if (m_initialized) return true;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        spdlog::error("SDL_Init failed: {}", SDL_GetError());
        return false;
    }

    if (TTF_Init() == -1) {
        spdlog::error("TTF_Init failed: {}", TTF_GetError());
        SDL_Quit();
        return false;
    }

    const char* fontPath = "/usr/share/fonts/truetype/freefont/FreeMono.ttf";
    m_font12 = TTF_OpenFont(fontPath, 13);
    m_font14 = TTF_OpenFont(fontPath, 15);
    m_font16 = TTF_OpenFont(fontPath, 18);
    m_font18 = TTF_OpenFont(fontPath, 20);
    m_font22 = TTF_OpenFont(fontPath, 24);

    if (!m_font14) {
        spdlog::error("Failed to load font '{}': {}", fontPath, TTF_GetError());
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    m_window = SDL_CreateWindow(
        "Telemetry Dashboard",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        m_width, m_height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    if (!m_window) {
        spdlog::error("SDL_CreateWindow failed: {}", SDL_GetError());
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED);
    if (!m_renderer) {
        spdlog::error("SDL_CreateRenderer failed: {}", SDL_GetError());
        SDL_DestroyWindow(m_window);
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    m_initialized = true;
    m_running     = true;

    startTelemetry();
    spdlog::info("TelemetryUI initialized ({}x{})", m_width, m_height);
    return true;
}

void MainWindow::setRosNode(rclcpp::Node::SharedPtr node)
{
    m_rosNode = node;

    // QoS matching telemetry_pkg's aggregated publisher
    auto telem_qos = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort()
        .durability_volatile();

    m_telemSub = m_rosNode->create_subscription<std_msgs::msg::String>(
        "/telemetry/aggregated", telem_qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            onTelemetryReceived(msg);
        }
    );

    auto config_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    m_configPub = m_rosNode->create_publisher<robot_msgs::msg::MotorConfig>(
        "/ground_station/motor_config", config_qos
    );

    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    m_modeSub = m_rosNode->create_subscription<std_msgs::msg::UInt8>(
        "/robot/mode", best_effort_qos,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
            m_currentMode.store(msg->data, std::memory_order_relaxed);
        }
    );

    m_dpadConfigPub = m_rosNode->create_publisher<robot_msgs::msg::DPadConfig>(
        "/ground_station/dpad_config", config_qos
    );

    // Publisher for autonomy flag (published on demand when buttons pressed)
    m_autonomyPub = m_rosNode->create_publisher<std_msgs::msg::Bool>(
        "/ground_station/is_autonomous", rclcpp::QoS(1).reliable()
    );

    spdlog::info("ROS2 telemetry subscription and config publisher created");
}

bool MainWindow::spinOnce()
{
    if (!m_initialized) return false;
    handleEvents();
    render();
    SDL_Delay(4);
    return m_running;
}

void MainWindow::shutdown()
{
    if (!m_initialized) return;

    stopTelemetry();

    m_telemSub.reset();
    m_configPub.reset();
    m_modeSub.reset();
    m_dpadConfigPub.reset();
    m_rosNode.reset();

    if (m_font12) { TTF_CloseFont(m_font12); m_font12 = nullptr; }
    if (m_font14) { TTF_CloseFont(m_font14); m_font14 = nullptr; }
    if (m_font16) { TTF_CloseFont(m_font16); m_font16 = nullptr; }
    if (m_font18) { TTF_CloseFont(m_font18); m_font18 = nullptr; }
    if (m_font22) { TTF_CloseFont(m_font22); m_font22 = nullptr; }

    if (m_renderer) { SDL_DestroyRenderer(m_renderer); m_renderer = nullptr; }
    if (m_window)   { SDL_DestroyWindow(m_window);     m_window   = nullptr; }

    TTF_Quit();
    SDL_Quit();
    m_running     = false;
    m_initialized = false;
    spdlog::info("TelemetryUI shutdown complete");
}

// -------------------------------------------------------
// ROS2 callbacks
// -------------------------------------------------------

void MainWindow::onTelemetryReceived(const std_msgs::msg::String::SharedPtr msg)
{
    auto j = json::parse(msg->data, nullptr, false);
    if (j.is_discarded() || !j.contains("motors")) return;

    std::lock_guard<std::mutex> lock(m_motorMutex);
    for (auto& mj : j["motors"]) {
        int idx = mj.value("config_index", -1);
        if (idx < 0 || idx >= NUM_MOTORS) continue;
        auto& m = m_motors[idx];
        m.config_index     = idx;
        m.motor_id         = mj.value("motor_id", (uint8_t)0);
        m.motor_name       = mj.value("motor_name", std::string(""));
        m.rpm              = mj.value("rpm", (int32_t)0);
        m.duty_cycle       = mj.value("duty_cycle", 0.0f);
        m.voltage          = mj.value("voltage", 0.0f);
        m.control_mode     = mj.value("control_mode", (uint8_t)0);
        m.inverted         = mj.value("inverted", false);
        m.enabled          = mj.value("enabled", true);
        m.current_in       = mj.value("current_in", 0.0f);
        m.current_motor    = mj.value("current_motor", 0.0f);
        m.position         = mj.value("position", 0.0f);
        m.fault_code       = mj.value("fault_code", (uint8_t)0);
        m.rpm_limit        = mj.value("rpm_limit", 0.0f);
        m.duty_cycle_limit = mj.value("duty_cycle_limit", 0.0f);
        m.received         = true;
    }
}

void MainWindow::publishConfigUpdate(int configIndex, bool applyEditEnabled)
{
    if (!m_configPub || configIndex < 0 || configIndex >= NUM_MOTORS) return;

    uint8_t     vesId;
    std::string vesName;
    bool        currentEnabled;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        vesId          = m_motors[configIndex].motor_id;
        vesName        = m_motors[configIndex].motor_name;
        currentEnabled = m_motors[configIndex].enabled;
    }

    robot_msgs::msg::MotorConfig msg;
    msg.config_index     = static_cast<uint8_t>(configIndex);
    msg.motor_vesc_id    = vesId;
    msg.motor_name       = vesName;
    msg.rpm_limit        = m_editRpmLimit;
    msg.duty_cycle_limit = m_editDutyCycleLimit;
    msg.control_mode     = m_editControlMode;
    msg.inverted         = m_editInverted;
    msg.enabled          = applyEditEnabled ? m_editEnabled : currentEnabled;

    m_configPub->publish(msg);
    spdlog::info("Published config for motor [{}] {}: rpm_limit={:.0f} duty={:.3f} mode={} inv={} enabled={}",
        configIndex, vesName, msg.rpm_limit, msg.duty_cycle_limit,
        msg.control_mode, msg.inverted ? "yes" : "no", msg.enabled ? "yes" : "no");
}

void MainWindow::publishConfigToSelection(bool applyEditEnabled)
{
    for (int idx : m_selectedMotors)
        publishConfigUpdate(idx, applyEditEnabled);
}

void MainWindow::publishDPadConfig()
{
    if (!m_dpadConfigPub) return;
    robot_msgs::msg::DPadConfig msg;
    msg.rpm_limit        = m_dpadRpmLimit;
    msg.duty_cycle_limit = m_dpadDutyLimit;
    m_dpadConfigPub->publish(msg);
    spdlog::info("Published DPad config: rpm={:.0f} duty={:.3f}", m_dpadRpmLimit, m_dpadDutyLimit);
}

bool MainWindow::getCommonEditValues(
    const std::array<MotorData, NUM_MOTORS>& motors,
    bool& rpmSame, bool& dutySame, bool& modeSame, bool& invSame) const
{
    if (m_selectedMotors.empty()) return false;

    float refRpm  = -1.0f;
    float refDuty = -1.0f;
    int   refMode = -1;
    int   refInv  = -1;
    rpmSame = dutySame = modeSame = invSame = true;

    for (int idx : m_selectedMotors) {
        if (idx < 0 || idx >= NUM_MOTORS || !motors[idx].received) continue;
        const auto& m = motors[idx];
        if (refRpm  < 0.0f)                      refRpm  = m.rpm_limit;
        else if (m.rpm_limit != refRpm)           rpmSame = false;

        if (refDuty < 0.0f)                       refDuty = m.duty_cycle_limit;
        else if (m.duty_cycle_limit != refDuty)   dutySame = false;

        if (refMode < 0)                          refMode = m.control_mode;
        else if (m.control_mode != refMode)       modeSame = false;

        if (refInv < 0)                           refInv = m.inverted ? 1 : 0;
        else if ((m.inverted ? 1 : 0) != refInv)  invSame = false;
    }
    return true;
}

// -------------------------------------------------------
// Event handling
// -------------------------------------------------------

void MainWindow::handleEvents()
{
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        switch (e.type) {
            case SDL_QUIT:
                m_running = false;
                break;
            case SDL_TEXTINPUT:
                handleTextInput(e.text.text);
                break;
            case SDL_KEYDOWN:
                if (m_focusedField != FOCUS_NONE) {
                    handleKeyDown(e.key.keysym.sym);
                } else if (e.key.keysym.sym == SDLK_ESCAPE) {
                    m_running = false;
                } else if (e.key.keysym.sym == SDLK_q) {
                    m_running = false;
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (e.button.button == SDL_BUTTON_LEFT) {
                    SDL_Keymod mod = SDL_GetModState();
                    handleMouseClick(e.button.x, e.button.y, (mod & KMOD_CTRL) != 0);
                }
                break;
            default:
                break;
        }
    }
}

void MainWindow::handleTextInput(const char* text)
{
    if (m_focusedField == FOCUS_NONE) return;
    std::string& buf = (m_focusedField == FOCUS_RPM)       ? m_rpmInputText
                     : (m_focusedField == FOCUS_DUTY)      ? m_dutyInputText
                     : (m_focusedField == FOCUS_DPAD_RPM)  ? m_dpadRpmText
                     :                                        m_dpadDutyText;
    // Only allow digits and decimal point
    for (const char* c = text; *c; ++c) {
        if ((*c >= '0' && *c <= '9') || *c == '.') {
            buf += *c;
        }
    }
}

void MainWindow::handleKeyDown(SDL_Keycode key)
{
    if (m_focusedField == FOCUS_NONE) return;
    std::string& buf = (m_focusedField == FOCUS_RPM)       ? m_rpmInputText
                     : (m_focusedField == FOCUS_DUTY)      ? m_dutyInputText
                     : (m_focusedField == FOCUS_DPAD_RPM)  ? m_dpadRpmText
                     :                                        m_dpadDutyText;

    if (key == SDLK_BACKSPACE && !buf.empty()) {
        buf.pop_back();
    } else if (key == SDLK_RETURN || key == SDLK_KP_ENTER) {
        // Commit the typed value and publish immediately
        if (!buf.empty()) {
            try {
                float val = std::stof(buf);
                if (m_focusedField == FOCUS_RPM) {
                    m_editRpmLimit = roundf(std::max(0.0f, val) * 100.0f) / 100.0f;
                    if (!m_selectedMotors.empty()) publishConfigToSelection(false);
                } else if (m_focusedField == FOCUS_DUTY) {
                    m_editDutyCycleLimit = roundf(std::clamp(val, 0.0f, 1.0f) * 100.0f) / 100.0f;
                    if (!m_selectedMotors.empty()) publishConfigToSelection(false);
                } else if (m_focusedField == FOCUS_DPAD_RPM) {
                    m_dpadRpmLimit = std::max(0.0f, val);
                    publishDPadConfig();
                } else if (m_focusedField == FOCUS_DPAD_DUTY) {
                    m_dpadDutyLimit = std::clamp(val, 0.0f, 1.0f);
                    publishDPadConfig();
                }
            } catch (...) {}
        }
        m_focusedField = FOCUS_NONE;
        SDL_StopTextInput();
    } else if (key == SDLK_ESCAPE) {
        // Cancel text input, revert without publishing
        m_focusedField = FOCUS_NONE;
        SDL_StopTextInput();
    } else if (key == SDLK_TAB) {
        // Commit current and move to next field (no publish on tab, wait for Apply/Enter)
        try {
            float val = std::stof(buf);
            if (m_focusedField == FOCUS_RPM) {
                m_editRpmLimit = roundf(std::max(0.0f, val) * 100.0f) / 100.0f;
                m_focusedField = FOCUS_DUTY;
                char dbuf[32];
                std::snprintf(dbuf, sizeof(dbuf), "%.2f", m_editDutyCycleLimit);
                m_dutyInputText = dbuf;
            } else {
                m_editDutyCycleLimit = roundf(std::clamp(val, 0.0f, 1.0f) * 100.0f) / 100.0f;
                m_focusedField = FOCUS_RPM;
                char rbuf[32];
                std::snprintf(rbuf, sizeof(rbuf), "%.2f", m_editRpmLimit);
                m_rpmInputText = rbuf;
            }
        } catch (...) {}
    }
}

void MainWindow::handleMouseClick(int mx, int my, bool ctrlHeld)
{
    // Helper: load edit state from the first selected motor
    auto loadEditFromSelection = [this]() {
        if (!m_selectedMotors.empty()) {
            int refIdx = *m_selectedMotors.begin();
            std::lock_guard<std::mutex> lock(m_motorMutex);
            const auto& m = m_motors[refIdx];
            m_editRpmLimit       = m.rpm_limit;
            m_editDutyCycleLimit = m.duty_cycle_limit;
            m_editControlMode    = m.control_mode;
            m_editInverted       = m.inverted;
            m_editEnabled        = m.enabled;
        }
    };

    // Check edit panel interactions (always visible)
    {
        // Check text box clicks
        bool clickedTextBox = false;
        for (const auto& tb : m_textBoxRects) {
            if (pointInRect(mx, my, tb.x, tb.y, tb.w, tb.h)) {
                m_focusedField = tb.field;
                SDL_StartTextInput();
                if (tb.field == FOCUS_DPAD_RPM) {
                    char buf[32];
                    std::snprintf(buf, sizeof(buf), "%.0f", m_dpadRpmLimit);
                    m_dpadRpmText = buf;
                } else if (tb.field == FOCUS_DPAD_DUTY) {
                    char buf[32];
                    std::snprintf(buf, sizeof(buf), "%.3f", m_dpadDutyLimit);
                    m_dpadDutyText = buf;
                } else if (!m_selectedMotors.empty()) {
                    // Initialize text buffer from first selected motor
                    int refIdx = *m_selectedMotors.begin();
                    std::lock_guard<std::mutex> lock(m_motorMutex);
                    if (tb.field == FOCUS_RPM) {
                        m_editRpmLimit = m_motors[refIdx].rpm_limit;
                        char buf[32];
                        std::snprintf(buf, sizeof(buf), "%.2f", m_editRpmLimit);
                        m_rpmInputText = buf;
                    } else {
                        m_editDutyCycleLimit = m_motors[refIdx].duty_cycle_limit;
                        char buf[32];
                        std::snprintf(buf, sizeof(buf), "%.2f", m_editDutyCycleLimit);
                        m_dutyInputText = buf;
                    }
                }
                clickedTextBox = true;
                return;
            }
        }

        // Check button clicks
        for (const auto& btn : m_editButtons) {
            if (pointInRect(mx, my, btn.x, btn.y, btn.w, btn.h)) {
                if (m_focusedField != FOCUS_NONE) {
                    m_focusedField = FOCUS_NONE;
                    SDL_StopTextInput();
                }
                switch (btn.action) {
                    case ACT_MODE_TOGGLE:
                        m_editControlMode = (m_editControlMode == 0) ? 1 : 0;
                        if (!m_selectedMotors.empty()) publishConfigToSelection(false);
                        return;
                    case ACT_INV_TOGGLE:
                        m_editInverted = !m_editInverted;
                        if (!m_selectedMotors.empty()) publishConfigToSelection(false);
                        return;
                    case ACT_ENABLED_TOGGLE:
                        m_editEnabled = !m_editEnabled;
                        if (!m_selectedMotors.empty()) publishConfigToSelection(true);
                        return;
                    case ACT_APPLY: {
                        if (!m_rpmInputText.empty()) {
                            try {
                                float val = std::stof(m_rpmInputText);
                                m_editRpmLimit = roundf(std::max(0.0f, val) * 100.0f) / 100.0f;
                            } catch (...) {}
                        } else if (!m_selectedMotors.empty()) {
                            std::lock_guard<std::mutex> lock(m_motorMutex);
                            m_editRpmLimit = m_motors[*m_selectedMotors.begin()].rpm_limit;
                        }
                        if (!m_dutyInputText.empty()) {
                            try {
                                float val = std::stof(m_dutyInputText);
                                m_editDutyCycleLimit = roundf(std::clamp(val, 0.0f, 1.0f) * 100.0f) / 100.0f;
                            } catch (...) {}
                        } else if (!m_selectedMotors.empty()) {
                            std::lock_guard<std::mutex> lock(m_motorMutex);
                            m_editDutyCycleLimit = m_motors[*m_selectedMotors.begin()].duty_cycle_limit;
                        }
                        if (!m_selectedMotors.empty()) publishConfigToSelection(true);
                        m_focusedField = FOCUS_NONE;
                        SDL_StopTextInput();
                        return;
                    }
                }
            }
        }

        // Clicked outside text boxes — unfocus without publishing
        if (!clickedTextBox && m_focusedField != FOCUS_NONE) {
            m_focusedField = FOCUS_NONE;
            SDL_StopTextInput();
        }
    }

    // Check motor card clicks
    // Check footer buttons (autonomy) — they live in the footer area
    for (const auto& fb : m_footerButtons) {
        if (pointInRect(mx, my, fb.x, fb.y, fb.w, fb.h)) {
            if (m_focusedField != FOCUS_NONE) {
                m_focusedField = FOCUS_NONE;
                SDL_StopTextInput();
            }
            if (fb.action == ACT_SET_AUTO) {
                m_isAutonomous.store(true);
                if (m_autonomyPub) {
                    std_msgs::msg::Bool msg;
                    msg.data = true;
                    m_autonomyPub->publish(msg);
                }
                spdlog::info("Set is_autonomous = true (Auto button)");
            } else if (fb.action == ACT_SET_TELEOP) {
                m_isAutonomous.store(false);
                if (m_autonomyPub) {
                    std_msgs::msg::Bool msg;
                    msg.data = false;
                    m_autonomyPub->publish(msg);
                }
                spdlog::info("Set is_autonomous = false (Teleop button)");
            }
            return;
        }
    }
    for (const auto& card : m_cardRects) {
        if (card.configIndex < 0) continue;
        if (pointInRect(mx, my, card.x, card.y, card.w, card.h)) {
            if (ctrlHeld) {
                // Ctrl+Click: toggle this motor in/out of selection
                if (m_selectedMotors.count(card.configIndex))
                    m_selectedMotors.erase(card.configIndex);
                else
                    m_selectedMotors.insert(card.configIndex);
            } else {
                // Plain click: single-select, or deselect if it's the only one
                if (m_selectedMotors.size() == 1 && m_selectedMotors.count(card.configIndex))
                    m_selectedMotors.clear();
                else {
                    m_selectedMotors.clear();
                    m_selectedMotors.insert(card.configIndex);
                }
            }
            loadEditFromSelection();
            m_focusedField = FOCUS_NONE;
            SDL_StopTextInput();
            return;
        }
    }

    // Check group card clicks
    for (const auto& gc : m_groupCardRects) {
        if (pointInRect(mx, my, gc.x, gc.y, gc.w, gc.h)) {
            const auto& group = MOTOR_GROUPS[gc.groupIndex];
            bool allSelected = true;
            for (int idx : group.configIndices)
                if (!m_selectedMotors.count(idx)) { allSelected = false; break; }
            if (allSelected) {
                for (int idx : group.configIndices) m_selectedMotors.erase(idx);
            } else {
                for (int idx : group.configIndices) m_selectedMotors.insert(idx);
            }
            loadEditFromSelection();
            m_focusedField = FOCUS_NONE;
            SDL_StopTextInput();
            return;
        }
    }
}

// -------------------------------------------------------
// Rendering
// -------------------------------------------------------

void MainWindow::render()
{
    int winW, winH;
    SDL_GetWindowSize(m_window, &winW, &winH);

    SDL_SetRenderDrawColor(m_renderer,
        Colors::BACKGROUND.r, Colors::BACKGROUND.g,
        Colors::BACKGROUND.b, Colors::BACKGROUND.a);
    SDL_RenderClear(m_renderer);

    renderMainArea(winW, winH);
    renderFooter(winW, winH);

    SDL_RenderPresent(m_renderer);
}

void MainWindow::renderMainArea(int winW, int winH)
{
    int mainH = winH - FOOTER_H;
    if (mainH <= 0) return;

    int sidebarW = SIDEBAR_W;
    int gridW = winW - sidebarW;
    if (gridW < 200) { gridW = winW; sidebarW = 0; }

    const int editH  = EDIT_PANEL_H;
    const int groupH = GROUP_ROW_H;
    int gridH        = mainH - editH - groupH;
    if (gridH < 100) gridH = 100;
    int gridOriginY  = groupH;

    // Copy motor data under lock
    std::array<MotorData, NUM_MOTORS> localMotors;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        localMotors = m_motors;
    }

    // Render group cards row
    m_groupCardRects.clear();
    {
        int numG = static_cast<int>(MOTOR_GROUPS.size());
        int gcW  = (gridW - CARD_PAD * (numG + 1)) / numG;
        int gcH  = groupH - CARD_PAD * 2;
        for (int g = 0; g < numG; ++g) {
            int gx = CARD_PAD + g * (gcW + CARD_PAD);
            int gy = CARD_PAD;
            int selCount = 0;
            for (int idx : MOTOR_GROUPS[g].configIndices)
                if (m_selectedMotors.count(idx)) ++selCount;
            int total = static_cast<int>(MOTOR_GROUPS[g].configIndices.size());
            int state = (selCount == 0) ? 0 : (selCount == total) ? 2 : 1;
            renderGroupCard(gx, gy, gcW, gcH, MOTOR_GROUPS[g], localMotors, state);
            m_groupCardRects.push_back({gx, gy, gcW, gcH, g});
        }
    }

    // Collect only active (received) motors
    std::vector<int> activeIndices;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (localMotors[i].received) {
            activeIndices.push_back(i);
        }
    }

    // Render grid background (below group row)
    renderMotorGrid(0, gridOriginY, gridW, gridH);

    // Dynamic grid: compute cols/rows based on active motor count
    m_cardRects.clear();
    int activeCount = static_cast<int>(activeIndices.size());

    if (activeCount > 0) {
        // Compute grid dimensions: cols = ceil(sqrt(n)), rows = ceil(n/cols)
        int cols = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(activeCount))));
        if (cols < 1) cols = 1;
        int rows = (activeCount + cols - 1) / cols;

        int cardW = (gridW - CARD_PAD * (cols + 1)) / cols;
        int cardH = (gridH - CARD_PAD * (rows + 1)) / rows;
        if (cardW < 80) cardW = 80;
        if (cardH < 80) cardH = 80;

        for (int i = 0; i < activeCount; ++i) {
            int configIdx = activeIndices[i];
            int col = i % cols;
            int row = i / cols;
            int cx = CARD_PAD + col * (cardW + CARD_PAD);
            int cy = gridOriginY + CARD_PAD + row * (cardH + CARD_PAD);

            bool selected = m_selectedMotors.count(configIdx) > 0;
            renderMotorCard(cx, cy, cardW, cardH, localMotors[configIdx], selected);

            m_cardRects.push_back({cx, cy, cardW, cardH, configIdx});
        }
    } else {
        // No motors connected yet
        drawText(CARD_PAD + 10, gridOriginY + CARD_PAD + 10,
                 "Waiting for motor telemetry...", Colors::STAT_LABEL, m_font14);
    }

    // Remove stale selections (motors that lost telemetry)
    {
        std::vector<int> toErase;
        for (int idx : m_selectedMotors)
            if (idx < 0 || idx >= NUM_MOTORS || !localMotors[idx].received)
                toErase.push_back(idx);
        for (int idx : toErase) m_selectedMotors.erase(idx);
    }

    // Edit panel always visible at bottom
    renderEditPanel(0, gridOriginY + gridH, gridW, editH);

    // Render sidebar
    if (sidebarW > 0) {
        renderSidebar(gridW, 0, sidebarW, mainH);
    }
}

void MainWindow::renderMotorGrid(int x, int y, int w, int h)
{
    drawFilledRect(x, y, w, h, Colors::BACKGROUND);
}

void MainWindow::renderGroupCard(int x, int y, int w, int h,
                                  const GroupDef& group,
                                  const std::array<MotorData, NUM_MOTORS>& motors,
                                  int selectionState)
{
    // Background
    SDL_Color bg = (selectionState == 2) ? Colors::CARD_SELECT : Colors::CARD_BG;
    drawFilledRect(x, y, w, h, bg);

    // Border
    bool anyFault = false;
    float sumCurrent = 0.0f;
    float sumVoltage = 0.0f;
    int   voltCount  = 0;
    for (int idx : group.configIndices) {
        if (idx < 0 || idx >= NUM_MOTORS || !motors[idx].received) continue;
        sumCurrent += motors[idx].current_in;
        sumVoltage += motors[idx].voltage;
        voltCount++;
        if (motors[idx].fault_code != 0) anyFault = true;
    }
    float avgVoltage = (voltCount > 0) ? (sumVoltage / voltCount) : 0.0f;

    if (anyFault) {
        bool blinkOn = ((SDL_GetTicks() / 500) % 2) == 0;
        drawBorderRect(x, y, w, h, blinkOn ? Colors::STAT_CRIT : Colors::CARD_BG);
    } else if (selectionState == 2) {
        drawBorderRect(x, y, w, h, Colors::ACCENT_BLUE);
    } else if (selectionState == 1) {
        SDL_Color partialBorder = {
            static_cast<Uint8>((Colors::ACCENT_BLUE.r + Colors::BORDER.r) / 2),
            static_cast<Uint8>((Colors::ACCENT_BLUE.g + Colors::BORDER.g) / 2),
            static_cast<Uint8>((Colors::ACCENT_BLUE.b + Colors::BORDER.b) / 2),
            255
        };
        drawBorderRect(x, y, w, h, partialBorder);
    } else {
        drawBorderRect(x, y, w, h, Colors::BORDER);
    }

    SDL_Rect clip = {x, y, w, h};
    SDL_RenderSetClipRect(m_renderer, &clip);

    char buf[64];
    int lx = x + 8;
    int ty = y + 6;

    // Group name (left column)
    drawText(lx, ty, group.name, Colors::TEXT, m_font16);
    ty += 22;
    std::snprintf(buf, sizeof(buf), "%d motors", static_cast<int>(group.configIndices.size()));
    drawText(lx, ty, buf, Colors::STAT_LABEL, m_font12);

    // Aggregate stats (right column)
    int rx  = x + w - 130;
    int rty = y + 6;

    const char* faultLabel = anyFault ? "FAULT" : "OK";
    drawText(rx, rty, faultLabel, anyFault ? Colors::STAT_CRIT : Colors::STAT_GOOD, m_font12);
    rty += 18;

    std::snprintf(buf, sizeof(buf), "I: %.2fA", sumCurrent);
    drawText(rx, rty, buf, Colors::TEXT, m_font12);
    rty += 16;

    std::snprintf(buf, sizeof(buf), "V: %.2fV", avgVoltage);
    SDL_Color vCol = (avgVoltage > 20.0f) ? Colors::STAT_GOOD :
                     (avgVoltage > 15.0f) ? Colors::STAT_WARN : Colors::STAT_CRIT;
    drawText(rx, rty, buf, vCol, m_font12);

    SDL_RenderSetClipRect(m_renderer, nullptr);
}

void MainWindow::renderMotorCard(int x, int y, int w, int h,
                                  const MotorData& motor, bool selected)
{
    SDL_Color bg = selected ? Colors::CARD_SELECT : Colors::CARD_BG;
    drawFilledRect(x, y, w, h, bg);

    bool hasFault = (motor.fault_code != 0);
    if (hasFault) {
        // Blink at ~1 s cycle (500 ms on, 500 ms off)
        bool blinkOn = ((SDL_GetTicks() / 500) % 2) == 0;
        SDL_Color borderCol = blinkOn ? Colors::STAT_CRIT : Colors::CARD_BG;
        drawBorderRect(x, y, w, h, borderCol);
    } else if (selected) {
        drawBorderRect(x, y, w, h, Colors::ACCENT_BLUE);
    } else {
        drawBorderRect(x, y, w, h, Colors::BORDER);
    }

    // Clip all content to card bounds to prevent overflow
    SDL_Rect clipR = {x, y, w, h};
    SDL_RenderSetClipRect(m_renderer, &clipR);

    if (!motor.received) {
        drawText(x + 10, y + 10, "No Data", Colors::STAT_LABEL, m_font14);
        SDL_RenderSetClipRect(m_renderer, nullptr);
        return;
    }

    int ty = y + 8;
    int lx = x + 10;
    int maxTextW = w - 20;
    char buf[128];

    // Motor name — wrapped to card width
    ty += drawTextWrapped(lx, ty, maxTextW, motor.motor_name, Colors::TEXT, m_font16) + 4;

    // Fault code — always visible under motor name
    {
        const char* faultStr = "NO FAULT";
        SDL_Color faultColor = Colors::STAT_GOOD;
        if (motor.fault_code != 0) {
            faultColor = Colors::STAT_CRIT;
            switch (motor.fault_code) {
                case 1: faultStr = "OVER_VOLTAGE";     break;
                case 2: faultStr = "UNDER_VOLTAGE";    break;
                case 3: faultStr = "DRV";              break;
                case 4: faultStr = "ABS_OVER_CURRENT"; break;
                case 5: faultStr = "OVER_TEMP_FET";    break;
                case 6: faultStr = "OVER_TEMP_MOTOR";  break;
                default: faultStr = "UNKNOWN";
            }
        }
        drawText(lx, ty, faultStr, faultColor, m_font12);
        ty += 18;
    }

    // ID
    std::snprintf(buf, sizeof(buf), "ID: %u", motor.motor_id);
    drawText(lx, ty, buf, Colors::STAT_LABEL, m_font12);
    ty += 20;

    // RPM bar (hardware limit: 0–30000)
    int barW = w - 20;
    std::snprintf(buf, sizeof(buf), "RPM: %d", motor.rpm);
    drawText(lx, ty, buf, Colors::TEXT, m_font14);
    ty += 18;
    {
        float pct   = std::min(1.0f, std::abs(static_cast<float>(motor.rpm)) / 30000.0f);
        int   fillW = static_cast<int>(pct * barW);
        SDL_Color barCol = (motor.rpm >= 0) ? Colors::STAT_GOOD : Colors::STAT_CRIT;
        drawFilledRect(lx, ty, fillW, 7, barCol);
        drawFilledRect(lx + fillW, ty, barW - fillW, 7, Colors::BORDER);
        ty += 11;
    }

    // Duty cycle bar (hardware limit: 0–1.0)
    std::snprintf(buf, sizeof(buf), "Duty: %.3f", motor.duty_cycle);
    drawText(lx, ty, buf, Colors::TEXT, m_font14);
    ty += 18;
    {
        float pct   = std::min(1.0f, std::abs(motor.duty_cycle));
        int   fillW = static_cast<int>(pct * barW);
        SDL_Color barCol = (motor.duty_cycle >= 0.0f) ? Colors::STAT_GOOD : Colors::STAT_CRIT;
        drawFilledRect(lx, ty, fillW, 7, barCol);
        drawFilledRect(lx + fillW, ty, barW - fillW, 7, Colors::BORDER);
        ty += 11;
    }

    // Control mode badge
    const char* modeStr = (motor.control_mode == 0) ? "RPM" : "DUTY";
    SDL_Color modeBg = (motor.control_mode == 0) ? Colors::STAT_GOOD : Colors::STAT_WARN;
    int badgeW = 64, badgeH = 22;
    drawFilledRect(lx, ty, badgeW, badgeH, modeBg);
    drawText(lx + 6, ty + 3, modeStr, Colors::TEXT_DARK, m_font14);
    ty += 28;

    // Inverted badge
    const char* invStr = motor.inverted ? "INVERTED" : "NORMAL";
    SDL_Color invBg = motor.inverted ? Colors::STAT_WARN : Colors::STAT_GOOD;
    int invBadgeW = 88;
    drawFilledRect(lx, ty, invBadgeW, badgeH, invBg);
    drawText(lx + 6, ty + 3, invStr, Colors::TEXT_DARK, m_font14);
    ty += 28;

    // Enabled badge
    const char* enStr = motor.enabled ? "ENABLED" : "DISABLED";
    SDL_Color enBg = motor.enabled ? Colors::STAT_GOOD : Colors::STAT_CRIT;
    int enBadgeW = 88;
    drawFilledRect(lx, ty, enBadgeW, badgeH, enBg);
    drawText(lx + 6, ty + 3, enStr, Colors::TEXT_DARK, m_font14);
    ty += 28;

    // Current draw (side by side)
    std::snprintf(buf, sizeof(buf), "I_in: %.2fA", motor.current_in);
    drawText(lx, ty, buf, Colors::TEXT, m_font12);
    std::snprintf(buf, sizeof(buf), "I_mot: %.2fA", motor.current_motor);
    drawText(lx + w / 2 - 10, ty, buf, Colors::TEXT, m_font12);
    ty += 18;

    // Position
    std::snprintf(buf, sizeof(buf), "Pos: %.2f deg", motor.position);
    drawText(lx, ty, buf, Colors::TEXT, m_font12);
    ty += 18;

    // Configured limits
    std::snprintf(buf, sizeof(buf), "Lim: %.2f RPM / %.2f duty", motor.rpm_limit, motor.duty_cycle_limit);
    drawText(lx, ty, buf, Colors::STAT_LABEL, m_font12);

    SDL_RenderSetClipRect(m_renderer, nullptr);
}

void MainWindow::renderSidebar(int x, int y, int w, int h)
{
    drawFilledRect(x, y, w, h, Colors::SIDEBAR_BG);

    // Vertical border
    SDL_SetRenderDrawColor(m_renderer,
        Colors::BORDER.r, Colors::BORDER.g,
        Colors::BORDER.b, Colors::BORDER.a);
    SDL_RenderDrawLine(m_renderer, x, y, x, y + h);

    // Clip all sidebar content to sidebar bounds
    SDL_Rect sideClip = {x, y, w, h};
    SDL_RenderSetClipRect(m_renderer, &sideClip);

    int px = x + 10;
    int py = y + 10;

    // --- Mode banner ---
    {
        uint8_t curMode  = m_currentMode.load(std::memory_order_relaxed);
        const char* ml   = (curMode == 0) ? "MOVEMENT" : "ARM";
        SDL_Color modeCol = (curMode == 0) ? Colors::ACCENT_BLUE : Colors::STAT_WARN;
        drawFilledRect(x, y, w, 46, Colors::CARD_BG);
        int tw = 0, th = 0;
        TTF_SizeText(m_font22, ml, &tw, &th);
        drawText(x + (w - tw) / 2, y + (46 - th) / 2, ml, modeCol, m_font22);
        SDL_SetRenderDrawColor(m_renderer,
            Colors::BORDER.r, Colors::BORDER.g,
            Colors::BORDER.b, Colors::BORDER.a);
        SDL_RenderDrawLine(m_renderer, px, y + 48, x + w - 10, y + 48);
        py = y + 56;
    }

    drawText(px, py, "SYSTEM POWER", Colors::TEXT, m_font16);
    py += 28;

    // Compute total estimated power and per-motor data
    std::array<MotorData, NUM_MOTORS> localMotors;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        localMotors = m_motors;
    }

    float totalCurrent = 0.0f;
    float avgVoltage   = 0.0f;
    int   voltageCount = 0;

    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (!localMotors[i].received) continue;
        totalCurrent += localMotors[i].current_in;
        avgVoltage   += localMotors[i].voltage;
        voltageCount++;
    }
    if (voltageCount > 0) avgVoltage /= voltageCount;
    float totalPower = avgVoltage * totalCurrent;

    // Total power display
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f W", totalPower);
    drawText(px, py, buf, Colors::ACCENT_BLUE, m_font22);
    py += 34;

    // Average bus voltage
    SDL_SetRenderDrawColor(m_renderer,
        Colors::BORDER.r, Colors::BORDER.g,
        Colors::BORDER.b, Colors::BORDER.a);
    SDL_RenderDrawLine(m_renderer, px, py, x + w - 10, py);
    py += 10;

    drawText(px, py, "BUS VOLTAGE", Colors::TEXT, m_font14);
    py += 22;

    std::snprintf(buf, sizeof(buf), "Avg: %.2f V", avgVoltage);
    SDL_Color vColor = (avgVoltage > 20.0f) ? Colors::STAT_GOOD :
                       (avgVoltage > 15.0f) ? Colors::STAT_WARN : Colors::STAT_CRIT;
    drawText(px, py, buf, vColor, m_font16);
    py += 28;

    // Separator
    SDL_RenderDrawLine(m_renderer, px, py, x + w - 10, py);
    py += 10;

    drawText(px, py, "PER MOTOR", Colors::TEXT, m_font14);
    py += 20;

    // Per-motor grouped bars: voltage + current per motor
    // barMaxW leaves room for "XX.XA" / "XX.XV" label (170px bar + ~70px label = 240px < 260px sidebar)
    int barMaxW = w - 90;
    float maxVolt    = 30.0f;
    float maxCurrent = 20.0f;

    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (py + 50 > y + h) break;
        const auto& m = localMotors[i];
        if (!m.received) continue;

        // Motor name header
        drawText(px, py, m.motor_name, Colors::TEXT, m_font12);
        py += 14;

        // Voltage bar
        float vPct  = std::min(1.0f, m.voltage / maxVolt);
        int   vBarW = static_cast<int>(vPct * barMaxW);
        SDL_Color vCol = (m.voltage > 20.0f) ? Colors::STAT_GOOD :
                         (m.voltage > 15.0f) ? Colors::STAT_WARN : Colors::STAT_CRIT;
        drawFilledRect(px, py, vBarW, 7, vCol);
        drawFilledRect(px + vBarW, py, barMaxW - vBarW, 7, Colors::BORDER);
        std::snprintf(buf, sizeof(buf), "%.1fV", m.voltage);
        drawText(px + barMaxW + 4, py - 1, buf, Colors::STAT_LABEL, m_font12);
        py += 11;

        // Current bar
        float cPct  = std::min(1.0f, m.current_in / maxCurrent);
        int   cBarW = static_cast<int>(cPct * barMaxW);
        SDL_Color cCol = (m.current_in < 5.0f) ? Colors::STAT_GOOD :
                         (m.current_in < 12.0f) ? Colors::STAT_WARN : Colors::STAT_CRIT;
        drawFilledRect(px, py, cBarW, 7, cCol);
        drawFilledRect(px + cBarW, py, barMaxW - cBarW, 7, Colors::BORDER);
        std::snprintf(buf, sizeof(buf), "%.1fA", m.current_in);
        drawText(px + barMaxW + 4, py - 1, buf, Colors::STAT_LABEL, m_font12);
        py += 14;
    }

    // --- D-pad limits section ---
    if (py + 90 < y + h) {
        SDL_SetRenderDrawColor(m_renderer,
            Colors::BORDER.r, Colors::BORDER.g,
            Colors::BORDER.b, Colors::BORDER.a);
        SDL_RenderDrawLine(m_renderer, px, py, x + w - 10, py);
        py += 8;
        drawText(px, py, "D-PAD LIMITS", Colors::TEXT, m_font14);
        py += 22;

        int tbX = px + 55, tbW = w - 65, tbH = 22;

        drawText(px, py + 4, "RPM:", Colors::STAT_LABEL, m_font12);
        char rpmbuf[32];
        std::snprintf(rpmbuf, sizeof(rpmbuf), "%.0f", m_dpadRpmLimit);
        std::string rpmDisp = (m_focusedField == FOCUS_DPAD_RPM) ? m_dpadRpmText : std::string(rpmbuf);
        drawTextBox(tbX, py, tbW, tbH, rpmDisp, m_focusedField == FOCUS_DPAD_RPM, m_font12);
        m_textBoxRects.push_back({tbX, py, tbW, tbH, FOCUS_DPAD_RPM});
        py += 28;

        drawText(px, py + 4, "Duty:", Colors::STAT_LABEL, m_font12);
        char dutybuf[32];
        std::snprintf(dutybuf, sizeof(dutybuf), "%.3f", m_dpadDutyLimit);
        std::string dutyDisp = (m_focusedField == FOCUS_DPAD_DUTY) ? m_dpadDutyText : std::string(dutybuf);
        drawTextBox(tbX, py, tbW, tbH, dutyDisp, m_focusedField == FOCUS_DPAD_DUTY, m_font12);
        m_textBoxRects.push_back({tbX, py, tbW, tbH, FOCUS_DPAD_DUTY});
    }

    SDL_RenderSetClipRect(m_renderer, nullptr);
}

void MainWindow::renderEditPanel(int x, int y, int w, int h)
{
    m_editButtons.clear();
    m_textBoxRects.clear();

    drawFilledRect(x, y, w, h, Colors::FOOTER_BG);

    // Top border
    SDL_SetRenderDrawColor(m_renderer,
        Colors::ACCENT_BLUE.r, Colors::ACCENT_BLUE.g,
        Colors::ACCENT_BLUE.b, Colors::ACCENT_BLUE.a);
    SDL_RenderDrawLine(m_renderer, x, y, x + w, y);

    // Clip panel content to panel bounds
    SDL_Rect panelClip = {x, y, w, h};
    SDL_RenderSetClipRect(m_renderer, &panelClip);

    int px = x + 15;
    int py = y + 10;

    if (m_selectedMotors.empty()) {
        // No motor selected — show hint
        drawText(px, py + (h - 20) / 2,
                 "Click a motor card to configure it",
                 Colors::STAT_LABEL, m_font14);
        SDL_RenderSetClipRect(m_renderer, nullptr);
        return;
    }

    // Read motor data snapshot for display
    std::array<MotorData, NUM_MOTORS> localMotors;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        localMotors = m_motors;
    }

    bool rpmSame, dutySame, modeSame, invSame;
    getCommonEditValues(localMotors, rpmSame, dutySame, modeSame, invSame);

    // Header line
    char buf[128];
    if (m_selectedMotors.size() == 1) {
        int idx = *m_selectedMotors.begin();
        std::snprintf(buf, sizeof(buf), "CONFIGURE: %s [%d]",
                      localMotors[idx].motor_name.c_str(), idx);
    } else {
        std::snprintf(buf, sizeof(buf), "CONFIGURE: %zu motors selected",
                      m_selectedMotors.size());
    }
    drawText(px, py, buf, Colors::TEXT, m_font16);
    py += 30;

    // Sync control state when not actively typing (only when values agree)
    if (m_focusedField == FOCUS_NONE && !m_selectedMotors.empty()) {
        int refIdx = *m_selectedMotors.begin();
        if (modeSame) m_editControlMode = localMotors[refIdx].control_mode;
        if (invSame)  m_editInverted    = localMotors[refIdx].inverted;
        m_editEnabled = localMotors[refIdx].enabled;
    }

    // Reference values for display
    float liveRpmLimit  = 0.0f;
    float liveDutyLimit = 0.0f;
    if (!m_selectedMotors.empty()) {
        int refIdx = *m_selectedMotors.begin();
        liveRpmLimit  = localMotors[refIdx].rpm_limit;
        liveDutyLimit = localMotors[refIdx].duty_cycle_limit;
    }

    int btnH    = 26;
    int tbW     = 110;
    int spacing = 10;
    int col     = px;

    // RPM Limit — label + text box
    drawText(col, py + 4, "RPM Limit:", Colors::STAT_LABEL, m_font14);
    col += 120;

    std::string rpmDisplay;
    if (m_focusedField == FOCUS_RPM) {
        rpmDisplay = m_rpmInputText;
    } else if (rpmSame) {
        std::snprintf(buf, sizeof(buf), "%.2f", liveRpmLimit);
        rpmDisplay = buf;
    }  // else empty — mixed values
    drawTextBox(col, py, tbW, btnH, rpmDisplay, m_focusedField == FOCUS_RPM, m_font14);
    m_textBoxRects.push_back({col, py, tbW, btnH, FOCUS_RPM});
    col += tbW + spacing * 3;

    // Duty Cycle Limit — label + text box
    drawText(col, py + 4, "Duty Limit:", Colors::STAT_LABEL, m_font14);
    col += 125;

    std::string dutyDisplay;
    if (m_focusedField == FOCUS_DUTY) {
        dutyDisplay = m_dutyInputText;
    } else if (dutySame) {
        std::snprintf(buf, sizeof(buf), "%.2f", liveDutyLimit);
        dutyDisplay = buf;
    }  // else empty — mixed values
    drawTextBox(col, py, tbW, btnH, dutyDisplay, m_focusedField == FOCUS_DUTY, m_font14);
    m_textBoxRects.push_back({col, py, tbW, btnH, FOCUS_DUTY});
    col += tbW + spacing * 3;

    // Mode toggle (instant apply on click)
    drawText(col, py + 4, "Mode:", Colors::STAT_LABEL, m_font14);
    col += 72;
    const char* modeLabel;
    SDL_Color   modeBg;
    if (!modeSame) {
        modeLabel = "MIX";
        modeBg    = Colors::BUTTON_BG;
    } else {
        modeLabel = (m_editControlMode == 0) ? "RPM" : "DUTY";
        modeBg    = (m_editControlMode == 0) ? Colors::STAT_GOOD : Colors::STAT_WARN;
    }
    drawFilledRect(col, py, 70, btnH, modeBg);
    drawBorderRect(col, py, 70, btnH, Colors::BORDER);
    {
        int tw = 0, th = 0;
        TTF_SizeText(m_font14, modeLabel, &tw, &th);
        drawText(col + (70 - tw) / 2, py + (btnH - th) / 2, modeLabel, Colors::TEXT_DARK, m_font14);
    }
    m_editButtons.push_back({col, py, 70, btnH, ACT_MODE_TOGGLE});
    col += 70 + spacing * 2;

    // Inverted toggle (instant apply on click)
    drawText(col, py + 4, "Inv:", Colors::STAT_LABEL, m_font14);
    col += 50;
    const char* invLabel;
    SDL_Color   invBg;
    if (!invSame) {
        invLabel = "MIX";
        invBg    = Colors::BUTTON_BG;
    } else {
        invLabel = m_editInverted ? "YES" : "NO";
        invBg    = m_editInverted ? Colors::STAT_WARN : Colors::STAT_GOOD;
    }
    drawFilledRect(col, py, 56, btnH, invBg);
    drawBorderRect(col, py, 56, btnH, Colors::BORDER);
    {
        int tw = 0, th = 0;
        TTF_SizeText(m_font14, invLabel, &tw, &th);
        drawText(col + (56 - tw) / 2, py + (btnH - th) / 2, invLabel, Colors::TEXT_DARK, m_font14);
    }
    m_editButtons.push_back({col, py, 56, btnH, ACT_INV_TOGGLE});
    col += 56 + spacing * 2;

    // Enabled toggle (instant apply on click)
    drawText(col, py + 4, "En:", Colors::STAT_LABEL, m_font14);
    col += 44;
    const char* enLabel = m_editEnabled ? "ON" : "OFF";
    SDL_Color   enBg    = m_editEnabled ? Colors::STAT_GOOD : Colors::STAT_CRIT;
    drawFilledRect(col, py, 56, btnH, enBg);
    drawBorderRect(col, py, 56, btnH, Colors::BORDER);
    {
        int tw = 0, th = 0;
        TTF_SizeText(m_font14, enLabel, &tw, &th);
        drawText(col + (56 - tw) / 2, py + (btnH - th) / 2, enLabel, Colors::TEXT_DARK, m_font14);
    }
    m_editButtons.push_back({col, py, 56, btnH, ACT_ENABLED_TOGGLE});
    col += 56 + spacing * 3;

    // Apply button (applies text box values to all selected motors)
    drawButton(col, py, 80, btnH, "APPLY", Colors::ACCENT_BLUE, m_font14);
    m_editButtons.push_back({col, py, 80, btnH, ACT_APPLY});

    SDL_RenderSetClipRect(m_renderer, nullptr);
}

void MainWindow::renderFooter(int winW, int winH)
{
    m_footerButtons.clear();
    int footerY = winH - FOOTER_H;

    SDL_Rect rect = {0, footerY, winW, FOOTER_H};
    SDL_SetRenderDrawColor(m_renderer,
        Colors::FOOTER_BG.r, Colors::FOOTER_BG.g,
        Colors::FOOTER_BG.b, Colors::FOOTER_BG.a);
    SDL_RenderFillRect(m_renderer, &rect);

    SDL_SetRenderDrawColor(m_renderer,
        Colors::BORDER.r, Colors::BORDER.g,
        Colors::BORDER.b, Colors::BORDER.a);
    SDL_RenderDrawLine(m_renderer, 0, footerY, winW, footerY);

    const float cpu = m_cpu.load();
    const float ram = m_ram.load();
    const float gpu = m_gpu.load();
    const float vramUsed  = m_vramUsedMb.load();
    const float vramTotal = m_vramTotalMb.load();

    const int secW   = winW / 3;
    const int labelY = footerY + 8;
    const int valueY = footerY + 32;

    char buf[48];

    // CPU
    drawText(STAT_PAD, labelY,
             "CPU Usage", Colors::STAT_LABEL, m_font14);
    std::snprintf(buf, sizeof(buf), "%.1f%%", cpu);
    drawText(STAT_PAD, valueY,
             buf, getStatColor(cpu), m_font18);

    // RAM
    drawText(secW + STAT_PAD, labelY,
             "RAM Usage", Colors::STAT_LABEL, m_font14);
    std::snprintf(buf, sizeof(buf), "%.1f%%", ram);
    drawText(secW + STAT_PAD, valueY,
             buf, getStatColor(ram), m_font18);

    // GPU
    const int col3 = secW * 2 + STAT_PAD;
    if (vramTotal > 0.0f) {
        drawText(col3, labelY,
                 "GPU Core / VRAM", Colors::STAT_LABEL, m_font14);
        std::snprintf(buf, sizeof(buf), "%.0f%%  %.0f/%.0f MB",
                      gpu, vramUsed, vramTotal);
    } else {
        drawText(col3, labelY,
                 "GPU Usage", Colors::STAT_LABEL, m_font14);
        std::snprintf(buf, sizeof(buf), "%.0f%%", gpu);
    }
    drawText(col3, valueY, buf, getStatColor(gpu), m_font18);

    // --- Autonomy buttons (to the right) ---
    int btnW = 90;
    int btnH = 34;
    int spacing = 8;
    int rightX = winW - STAT_PAD - btnW;
    int btnY = footerY + (FOOTER_H - btnH) / 2;

    // Order: left = Teleop (false), right = Auto (true)
    int autoX   = rightX;
    int teleopX = rightX - btnW - spacing;

    // Auto button style reflects current state
    SDL_Color autoBg = m_isAutonomous.load() ? Colors::ACCENT_BLUE : Colors::BUTTON_BG;
    SDL_Color teleopBg = m_isAutonomous.load() ? Colors::BUTTON_BG : Colors::ACCENT_BLUE;

    drawButton(teleopX, btnY, btnW, btnH, "Teleop", teleopBg, m_font14);
    m_footerButtons.push_back({teleopX, btnY, btnW, btnH, ACT_SET_TELEOP});

    drawButton(autoX, btnY, btnW, btnH, "Auto", autoBg, m_font14);
    m_footerButtons.push_back({autoX, btnY, btnW, btnH, ACT_SET_AUTO});
}

// -------------------------------------------------------
// Drawing helpers
// -------------------------------------------------------

SDL_Color MainWindow::getStatColor(float pct) const
{
    if (pct < 50.0f) return Colors::STAT_GOOD;
    if (pct < 80.0f) return Colors::STAT_WARN;
    return Colors::STAT_CRIT;
}

void MainWindow::drawText(int x, int y, const std::string& text,
                          SDL_Color color, TTF_Font* fnt)
{
    if (!fnt || text.empty()) return;

    SDL_Surface* surf = TTF_RenderText_Blended(fnt, text.c_str(), color);
    if (!surf) return;

    SDL_Texture* tex = SDL_CreateTextureFromSurface(m_renderer, surf);
    if (tex) {
        SDL_Rect dst = {x, y, surf->w, surf->h};
        SDL_RenderCopy(m_renderer, tex, nullptr, &dst);
        SDL_DestroyTexture(tex);
    }
    SDL_FreeSurface(surf);
}

int MainWindow::drawTextWrapped(int x, int y, int maxW, const std::string& text,
                                SDL_Color color, TTF_Font* fnt)
{
    if (!fnt || text.empty() || maxW <= 0) return 0;

    SDL_Surface* surf = TTF_RenderText_Blended_Wrapped(
        fnt, text.c_str(), color, static_cast<Uint32>(maxW));
    if (!surf) return 0;

    SDL_Texture* tex = SDL_CreateTextureFromSurface(m_renderer, surf);
    if (tex) {
        SDL_Rect dst = {x, y, surf->w, surf->h};
        SDL_RenderCopy(m_renderer, tex, nullptr, &dst);
        SDL_DestroyTexture(tex);
    }
    int h = surf->h;
    SDL_FreeSurface(surf);
    return h;
}

void MainWindow::drawFilledRect(int x, int y, int w, int h, SDL_Color color)
{
    SDL_Rect r = {x, y, w, h};
    SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
    SDL_RenderFillRect(m_renderer, &r);
}

void MainWindow::drawBorderRect(int x, int y, int w, int h, SDL_Color color)
{
    SDL_Rect r = {x, y, w, h};
    SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawRect(m_renderer, &r);
}

bool MainWindow::pointInRect(int px, int py, int rx, int ry, int rw, int rh) const
{
    return px >= rx && px < rx + rw && py >= ry && py < ry + rh;
}

void MainWindow::drawButton(int x, int y, int w, int h,
                             const std::string& label, SDL_Color bg, TTF_Font* fnt)
{
    drawFilledRect(x, y, w, h, bg);
    drawBorderRect(x, y, w, h, Colors::BORDER);

    // Center text in button
    if (!fnt || label.empty()) return;
    int tw = 0, th = 0;
    TTF_SizeText(fnt, label.c_str(), &tw, &th);
    int tx = x + (w - tw) / 2;
    int ty = y + (h - th) / 2;
    drawText(tx, ty, label, Colors::TEXT, fnt);
}

void MainWindow::drawTextBox(int x, int y, int w, int h,
                              const std::string& text, bool focused, TTF_Font* fnt)
{
    SDL_Color bg = focused ? Colors::TEXTBOX_FOCUS : Colors::TEXTBOX_BG;
    drawFilledRect(x, y, w, h, bg);

    // Border — highlight when focused
    SDL_Color border = focused ? Colors::ACCENT_BLUE : Colors::BORDER;
    drawBorderRect(x, y, w, h, border);

    // Text content
    std::string display = text;
    if (focused) display += "|";  // cursor
    if (!display.empty()) {
        drawText(x + 6, y + (h - 16) / 2, display, Colors::TEXT, fnt);
    }
}

// -------------------------------------------------------
// Telemetry background thread (system stats)
// -------------------------------------------------------

void MainWindow::startTelemetry()
{
    if (m_telemetryRunning) return;
    m_telemetryRunning = true;
    m_telemetryThread  = std::thread(&MainWindow::telemetryLoop, this);
}

void MainWindow::stopTelemetry()
{
    m_telemetryRunning = false;
    if (m_telemetryThread.joinable())
        m_telemetryThread.join();
}

void MainWindow::telemetryLoop()
{
    using namespace std::chrono_literals;
    while (m_telemetryRunning) {
        m_cpu.store(sampleCpuPercent());
        m_ram.store(sampleRamPercent());
        sampleGpuStats();

        for (int i = 0; i < 10 && m_telemetryRunning; ++i)
            std::this_thread::sleep_for(100ms);
    }
}

float MainWindow::sampleCpuPercent()
{
    std::ifstream f("/proc/stat");
    if (!f.is_open()) return 0.0f;

    std::string line;
    std::getline(f, line);

    std::istringstream ss(line);
    std::string label;
    ss >> label;

    uint64_t user = 0, nice = 0, sys = 0, idle = 0,
             iowait = 0, irq = 0, softirq = 0, steal = 0;
    ss >> user >> nice >> sys >> idle >> iowait >> irq >> softirq >> steal;

    const uint64_t total  = user + nice + sys + idle + iowait + irq + softirq + steal;
    const uint64_t active = total - idle - iowait;

    float pct = 0.0f;
    if (m_hasPrevCpu) {
        const uint64_t dt = total  - m_prevTotalJiffies;
        const uint64_t da = active - m_prevActiveJiffies;
        if (dt > 0)
            pct = static_cast<float>(da) / static_cast<float>(dt) * 100.0f;
    }

    m_prevTotalJiffies  = total;
    m_prevActiveJiffies = active;
    m_hasPrevCpu        = true;
    return pct;
}

float MainWindow::sampleRamPercent()
{
    std::ifstream f("/proc/meminfo");
    if (!f.is_open()) return 0.0f;

    uint64_t memTotal     = 0;
    uint64_t memAvailable = 0;

    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("MemTotal:", 0) == 0)
            std::sscanf(line.c_str(), "MemTotal: %lu kB", &memTotal);
        else if (line.rfind("MemAvailable:", 0) == 0)
            std::sscanf(line.c_str(), "MemAvailable: %lu kB", &memAvailable);

        if (memTotal && memAvailable) break;
    }

    if (!memTotal) return 0.0f;
    return static_cast<float>(memTotal - memAvailable) /
           static_cast<float>(memTotal) * 100.0f;
}

void MainWindow::sampleGpuStats()
{
    const char* cmd =
        "timeout 3 nvidia-smi "
        "--query-gpu=utilization.gpu,memory.used,memory.total "
        "--format=csv,noheader,nounits -i 0";

    FILE* pipe = popen(cmd, "r");
    if (!pipe) return;

    char buf[256] = {};
    if (fgets(buf, sizeof(buf), pipe)) {
        std::istringstream ss(buf);
        std::string tok;
        float vals[3] = {0.0f, 0.0f, 0.0f};

        for (int i = 0; i < 3 && std::getline(ss, tok, ','); ++i) {
            const auto first = tok.find_first_not_of(" \t\n\r");
            const auto last  = tok.find_last_not_of(" \t\n\r");
            if (first == std::string::npos) continue;
            tok = tok.substr(first, last - first + 1);
            try { vals[i] = std::stof(tok); } catch (...) {}
        }

        m_gpu.store(vals[0]);
        m_vramUsedMb.store(vals[1]);
        m_vramTotalMb.store(vals[2]);
    }

    pclose(pipe);
}

} // namespace telemetry_ui
