#include "telemetry_ui_pkg/main_window.h"
#include "telemetry_ui_pkg/json.hpp"

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
    ACT_APPLY,
    ACT_CANCEL,
};

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
    m_font12 = TTF_OpenFont(fontPath, 12);
    m_font14 = TTF_OpenFont(fontPath, 14);
    m_font16 = TTF_OpenFont(fontPath, 16);
    m_font18 = TTF_OpenFont(fontPath, 18);
    m_font22 = TTF_OpenFont(fontPath, 22);

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
        m.current_in       = mj.value("current_in", 0.0f);
        m.current_motor    = mj.value("current_motor", 0.0f);
        m.position         = mj.value("position", 0.0f);
        m.fault_code       = mj.value("fault_code", (uint8_t)0);
        m.rpm_limit        = mj.value("rpm_limit", 0.0f);
        m.duty_cycle_limit = mj.value("duty_cycle_limit", 0.0f);
        m.received         = true;
    }
}

void MainWindow::publishConfigUpdate(int configIndex)
{
    if (!m_configPub || configIndex < 0 || configIndex >= NUM_MOTORS) return;

    robot_msgs::msg::MotorConfig msg;
    msg.config_index     = static_cast<uint8_t>(configIndex);
    msg.motor_vesc_id    = m_motors[configIndex].motor_id;
    msg.motor_name       = m_motors[configIndex].motor_name;
    msg.rpm_limit        = m_editRpmLimit;
    msg.duty_cycle_limit = m_editDutyCycleLimit;
    msg.control_mode     = m_editControlMode;
    msg.inverted         = m_editInverted;

    m_configPub->publish(msg);
    spdlog::info("Published config for motor [{}] {}: rpm_limit={:.0f} duty={:.3f} mode={} inv={}",
        configIndex, msg.motor_name, msg.rpm_limit, msg.duty_cycle_limit,
        msg.control_mode, msg.inverted ? "yes" : "no");
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
                    if (m_editPanelOpen) {
                        m_editPanelOpen = false;
                        m_selectedMotor = -1;
                    } else {
                        m_running = false;
                    }
                } else if (e.key.keysym.sym == SDLK_q) {
                    m_running = false;
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (e.button.button == SDL_BUTTON_LEFT) {
                    handleMouseClick(e.button.x, e.button.y);
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
    std::string& buf = (m_focusedField == FOCUS_RPM) ? m_rpmInputText : m_dutyInputText;
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
    std::string& buf = (m_focusedField == FOCUS_RPM) ? m_rpmInputText : m_dutyInputText;

    if (key == SDLK_BACKSPACE && !buf.empty()) {
        buf.pop_back();
    } else if (key == SDLK_RETURN || key == SDLK_KP_ENTER) {
        // Commit the typed value
        try {
            float val = std::stof(buf);
            if (m_focusedField == FOCUS_RPM) {
                m_editRpmLimit = std::max(0.0f, val);
            } else {
                m_editDutyCycleLimit = std::clamp(val, 0.0f, 1.0f);
            }
        } catch (...) {}
        m_focusedField = FOCUS_NONE;
        SDL_StopTextInput();
    } else if (key == SDLK_ESCAPE) {
        // Cancel text input, revert to current value
        m_focusedField = FOCUS_NONE;
        SDL_StopTextInput();
    } else if (key == SDLK_TAB) {
        // Commit current and move to next field
        try {
            float val = std::stof(buf);
            if (m_focusedField == FOCUS_RPM) {
                m_editRpmLimit = std::max(0.0f, val);
                m_focusedField = FOCUS_DUTY;
                char dbuf[32];
                std::snprintf(dbuf, sizeof(dbuf), "%.2f", m_editDutyCycleLimit);
                m_dutyInputText = dbuf;
            } else {
                m_editDutyCycleLimit = std::clamp(val, 0.0f, 1.0f);
                m_focusedField = FOCUS_RPM;
                char rbuf[32];
                std::snprintf(rbuf, sizeof(rbuf), "%.0f", m_editRpmLimit);
                m_rpmInputText = rbuf;
            }
        } catch (...) {}
    }
}

void MainWindow::handleMouseClick(int mx, int my)
{
    // Check edit panel interactions (if open)
    if (m_editPanelOpen) {
        // Check text box clicks
        bool clickedTextBox = false;
        for (const auto& tb : m_textBoxRects) {
            if (pointInRect(mx, my, tb.x, tb.y, tb.w, tb.h)) {
                m_focusedField = tb.field;
                SDL_StartTextInput();
                // Initialize text buffer with current value
                if (tb.field == FOCUS_RPM) {
                    char buf[32];
                    std::snprintf(buf, sizeof(buf), "%.0f", m_editRpmLimit);
                    m_rpmInputText = buf;
                } else {
                    char buf[32];
                    std::snprintf(buf, sizeof(buf), "%.2f", m_editDutyCycleLimit);
                    m_dutyInputText = buf;
                }
                clickedTextBox = true;
                return;
            }
        }

        // Check button clicks
        for (const auto& btn : m_editButtons) {
            if (pointInRect(mx, my, btn.x, btn.y, btn.w, btn.h)) {
                // Unfocus text if clicking a button
                if (m_focusedField != FOCUS_NONE) {
                    m_focusedField = FOCUS_NONE;
                    SDL_StopTextInput();
                }
                switch (btn.action) {
                    case ACT_MODE_TOGGLE:
                        m_editControlMode = (m_editControlMode == 0) ? 1 : 0;
                        return;
                    case ACT_INV_TOGGLE:
                        m_editInverted = !m_editInverted;
                        return;
                    case ACT_APPLY:
                        publishConfigUpdate(m_selectedMotor);
                        m_editPanelOpen = false;
                        m_selectedMotor = -1;
                        m_focusedField = FOCUS_NONE;
                        SDL_StopTextInput();
                        return;
                    case ACT_CANCEL:
                        m_editPanelOpen = false;
                        m_selectedMotor = -1;
                        m_focusedField = FOCUS_NONE;
                        SDL_StopTextInput();
                        return;
                }
            }
        }

        // Clicked outside text boxes in edit panel — unfocus
        if (!clickedTextBox && m_focusedField != FOCUS_NONE) {
            // Commit current value before unfocusing
            try {
                if (m_focusedField == FOCUS_RPM)
                    m_editRpmLimit = std::max(0.0f, std::stof(m_rpmInputText));
                else
                    m_editDutyCycleLimit = std::clamp(std::stof(m_dutyInputText), 0.0f, 1.0f);
            } catch (...) {}
            m_focusedField = FOCUS_NONE;
            SDL_StopTextInput();
        }
    }

    // Check motor card clicks
    for (const auto& card : m_cardRects) {
        if (card.configIndex < 0) continue;
        if (pointInRect(mx, my, card.x, card.y, card.w, card.h)) {
            if (m_selectedMotor == card.configIndex && m_editPanelOpen) {
                // Clicking same card toggles off
                m_editPanelOpen = false;
                m_selectedMotor = -1;
            } else {
                m_selectedMotor = card.configIndex;
                m_editPanelOpen = true;
                // Load current values as starting edit values
                std::lock_guard<std::mutex> lock(m_motorMutex);
                const auto& m = m_motors[card.configIndex];
                m_editRpmLimit       = m.rpm_limit;
                m_editDutyCycleLimit = m.duty_cycle_limit;
                m_editControlMode    = m.control_mode;
                m_editInverted       = m.inverted;
            }
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

    int editH = m_editPanelOpen ? EDIT_PANEL_H : 0;
    int gridH = mainH - editH;

    // Copy motor data under lock
    std::array<MotorData, NUM_MOTORS> localMotors;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        localMotors = m_motors;
    }

    // Collect only active (received) motors
    std::vector<int> activeIndices;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (localMotors[i].received) {
            activeIndices.push_back(i);
        }
    }

    // Render grid background
    renderMotorGrid(0, 0, gridW, gridH);

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
            int cy = CARD_PAD + row * (cardH + CARD_PAD);

            bool selected = (m_selectedMotor == configIdx);
            renderMotorCard(cx, cy, cardW, cardH, localMotors[configIdx], selected);

            m_cardRects.push_back({cx, cy, cardW, cardH, configIdx});
        }
    } else {
        // No motors connected yet
        drawText(CARD_PAD + 10, CARD_PAD + 10,
                 "Waiting for motor telemetry...", Colors::STAT_LABEL, m_font14);
    }

    // If selected motor disconnected, close edit panel
    if (m_editPanelOpen && m_selectedMotor >= 0) {
        if (!localMotors[m_selectedMotor].received) {
            m_editPanelOpen = false;
            m_selectedMotor = -1;
        }
    }

    // Render edit panel (if open)
    if (m_editPanelOpen && m_selectedMotor >= 0 && m_selectedMotor < NUM_MOTORS) {
        renderEditPanel(0, gridH, gridW, editH);
    }

    // Render sidebar
    if (sidebarW > 0) {
        renderSidebar(gridW, 0, sidebarW, mainH);
    }
}

void MainWindow::renderMotorGrid(int x, int y, int w, int h)
{
    drawFilledRect(x, y, w, h, Colors::BACKGROUND);
}

void MainWindow::renderMotorCard(int x, int y, int w, int h,
                                  const MotorData& motor, bool selected)
{
    SDL_Color bg = selected ? Colors::CARD_SELECT : Colors::CARD_BG;
    drawFilledRect(x, y, w, h, bg);

    if (selected) {
        drawBorderRect(x, y, w, h, Colors::ACCENT_BLUE);
    } else {
        drawBorderRect(x, y, w, h, Colors::BORDER);
    }

    if (!motor.received) {
        drawText(x + 10, y + 10, "No Data", Colors::STAT_LABEL, m_font14);
        return;
    }

    int ty = y + 8;
    int lx = x + 10;
    char buf[128];

    // Motor name — full name, no truncation (wraps visually via card size)
    drawText(lx, ty, motor.motor_name, Colors::TEXT, m_font16);
    ty += 24;

    // ID
    std::snprintf(buf, sizeof(buf), "ID: %u", motor.motor_id);
    drawText(lx, ty, buf, Colors::STAT_LABEL, m_font12);
    ty += 18;

    // RPM — color coded
    std::snprintf(buf, sizeof(buf), "RPM: %d", motor.rpm);
    float rpmPct = std::min(100.0f, std::abs(motor.rpm) / 50.0f);
    drawText(lx, ty, buf, getStatColor(rpmPct), m_font16);
    ty += 22;

    // Duty cycle
    std::snprintf(buf, sizeof(buf), "Duty: %.2f", motor.duty_cycle);
    drawText(lx, ty, buf, Colors::TEXT, m_font16);
    ty += 22;

    // Control mode — use dark text on bright badge
    const char* modeStr = (motor.control_mode == 0) ? "RPM" : "DUTY";
    SDL_Color modeBg = (motor.control_mode == 0) ? Colors::STAT_GOOD : Colors::STAT_WARN;
    int badgeW = 60, badgeH = 22;
    drawFilledRect(lx, ty, badgeW, badgeH, modeBg);
    drawText(lx + 6, ty + 2, modeStr, Colors::TEXT_DARK, m_font14);
    ty += 28;

    // Inverted — use dark text on bright badge
    const char* invStr = motor.inverted ? "INVERTED" : "NORMAL";
    SDL_Color invBg = motor.inverted ? Colors::STAT_WARN : Colors::STAT_GOOD;
    int invBadgeW = 80;
    drawFilledRect(lx, ty, invBadgeW, badgeH, invBg);
    drawText(lx + 6, ty + 2, invStr, Colors::TEXT_DARK, m_font14);
    ty += 28;

    // Current draw (side by side)
    std::snprintf(buf, sizeof(buf), "I_in: %.1fA", motor.current_in);
    drawText(lx, ty, buf, Colors::TEXT, m_font12);
    std::snprintf(buf, sizeof(buf), "I_mot: %.1fA", motor.current_motor);
    drawText(lx + w / 2 - 10, ty, buf, Colors::TEXT, m_font12);
    ty += 16;

    // Position
    std::snprintf(buf, sizeof(buf), "Pos: %.1f deg", motor.position);
    drawText(lx, ty, buf, Colors::TEXT, m_font12);
    ty += 16;

    // Configured limits
    std::snprintf(buf, sizeof(buf), "Lim: %.0f RPM / %.2f duty", motor.rpm_limit, motor.duty_cycle_limit);
    drawText(lx, ty, buf, Colors::STAT_LABEL, m_font12);
    ty += 16;

    // Fault code (only if non-zero)
    if (motor.fault_code != 0) {
        const char* faultStr = "UNKNOWN";
        switch (motor.fault_code) {
            case 1: faultStr = "OVER_VOLTAGE"; break;
            case 2: faultStr = "UNDER_VOLTAGE"; break;
            case 3: faultStr = "DRV"; break;
            case 4: faultStr = "ABS_OVER_CURRENT"; break;
            case 5: faultStr = "OVER_TEMP_FET"; break;
            case 6: faultStr = "OVER_TEMP_MOTOR"; break;
        }
        std::snprintf(buf, sizeof(buf), "FAULT: %s", faultStr);
        drawText(lx, ty, buf, Colors::STAT_CRIT, m_font12);
    }
}

void MainWindow::renderSidebar(int x, int y, int w, int h)
{
    drawFilledRect(x, y, w, h, Colors::SIDEBAR_BG);

    // Vertical border
    SDL_SetRenderDrawColor(m_renderer,
        Colors::BORDER.r, Colors::BORDER.g,
        Colors::BORDER.b, Colors::BORDER.a);
    SDL_RenderDrawLine(m_renderer, x, y, x, y + h);

    int px = x + 10;
    int py = y + 10;

    drawText(px, py, "SYSTEM POWER", Colors::TEXT, m_font16);
    py += 28;

    // Compute total estimated power and per-motor data
    std::array<MotorData, NUM_MOTORS> localMotors;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        localMotors = m_motors;
    }

    float totalPower = 0.0f;
    float avgVoltage = 0.0f;
    int voltageCount = 0;

    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (!localMotors[i].received) continue;
        float pwr = std::abs(localMotors[i].duty_cycle) * localMotors[i].voltage;
        totalPower += pwr;
        avgVoltage += localMotors[i].voltage;
        voltageCount++;
    }
    if (voltageCount > 0) avgVoltage /= voltageCount;

    // Total power display
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.1f W*", totalPower);
    drawText(px, py, buf, Colors::ACCENT_BLUE, m_font22);
    py += 30;

    drawText(px, py, "(duty x volt est.)", Colors::STAT_LABEL, m_font12);
    py += 22;

    // Average bus voltage
    SDL_SetRenderDrawColor(m_renderer,
        Colors::BORDER.r, Colors::BORDER.g,
        Colors::BORDER.b, Colors::BORDER.a);
    SDL_RenderDrawLine(m_renderer, px, py, x + w - 10, py);
    py += 10;

    drawText(px, py, "BUS VOLTAGE", Colors::TEXT, m_font14);
    py += 22;

    std::snprintf(buf, sizeof(buf), "Avg: %.1f V", avgVoltage);
    SDL_Color vColor = (avgVoltage > 20.0f) ? Colors::STAT_GOOD :
                       (avgVoltage > 15.0f) ? Colors::STAT_WARN : Colors::STAT_CRIT;
    drawText(px, py, buf, vColor, m_font16);
    py += 28;

    // Separator
    SDL_RenderDrawLine(m_renderer, px, py, x + w - 10, py);
    py += 10;

    drawText(px, py, "PER MOTOR", Colors::TEXT, m_font14);
    py += 22;

    // Per-motor voltage bars
    int barMaxW = w - 40;
    float maxVolt = 30.0f; // normalize to 30V scale

    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (py + 30 > y + h) break; // don't overflow sidebar

        const auto& m = localMotors[i];
        if (!m.received) continue;  // skip disconnected motors entirely

        // Full motor name — no truncation
        drawText(px, py, m.motor_name, Colors::TEXT, m_font12);
        py += 16;

        // Voltage bar
        float pct = std::min(1.0f, m.voltage / maxVolt);
        int barW = static_cast<int>(pct * barMaxW);
        SDL_Color barColor = (m.voltage > 20.0f) ? Colors::STAT_GOOD :
                             (m.voltage > 15.0f) ? Colors::STAT_WARN : Colors::STAT_CRIT;

        drawFilledRect(px, py, barW, 8, barColor);
        drawFilledRect(px + barW, py, barMaxW - barW, 8, Colors::BORDER);

        // Voltage value
        std::snprintf(buf, sizeof(buf), "%.1fV", m.voltage);
        drawText(px + barMaxW + 4, py - 2, buf, Colors::STAT_LABEL, m_font12);
        py += 16;
    }

    // Separator
    if (py + 60 < y + h) {
        SDL_RenderDrawLine(m_renderer, px, py, x + w - 10, py);
        py += 10;

        drawText(px, py, "CURRENT DRAW", Colors::TEXT, m_font14);
        py += 22;

        float totalCurrent = 0.0f;
        for (int i = 0; i < NUM_MOTORS; ++i) {
            if (localMotors[i].received)
                totalCurrent += localMotors[i].current_in;
        }

        std::snprintf(buf, sizeof(buf), "Total: %.1f A", totalCurrent);
        drawText(px, py, buf, Colors::ACCENT_BLUE, m_font16);
        py += 24;

        // Per-motor current bars
        float maxCurrent = 20.0f;
        for (int i = 0; i < NUM_MOTORS; ++i) {
            if (py + 30 > y + h) break;
            const auto& m = localMotors[i];
            if (!m.received) continue;

            drawText(px, py, m.motor_name, Colors::TEXT, m_font12);
            py += 16;

            float pct = std::min(1.0f, m.current_in / maxCurrent);
            int barW = static_cast<int>(pct * barMaxW);
            SDL_Color barColor = (m.current_in < 5.0f) ? Colors::STAT_GOOD :
                                 (m.current_in < 12.0f) ? Colors::STAT_WARN : Colors::STAT_CRIT;

            drawFilledRect(px, py, barW, 8, barColor);
            drawFilledRect(px + barW, py, barMaxW - barW, 8, Colors::BORDER);

            std::snprintf(buf, sizeof(buf), "%.1fA", m.current_in);
            drawText(px + barMaxW + 4, py - 2, buf, Colors::STAT_LABEL, m_font12);
            py += 16;
        }
    }
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

    if (m_selectedMotor < 0 || m_selectedMotor >= NUM_MOTORS) return;

    std::string motorName;
    {
        std::lock_guard<std::mutex> lock(m_motorMutex);
        motorName = m_motors[m_selectedMotor].motor_name;
    }

    int px = x + 15;
    int py = y + 12;

    char buf[128];
    std::snprintf(buf, sizeof(buf), "CONFIGURE: %s [%d]", motorName.c_str(), m_selectedMotor);
    drawText(px, py, buf, Colors::TEXT, m_font16);
    py += 28;

    int btnH = 26;
    int tbW = 100;  // text box width
    int spacing = 10;
    int col = px;

    // RPM Limit — label + text box
    drawText(col, py + 3, "RPM Limit:", Colors::STAT_LABEL, m_font14);
    col += 120;

    std::snprintf(buf, sizeof(buf), "%.0f", m_editRpmLimit);
    std::string rpmDisplay = (m_focusedField == FOCUS_RPM) ? m_rpmInputText : std::string(buf);
    drawTextBox(col, py, tbW, btnH, rpmDisplay, m_focusedField == FOCUS_RPM, m_font14);
    m_textBoxRects.push_back({col, py, tbW, btnH, FOCUS_RPM});
    col += tbW + spacing * 4;

    // Duty Cycle Limit — label + text box
    drawText(col, py + 3, "Duty Limit:", Colors::STAT_LABEL, m_font14);
    col += 130;

    std::snprintf(buf, sizeof(buf), "%.2f", m_editDutyCycleLimit);
    std::string dutyDisplay = (m_focusedField == FOCUS_DUTY) ? m_dutyInputText : std::string(buf);
    drawTextBox(col, py, tbW, btnH, dutyDisplay, m_focusedField == FOCUS_DUTY, m_font14);
    m_textBoxRects.push_back({col, py, tbW, btnH, FOCUS_DUTY});

    // Second row: Mode, Inverted, Apply, Cancel
    py += 36;
    col = px;

    // Control Mode toggle — dark text on bright background
    drawText(col, py + 3, "Mode:", Colors::STAT_LABEL, m_font14);
    col += 70;

    const char* modeLabel = (m_editControlMode == 0) ? "RPM" : "DUTY";
    SDL_Color modeBg = (m_editControlMode == 0) ? Colors::STAT_GOOD : Colors::STAT_WARN;
    drawFilledRect(col, py, 70, btnH, modeBg);
    drawBorderRect(col, py, 70, btnH, Colors::BORDER);
    int tw = 0, th = 0;
    TTF_SizeText(m_font14, modeLabel, &tw, &th);
    drawText(col + (70 - tw) / 2, py + (btnH - th) / 2, modeLabel, Colors::TEXT_DARK, m_font14);
    m_editButtons.push_back({col, py, 70, btnH, ACT_MODE_TOGGLE});
    col += 70 + spacing * 3;

    // Inverted toggle — dark text on bright background
    drawText(col, py + 3, "Inverted:", Colors::STAT_LABEL, m_font14);
    col += 110;

    const char* invLabel = m_editInverted ? "YES" : "NO";
    SDL_Color invBg = m_editInverted ? Colors::STAT_WARN : Colors::STAT_GOOD;
    drawFilledRect(col, py, 60, btnH, invBg);
    drawBorderRect(col, py, 60, btnH, Colors::BORDER);
    TTF_SizeText(m_font14, invLabel, &tw, &th);
    drawText(col + (60 - tw) / 2, py + (btnH - th) / 2, invLabel, Colors::TEXT_DARK, m_font14);
    m_editButtons.push_back({col, py, 60, btnH, ACT_INV_TOGGLE});
    col += 60 + spacing * 4;

    // Apply button
    drawButton(col, py, 80, btnH, "APPLY", Colors::ACCENT_BLUE, m_font14);
    m_editButtons.push_back({col, py, 80, btnH, ACT_APPLY});
    col += 80 + spacing;

    // Cancel button
    drawButton(col, py, 80, btnH, "CANCEL", Colors::STAT_CRIT, m_font14);
    m_editButtons.push_back({col, py, 80, btnH, ACT_CANCEL});
}

void MainWindow::renderFooter(int winW, int winH)
{
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
