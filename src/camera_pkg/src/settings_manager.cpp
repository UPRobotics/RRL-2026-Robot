#include "camera_pkg/settings_manager.h"
#include <spdlog/spdlog.h>
#include <fstream>

using json = nlohmann::json;

namespace camera_viewer {

SettingsManager& SettingsManager::instance() {
    static SettingsManager instance;
    return instance;
}

int SettingsManager::getCameraRotation(int index) const {
    if (index < 0 || index >= static_cast<int>(m_cameraConfigs.size())) {
        return 0;
    }
    return m_cameraConfigs[index].rotation_deg;
}

void SettingsManager::setCameraRotation(int index, int rotationDeg, const std::string& filepath) {
    if (index < 0 || index >= static_cast<int>(m_cameraConfigs.size())) {
        return;
    }
    // Normalize to 0/90/180/270
    rotationDeg %= 360;
    if (rotationDeg < 0) rotationDeg += 360;
    if (rotationDeg % 90 != 0) {
        rotationDeg = 0;
    }
    m_cameraConfigs[index].rotation_deg = rotationDeg;
    save(filepath);
}

void SettingsManager::setGrid2x2Slots(const std::array<int, 4>& slots, const std::string& filepath) {
    m_grid2x2Slots = slots;
    save(filepath);
}

bool SettingsManager::load(const std::string& filepath) {
    // If a config directory was set (ROS2 package), resolve relative to it
    std::string resolvedPath = filepath;
    if (!m_configDir.empty() && filepath.front() != '/') {
        resolvedPath = m_configDir + "/" + filepath;
    }

    try {
        std::ifstream file(resolvedPath);
        if (!file.is_open()) {
            spdlog::warn("Settings file not found: {}. Using defaults.", resolvedPath);
            return false;
        }
        
        json j;
        file >> j;
        
        // Load keybindings
        if (j.contains("keybindings")) {
            for (auto& [action, keyName] : j["keybindings"].items()) {
                SDL_Keycode key = stringToKeycode(keyName.get<std::string>());
                if (key != SDLK_UNKNOWN) {
                    m_keybindings[action] = key;
                }
            }
        }
        
        // Load display settings
        if (j.contains("display")) {
            auto& display = j["display"];
            m_rememberWindowPosition = display.value("remember_window_position", true);
            m_rememberWindowSize = display.value("remember_window_size", true);
            m_fullscreenOnStartup = display.value("fullscreen_on_startup", false);
            m_lastWindowX = display.value("last_window_x", -1);
            m_lastWindowY = display.value("last_window_y", -1);
            m_lastWindowWidth = display.value("last_window_width", 1280);
            m_lastWindowHeight = display.value("last_window_height", 720);
        }
        
        // Load console settings
        if (j.contains("console")) {
            auto& console = j["console"];
            m_autoOpenConsole = console.value("auto_open_on_startup", false);
            m_rememberConsolePosition = console.value("remember_position", true);
            m_lastConsoleX = console.value("last_console_x", 100);
            m_lastConsoleY = console.value("last_console_y", 100);
            m_lastConsoleWidth = console.value("last_console_width", 800);
            m_lastConsoleHeight = console.value("last_console_height", 500);
        }
        
        // Load camera configurations
        if (j.contains("cameras")) {
            m_cameraConfigs.clear();
            for (const auto& cam : j["cameras"]) {
                CameraConfig config;
                config.id = cam.value("id", "");
                config.name = cam.value("name", "");
                config.ip = cam.value("ip", "");
                config.port = cam.value("port", 554);
                config.url_highres = cam.value("url_highres", "");
                config.url_lowres = cam.value("url_lowres", "");
                config.enabled = cam.value("enabled", true);
                config.rotation_deg = cam.value("rotation_deg", 0);
                config.available = false; // Will be set by discovery

                // Determine camera source type (RTSP is default)
                std::string srcType = cam.value("source_type", "rtsp");
                if (srcType == "thermal") {
                    config.source_type = CameraSourceType::Thermal;
                    config.ros_topic = cam.value("ros_topic", "/thermal_data");
                } else {
                    config.source_type = CameraSourceType::RTSP;
                }

                m_cameraConfigs.push_back(config);
            }
            spdlog::info("Loaded {} camera configurations", m_cameraConfigs.size());
        }
        
        // Load streaming settings
        if (j.contains("streaming")) {
            auto& streaming = j["streaming"];
            std::string quality = streaming.value("default_quality", "high");
            m_streamingSettings.default_quality = (quality == "low") ? StreamQuality::Low : StreamQuality::High;
            m_streamingSettings.ping_timeout_ms = streaming.value("ping_timeout_ms", 1000);
            m_streamingSettings.connection_timeout_ms = streaming.value("connection_timeout_ms", 5000);
            m_streamingSettings.reconnect_delay_ms = streaming.value("reconnect_delay_ms", 2000);
            m_streamingSettings.max_reconnect_attempts = streaming.value("max_reconnect_attempts", 5);
            m_streamingSettings.frame_buffer_size = streaming.value("frame_buffer_size", 1);
            m_streamingSettings.frame_timeout_ms = streaming.value("frame_timeout_ms", 5000);
        }
        
        // Load 2x2 grid slot assignments
        if (j.contains("grid_2x2") && j["grid_2x2"].contains("slots")) {
            auto& slots = j["grid_2x2"]["slots"];
            if (slots.is_array() && slots.size() == 4) {
                for (int i = 0; i < 4; ++i) {
                    m_grid2x2Slots[i] = slots[i].get<int>();
                }
                spdlog::info("Loaded 2x2 grid slots: [{}, {}, {}, {}]",
                    m_grid2x2Slots[0], m_grid2x2Slots[1], m_grid2x2Slots[2], m_grid2x2Slots[3]);
            }
        }
        
        spdlog::info("Settings loaded from {}", resolvedPath);
        return true;
    }
    catch (const std::exception& e) {
        spdlog::error("Failed to load settings: {}", e.what());
        return false;
    }
}

bool SettingsManager::save(const std::string& filepath) {
    // If a config directory was set (ROS2 package), resolve relative to it
    std::string resolvedPath = filepath;
    if (!m_configDir.empty() && filepath.front() != '/') {
        resolvedPath = m_configDir + "/" + filepath;
    }

    try {
        json j;
        
        // Save camera configurations (preserve them)
        json cameras = json::array();
        for (const auto& config : m_cameraConfigs) {
            json cam;
            cam["id"] = config.id;
            cam["name"] = config.name;
            cam["ip"] = config.ip;
            cam["port"] = config.port;
            cam["url_highres"] = config.url_highres;
            cam["url_lowres"] = config.url_lowres;
            cam["enabled"] = config.enabled;
            cam["rotation_deg"] = config.rotation_deg;

            // Persist source type for thermal cameras
            if (config.source_type == CameraSourceType::Thermal) {
                cam["source_type"] = "thermal";
                cam["ros_topic"] = config.ros_topic;
            } else {
                cam["source_type"] = "rtsp";
            }

            cameras.push_back(cam);
        }
        j["cameras"] = cameras;
        
        // Save streaming settings
        j["streaming"] = {
            {"default_quality", m_streamingSettings.default_quality == StreamQuality::High ? "high" : "low"},
            {"ping_timeout_ms", m_streamingSettings.ping_timeout_ms},
            {"connection_timeout_ms", m_streamingSettings.connection_timeout_ms},
            {"reconnect_delay_ms", m_streamingSettings.reconnect_delay_ms},
            {"max_reconnect_attempts", m_streamingSettings.max_reconnect_attempts},
            {"frame_buffer_size", m_streamingSettings.frame_buffer_size},
            {"frame_timeout_ms", m_streamingSettings.frame_timeout_ms}
        };
        
        // Save keybindings
        json keybindings;
        for (const auto& [action, key] : m_keybindings) {
            keybindings[action] = keycodeToString(key);
        }
        j["keybindings"] = keybindings;
        
        // Save display settings
        j["display"] = {
            {"remember_window_position", m_rememberWindowPosition},
            {"remember_window_size", m_rememberWindowSize},
            {"last_window_x", m_lastWindowX},
            {"last_window_y", m_lastWindowY},
            {"last_window_width", m_lastWindowWidth},
            {"last_window_height", m_lastWindowHeight},
            {"fullscreen_on_startup", m_fullscreenOnStartup}
        };
        
        // Save console settings
        j["console"] = {
            {"auto_open_on_startup", m_autoOpenConsole},
            {"remember_position", m_rememberConsolePosition},
            {"last_console_x", m_lastConsoleX},
            {"last_console_y", m_lastConsoleY},
            {"last_console_width", m_lastConsoleWidth},
            {"last_console_height", m_lastConsoleHeight}
        };
        
        // Save 2x2 grid slot assignments
        j["grid_2x2"] = {
            {"slots", {m_grid2x2Slots[0], m_grid2x2Slots[1], m_grid2x2Slots[2], m_grid2x2Slots[3]}}
        };
        
        std::ofstream file(resolvedPath);
        if (!file.is_open()) {
            spdlog::error("Failed to open settings file for writing: {}", resolvedPath);
            return false;
        }
        
        file << j.dump(2);
        spdlog::debug("Settings saved to {}", resolvedPath);
        return true;
    }
    catch (const std::exception& e) {
        spdlog::error("Failed to save settings: {}", e.what());
        return false;
    }
}

SDL_Keycode SettingsManager::getKeyBinding(const std::string& action) const {
    auto it = m_keybindings.find(action);
    return (it != m_keybindings.end()) ? it->second : SDLK_UNKNOWN;
}

bool SettingsManager::isKeyForAction(SDL_Keycode key, const std::string& action) const {
    auto it = m_keybindings.find(action);
    return (it != m_keybindings.end() && it->second == key);
}

void SettingsManager::setKeyBinding(const std::string& action, SDL_Keycode key) {
    m_keybindings[action] = key;
}

SDL_Keycode SettingsManager::stringToKeycode(const std::string& keyName) const {
    // Common key mappings
    static const std::map<std::string, SDL_Keycode> keyMap = {
        {"A", SDLK_a}, {"B", SDLK_b}, {"C", SDLK_c}, {"D", SDLK_d},
        {"E", SDLK_e}, {"F", SDLK_f}, {"G", SDLK_g}, {"H", SDLK_h},
        {"I", SDLK_i}, {"J", SDLK_j}, {"K", SDLK_k}, {"L", SDLK_l},
        {"M", SDLK_m}, {"N", SDLK_n}, {"O", SDLK_o}, {"P", SDLK_p},
        {"Q", SDLK_q}, {"R", SDLK_r}, {"S", SDLK_s}, {"T", SDLK_t},
        {"U", SDLK_u}, {"V", SDLK_v}, {"W", SDLK_w}, {"X", SDLK_x},
        {"Y", SDLK_y}, {"Z", SDLK_z},
        {"0", SDLK_0}, {"1", SDLK_1}, {"2", SDLK_2}, {"3", SDLK_3},
        {"4", SDLK_4}, {"5", SDLK_5}, {"6", SDLK_6}, {"7", SDLK_7},
        {"8", SDLK_8}, {"9", SDLK_9},
        {"Escape", SDLK_ESCAPE}, {"Space", SDLK_SPACE}, {"Return", SDLK_RETURN},
        {"Enter", SDLK_RETURN}, {"Tab", SDLK_TAB}, {"Backspace", SDLK_BACKSPACE},
        {"Delete", SDLK_DELETE}, {"Insert", SDLK_INSERT},
        {"Home", SDLK_HOME}, {"End", SDLK_END},
        {"PageUp", SDLK_PAGEUP}, {"PageDown", SDLK_PAGEDOWN},
        {"Left", SDLK_LEFT}, {"Right", SDLK_RIGHT}, 
        {"Up", SDLK_UP}, {"Down", SDLK_DOWN},
        {"F1", SDLK_F1}, {"F2", SDLK_F2}, {"F3", SDLK_F3}, {"F4", SDLK_F4},
        {"F5", SDLK_F5}, {"F6", SDLK_F6}, {"F7", SDLK_F7}, {"F8", SDLK_F8},
        {"F9", SDLK_F9}, {"F10", SDLK_F10}, {"F11", SDLK_F11}, {"F12", SDLK_F12}
    };
    
    auto it = keyMap.find(keyName);
    return (it != keyMap.end()) ? it->second : SDLK_UNKNOWN;
}

std::string SettingsManager::keycodeToString(SDL_Keycode key) const {
    // Reverse mapping
    static const std::map<SDL_Keycode, std::string> reverseMap = {
        {SDLK_a, "A"}, {SDLK_b, "B"}, {SDLK_c, "C"}, {SDLK_d, "D"},
        {SDLK_e, "E"}, {SDLK_f, "F"}, {SDLK_g, "G"}, {SDLK_h, "H"},
        {SDLK_i, "I"}, {SDLK_j, "J"}, {SDLK_k, "K"}, {SDLK_l, "L"},
        {SDLK_m, "M"}, {SDLK_n, "N"}, {SDLK_o, "O"}, {SDLK_p, "P"},
        {SDLK_q, "Q"}, {SDLK_r, "R"}, {SDLK_s, "S"}, {SDLK_t, "T"},
        {SDLK_u, "U"}, {SDLK_v, "V"}, {SDLK_w, "W"}, {SDLK_x, "X"},
        {SDLK_y, "Y"}, {SDLK_z, "Z"},
        {SDLK_0, "0"}, {SDLK_1, "1"}, {SDLK_2, "2"}, {SDLK_3, "3"},
        {SDLK_4, "4"}, {SDLK_5, "5"}, {SDLK_6, "6"}, {SDLK_7, "7"},
        {SDLK_8, "8"}, {SDLK_9, "9"},
        {SDLK_ESCAPE, "Escape"}, {SDLK_SPACE, "Space"}, {SDLK_RETURN, "Return"},
        {SDLK_TAB, "Tab"}, {SDLK_BACKSPACE, "Backspace"},
        {SDLK_DELETE, "Delete"}, {SDLK_INSERT, "Insert"},
        {SDLK_HOME, "Home"}, {SDLK_END, "End"},
        {SDLK_PAGEUP, "PageUp"}, {SDLK_PAGEDOWN, "PageDown"},
        {SDLK_LEFT, "Left"}, {SDLK_RIGHT, "Right"},
        {SDLK_UP, "Up"}, {SDLK_DOWN, "Down"},
        {SDLK_F1, "F1"}, {SDLK_F2, "F2"}, {SDLK_F3, "F3"}, {SDLK_F4, "F4"},
        {SDLK_F5, "F5"}, {SDLK_F6, "F6"}, {SDLK_F7, "F7"}, {SDLK_F8, "F8"},
        {SDLK_F9, "F9"}, {SDLK_F10, "F10"}, {SDLK_F11, "F11"}, {SDLK_F12, "F12"}
    };
    
    auto it = reverseMap.find(key);
    return (it != reverseMap.end()) ? it->second : "Unknown";
}

} // namespace camera_viewer
