#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <vector>
#include <mutex>
#include <chrono>

namespace detections {

/// Subscribes to /thermal_data (Float32MultiArray, 768 floats 32×24) and
/// renders a false-colour thermal image into the given SDL rect.
class ThermalPanel {
public:
    explicit ThermalPanel(rclcpp::Node::SharedPtr node);
    ~ThermalPanel();

    /// Render into area.  Must be called from the SDL main thread.
    void render(SDL_Renderer* r, SDL_Rect area, TTF_Font* font12, TTF_Font* font14);

private:
    void onThermalData(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_sub;

    std::vector<float> m_data;   // 768 floats (32×24), protected by m_mutex
    std::mutex  m_mutex;
    bool        m_hasData  = false;
    bool        m_dirty    = false;   // new data arrived, texture needs refresh
    std::chrono::steady_clock::time_point m_lastUpdate;

    SDL_Texture* m_tex = nullptr;    // 32×24 streaming RGBA texture
};

} // namespace detections
