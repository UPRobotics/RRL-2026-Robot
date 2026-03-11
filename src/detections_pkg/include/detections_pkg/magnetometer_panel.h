#pragma once

#include "detections_pkg/types.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <deque>
#include <mutex>
#include <string>

namespace detections {

/// Subscribes to /magnetometer_data and renders a side panel with
/// current reading + scrolling line graph.
class MagnetometerPanel {
public:
    explicit MagnetometerPanel(rclcpp::Node::SharedPtr node);

    /// Render the panel into the given rect area.
    void render(SDL_Renderer* renderer, SDL_Rect area);

private:
    void onMagData(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub;

    // Data ring buffer
    static constexpr size_t MAX_HISTORY = 60;
    std::deque<MagReading> m_history;
    MagReading m_latest;
    bool m_hasData = false;
    std::mutex m_dataMutex;
    TimePoint m_lastReceived;
};

} // namespace detections
