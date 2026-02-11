#ifndef THERMAL_PKG__THERMAL_DISPLAY_NODE_HPP_
#define THERMAL_PKG__THERMAL_DISPLAY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace thermal_pkg
{

struct ClickPoint
{
  int display_x;
  int display_y;
  int sensor_x;
  int sensor_y;
};

class ThermalDisplayNode : public rclcpp::Node
{
public:
  ThermalDisplayNode();
  ~ThermalDisplayNode() override;

private:
  void thermal_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void update_display();
  static void mouse_callback(int event, int x, int y, int flags, void * userdata);
  void handle_click(int x, int y);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr display_timer_;

  std::vector<float> current_data_;
  std::mutex data_mutex_;
  bool has_new_data_;

  std::vector<ClickPoint> click_points_;

  static constexpr int SENSOR_WIDTH = 32;
  static constexpr int SENSOR_HEIGHT = 24;
  static constexpr int DISPLAY_WIDTH = 640;
  static constexpr int DISPLAY_HEIGHT = 480;

  static const std::string WINDOW_NAME;
};

}  // namespace thermal_pkg

#endif  // THERMAL_PKG__THERMAL_DISPLAY_NODE_HPP_
