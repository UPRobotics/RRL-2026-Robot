#ifndef THERMAL_PKG__THERMAL_CAMERA_NODE_HPP_
#define THERMAL_PKG__THERMAL_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <string>
#include <vector>

namespace thermal_pkg
{

class ThermalCameraNode : public rclcpp::Node
{
public:
  ThermalCameraNode();
  ~ThermalCameraNode() override;

private:
  void timer_callback();
  bool initialize_serial();
  std::vector<float> read_thermal_data();

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thermal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;

  static constexpr int SENSOR_WIDTH = 32;
  static constexpr int SENSOR_HEIGHT = 24;
  static constexpr int PIXEL_COUNT = SENSOR_WIDTH * SENSOR_HEIGHT;  // 768
  static constexpr int DATA_SIZE = PIXEL_COUNT * 2;                 // 1536 bytes
};

}  // namespace thermal_pkg

#endif  // THERMAL_PKG__THERMAL_CAMERA_NODE_HPP_
