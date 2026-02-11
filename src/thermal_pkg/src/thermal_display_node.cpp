#include "thermal_pkg/thermal_display_node.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace thermal_pkg
{

const std::string ThermalDisplayNode::WINDOW_NAME =
  "MLX90640 Thermal Display - Click to measure, 'R' to clear";

ThermalDisplayNode::ThermalDisplayNode()
: Node("thermal_display_node"),
  has_new_data_(false)
{
  // Subscribe to thermal data
  subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/thermal_data", 10,
    std::bind(&ThermalDisplayNode::thermal_callback, this, std::placeholders::_1));

  // Create OpenCV window
  cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(WINDOW_NAME, &ThermalDisplayNode::mouse_callback, this);

  // Timer for display update at ~10 Hz
  display_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ThermalDisplayNode::update_display, this));

  RCLCPP_INFO(this->get_logger(),
    "Thermal display started. Click to measure temperature, press 'R' to clear points.");
}

ThermalDisplayNode::~ThermalDisplayNode()
{
  cv::destroyAllWindows();
}

void ThermalDisplayNode::thermal_callback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (static_cast<int>(msg->data.size()) != SENSOR_WIDTH * SENSOR_HEIGHT) {
    RCLCPP_WARN(this->get_logger(), "Invalid data length: %zu, expected %d",
      msg->data.size(), SENSOR_WIDTH * SENSOR_HEIGHT);
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  current_data_ = msg->data;
  has_new_data_ = true;

  float min_val = *std::min_element(current_data_.begin(), current_data_.end());
  float max_val = *std::max_element(current_data_.begin(), current_data_.end());
  RCLCPP_INFO(this->get_logger(), "Received thermal data: min=%.2fC, max=%.2fC",
    min_val, max_val);
}

void ThermalDisplayNode::update_display()
{
  std::vector<float> data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (current_data_.empty()) {
      cv::waitKey(1);
      return;
    }
    data = current_data_;
    has_new_data_ = false;
  }

  // Reshape to 24x32 matrix (CV_32F)
  cv::Mat sensor_data(SENSOR_HEIGHT, SENSOR_WIDTH, CV_32F, data.data());

  // Find min/max values and locations
  double min_val, max_val;
  cv::Point min_loc, max_loc;
  cv::minMaxLoc(sensor_data, &min_val, &max_val, &min_loc, &max_loc);

  // Use percentile-based normalization (2nd / 98th percentile)
  // to prevent any remaining outlier pixels from skewing the colors
  std::vector<float> sorted_vals(data.begin(), data.end());
  std::sort(sorted_vals.begin(), sorted_vals.end());
  int n = static_cast<int>(sorted_vals.size());
  double pct_low  = sorted_vals[static_cast<int>(n * 0.02)];
  double pct_high = sorted_vals[static_cast<int>(n * 0.98)];
  if (std::abs(pct_high - pct_low) < 1e-6) {
    pct_low  = min_val;
    pct_high = max_val;
  }

  // Normalize to 0-255 using the percentile range and clamp
  cv::Mat normalized;
  if (std::abs(pct_high - pct_low) < 1e-6) {
    normalized = cv::Mat::zeros(SENSOR_HEIGHT, SENSOR_WIDTH, CV_8U);
  } else {
    // Clamp then scale
    cv::Mat clamped;
    cv::max(sensor_data, pct_low, clamped);
    cv::min(clamped, pct_high, clamped);
    clamped.convertTo(normalized, CV_8U,
      255.0 / (pct_high - pct_low),
      -pct_low * 255.0 / (pct_high - pct_low));
  }

  // Apply INFERNO colormap
  cv::Mat colored;
  cv::applyColorMap(normalized, colored, cv::COLORMAP_INFERNO);

  // Resize to display size using bicubic interpolation
  cv::Mat display;
  cv::resize(colored, display, cv::Size(DISPLAY_WIDTH, DISPLAY_HEIGHT), 0, 0, cv::INTER_CUBIC);

  // Scale min/max positions to display coordinates
  int min_x_disp = static_cast<int>(
    static_cast<float>(min_loc.x) / (SENSOR_WIDTH - 1) * (DISPLAY_WIDTH - 1));
  int min_y_disp = static_cast<int>(
    static_cast<float>(min_loc.y) / (SENSOR_HEIGHT - 1) * (DISPLAY_HEIGHT - 1));
  int max_x_disp = static_cast<int>(
    static_cast<float>(max_loc.x) / (SENSOR_WIDTH - 1) * (DISPLAY_WIDTH - 1));
  int max_y_disp = static_cast<int>(
    static_cast<float>(max_loc.y) / (SENSOR_HEIGHT - 1) * (DISPLAY_HEIGHT - 1));

  // Draw max temperature marker (red circle)
  cv::circle(display, cv::Point(max_x_disp, max_y_disp), 8, cv::Scalar(0, 0, 255), 2);

  // Draw min temperature marker (cyan circle)
  cv::circle(display, cv::Point(min_x_disp, min_y_disp), 8, cv::Scalar(255, 255, 0), 2);

  // Draw clicked measurement points (green circles + temperature labels)
  for (const auto & point : click_points_) {
    int sensor_idx = point.sensor_y * SENSOR_WIDTH + point.sensor_x;
    if (sensor_idx >= 0 && sensor_idx < static_cast<int>(data.size())) {
      float temp = data[sensor_idx];

      // Green circle
      cv::circle(display, cv::Point(point.display_x, point.display_y),
        6, cv::Scalar(0, 255, 0), 2);

      // Temperature label
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1) << temp << " C";
      std::string label = oss.str();

      int label_x = point.display_x + 10;
      int label_y = point.display_y - 5;

      // Text background
      int baseline;
      cv::Size text_size = cv::getTextSize(
        label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      cv::rectangle(display,
        cv::Point(label_x - 2, label_y - text_size.height - 2),
        cv::Point(label_x + text_size.width + 2, label_y + baseline + 2),
        cv::Scalar(0, 0, 0), cv::FILLED);

      // Text
      cv::putText(display, label,
        cv::Point(label_x, label_y),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
  }

  // Temperature info bar at the bottom
  std::ostringstream info;
  info << std::fixed << std::setprecision(2);
  info << "Range: " << min_val << "C to " << max_val << "C | ";
  info << "MIN: " << min_val << "C at (" << min_loc.x << "," << min_loc.y << ") | ";
  info << "MAX: " << max_val << "C at (" << max_loc.x << "," << max_loc.y << ")";
  if (!click_points_.empty()) {
    info << " | Points: " << click_points_.size();
  }

  cv::Mat info_bar(40, DISPLAY_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::putText(info_bar, info.str(),
    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

  // Combine thermal image and info bar
  cv::Mat combined;
  cv::vconcat(display, info_bar, combined);

  cv::imshow(WINDOW_NAME, combined);

  int key = cv::waitKey(1);
  if (key == 'r' || key == 'R') {
    click_points_.clear();
    RCLCPP_INFO(this->get_logger(), "Cleared all temperature measurement points");
  }
}

void ThermalDisplayNode::mouse_callback(
  int event, int x, int y, int /*flags*/, void * userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN) {
    auto * node = static_cast<ThermalDisplayNode *>(userdata);
    node->handle_click(x, y);
  }
}

void ThermalDisplayNode::handle_click(int x, int y)
{
  // Ignore clicks outside the thermal image area
  if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (current_data_.empty()) {
    return;
  }

  // Convert display coordinates to sensor coordinates
  float x_ratio = static_cast<float>(x) / DISPLAY_WIDTH;
  float y_ratio = static_cast<float>(y) / DISPLAY_HEIGHT;

  x_ratio = std::max(0.0f, std::min(1.0f, x_ratio));
  y_ratio = std::max(0.0f, std::min(1.0f, y_ratio));

  int sensor_x = static_cast<int>(x_ratio * (SENSOR_WIDTH - 1));
  int sensor_y = static_cast<int>(y_ratio * (SENSOR_HEIGHT - 1));

  int sensor_idx = sensor_y * SENSOR_WIDTH + sensor_x;
  float temp = current_data_[sensor_idx];

  click_points_.push_back({x, y, sensor_x, sensor_y});

  RCLCPP_INFO(this->get_logger(),
    "Added measurement point at pixel (%d,%d): current temp %.2fC",
    sensor_x, sensor_y, temp);
}

}  // namespace thermal_pkg

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thermal_pkg::ThermalDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
