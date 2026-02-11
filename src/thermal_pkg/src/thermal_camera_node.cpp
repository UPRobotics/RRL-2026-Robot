#include "thermal_pkg/thermal_camera_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

namespace thermal_pkg
{

ThermalCameraNode::ThermalCameraNode()
: Node("thermal_camera_node"),
  serial_fd_(-1)
{
  // Declare parameters
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("publish_rate", 4.0);

  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  double publish_rate = this->get_parameter("publish_rate").as_double();

  // Create publisher
  thermal_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thermal_data", 10);

  // Initialize serial
  if (!initialize_serial()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port '%s'", serial_port_.c_str());
  }

  // Create timer at the requested publish rate
  auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / publish_rate));
  timer_ = this->create_wall_timer(
    period, std::bind(&ThermalCameraNode::timer_callback, this));
}

ThermalCameraNode::~ThermalCameraNode()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    RCLCPP_INFO(this->get_logger(), "Serial port closed");
  }
}

bool ThermalCameraNode::initialize_serial()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s (errno: %d)",
      serial_port_.c_str(), errno);
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes (errno: %d)", errno);
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Map baud rate
  speed_t baud;
  switch (baud_rate_) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unsupported baud rate: %d", baud_rate_);
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
  }

  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // 8N1 mode
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // Raw input
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // Raw output
  tty.c_oflag &= ~OPOST;

  // Read timeout: VMIN=0 VTIME=10 -> up to 1 second wait
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes (errno: %d)", errno);
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  RCLCPP_INFO(this->get_logger(), "Connected to %s at %d baud", serial_port_.c_str(), baud_rate_);
  return true;
}

std::vector<float> ThermalCameraNode::read_thermal_data()
{
  std::vector<float> result;
  if (serial_fd_ < 0) {
    return result;
  }

  // Flush pending input
  tcflush(serial_fd_, TCIFLUSH);

  // Synchronize on header: 0x5A 0x5A 0x02 0x06
  static constexpr uint8_t header[] = {0x5A, 0x5A, 0x02, 0x06};
  int header_idx = 0;
  uint8_t byte;
  int max_attempts = 10000;

  while (header_idx < 4 && max_attempts-- > 0) {
    ssize_t n = read(serial_fd_, &byte, 1);
    if (n <= 0) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for header");
      return result;
    }
    if (byte == header[header_idx]) {
      header_idx++;
    } else {
      header_idx = (byte == header[0]) ? 1 : 0;
    }
  }

  if (header_idx < 4) {
    RCLCPP_WARN(this->get_logger(), "Failed to find header after max attempts");
    return result;
  }

  // Read 768 * 2 = 1536 bytes of pixel data
  std::vector<uint8_t> data(DATA_SIZE);
  size_t total_read = 0;
  while (total_read < static_cast<size_t>(DATA_SIZE)) {
    ssize_t n = read(serial_fd_, data.data() + total_read, DATA_SIZE - total_read);
    if (n <= 0) {
      RCLCPP_WARN(this->get_logger(), "Incomplete data: received %zu bytes, expected %d",
        total_read, DATA_SIZE);
      return result;
    }
    total_read += static_cast<size_t>(n);
  }

  // Unpack as little-endian int16_t, convert to °C (divide by 100)
  result.resize(PIXEL_COUNT);
  for (int i = 0; i < PIXEL_COUNT; i++) {
    auto raw = static_cast<int16_t>(data[i * 2] | (data[i * 2 + 1] << 8));
    result[i] = raw / 100.0f;
  }

  // --- Dead-pixel filter ---
  // Compute median and MAD to detect statistical outliers.
  // Pixels deviating more than 4 MADs from the median are replaced
  // with the average of their valid spatial neighbors.
  std::vector<float> sorted = result;
  std::sort(sorted.begin(), sorted.end());
  float median = sorted[PIXEL_COUNT / 2];

  std::vector<float> abs_dev(PIXEL_COUNT);
  for (int i = 0; i < PIXEL_COUNT; i++) {
    abs_dev[i] = std::fabs(result[i] - median);
  }
  std::sort(abs_dev.begin(), abs_dev.end());
  float mad = abs_dev[PIXEL_COUNT / 2];
  float threshold = std::max(mad * 4.0f, 2.0f);  // at least 2 °C tolerance

  int fixed_count = 0;
  for (int y = 0; y < SENSOR_HEIGHT; y++) {
    for (int x = 0; x < SENSOR_WIDTH; x++) {
      int idx = y * SENSOR_WIDTH + x;
      if (std::fabs(result[idx] - median) > threshold) {
        // Replace with average of valid neighbors
        float sum = 0.0f;
        int count = 0;
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (nx >= 0 && nx < SENSOR_WIDTH && ny >= 0 && ny < SENSOR_HEIGHT) {
              int nidx = ny * SENSOR_WIDTH + nx;
              if (std::fabs(result[nidx] - median) <= threshold) {
                sum += result[nidx];
                count++;
              }
            }
          }
        }
        if (count > 0) {
          result[idx] = sum / count;
        } else {
          result[idx] = median;
        }
        fixed_count++;
      }
    }
  }
  if (fixed_count > 0) {
    RCLCPP_INFO(this->get_logger(), "Filtered %d dead/outlier pixel(s)", fixed_count);
  }

  RCLCPP_INFO(this->get_logger(), "Sample temperatures (C): %.2f, %.2f, %.2f, %.2f, %.2f",
    result[0], result[1], result[2], result[3], result[4]);

  return result;
}

void ThermalCameraNode::timer_callback()
{
  auto data = read_thermal_data();
  if (data.empty()) {
    return;
  }

  auto msg = std_msgs::msg::Float32MultiArray();
  msg.data = data;
  thermal_pub_->publish(msg);
}

}  // namespace thermal_pkg

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thermal_pkg::ThermalCameraNode>());
  rclcpp::shutdown();
  return 0;
}
