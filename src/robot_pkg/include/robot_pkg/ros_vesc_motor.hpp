#ifndef ROS_VESC_MOTOR_HPP
#define ROS_VESC_MOTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_pkg/VESC.hpp"
#include "robot_msgs/msg/motor_telemetry.hpp"
#include <atomic>
#include <string>
#include <mutex>
#include <functional>

struct MotorSettings {
    uint8_t     vesc_id          = 0;
    float       rpm_limit        = 5000.0f;
    float       duty_cycle_limit = 1.0f;
    uint8_t     control_mode     = 0; // 0 = RPM, 1 = Duty Cycle
    bool        inverted         = false;
    bool        enabled          = true;
    std::string port             = ""; 
};

class RosVescMotor {
public:
    RosVescMotor(rclcpp::Node* node, const std::string& name, const std::string& telemetry_topic, std::function<void()> on_port_updated);
    ~RosVescMotor();

    void disconnect();
    void set(float power);
    void setWithCustomLimits(float power, float custom_rpm, float custom_duty);
    
    void applySettings(const MotorSettings& new_settings);
    MotorSettings getSettings();
    bool autoConnectInit();

private:
    void controlLoop();
    void telemetryLoop();

    rclcpp::Node* node_;
    std::string name_;
    VESC motor_;
    
    MotorSettings settings_;
    std::mutex settings_mutex_;
    std::atomic<float> current_cmd_{0.0f};

    float active_rpm_limit_ = 5000.0f;
    float active_duty_limit_ = 1.0f;
    bool use_custom_limits_ = false;

    rclcpp::CallbackGroup::SharedPtr control_cb_group_;
    rclcpp::CallbackGroup::SharedPtr telem_cb_group_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr telem_timer_;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr telem_pub_;
    
    std::function<void()> on_port_updated_cb_;
};

#endif // ROS_VESC_MOTOR_HPP