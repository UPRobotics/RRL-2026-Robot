#ifndef BODY_NODE_HPP
#define BODY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_pkg/ros_vesc_motor.hpp"
#include "robot_msgs/msg/control_input.hpp"
#include <robot_msgs/msg/motor_config.hpp>
#include "robot_pkg/json.hpp"
#include <memory>
#include <string>
#include <atomic>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

class BodyNode : public rclcpp::Node {
public:
    BodyNode();
    ~BodyNode();

private:
    void onControlInput(const robot_msgs::msg::ControlInput::SharedPtr msg);
    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void loadConfig();

    std::unique_ptr<RosVescMotor> body_left_flipper_;
    std::unique_ptr<RosVescMotor> body_left_;
    std::unique_ptr<RosVescMotor> body_right_;
    std::unique_ptr<RosVescMotor> body_right_flipper_;

    rclcpp::Subscription<robot_msgs::msg::ControlInput>::SharedPtr sub_input_;
    rclcpp::Subscription<robot_msgs::msg::MotorConfig>::SharedPtr config_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_CmdVel;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_autonomy_;

    float max_linear_;
    float max_angular_;

    std::string config_path_;
    nlohmann::json config_data_;
    void handleMotorConfig(const robot_msgs::msg::MotorConfig::SharedPtr msg);
    std::atomic<bool> is_autonomous_{false};
    std::atomic<bool> joystick_override{false};

};

#endif