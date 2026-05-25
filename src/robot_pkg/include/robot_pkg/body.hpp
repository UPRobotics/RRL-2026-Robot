#ifndef BODY_NODE_HPP
#define BODY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_pkg/ros_vesc_motor.hpp"
#include "robot_msgs/msg/control_input.hpp"
#include "robot_pkg/json.hpp"
#include <memory>
#include <string>

class BodyNode : public rclcpp::Node {
public:
    BodyNode();
    ~BodyNode();

private:
    void onControlInput(const robot_msgs::msg::ControlInput::SharedPtr msg);
    void loadConfig();

    std::unique_ptr<RosVescMotor> body_left_flipper_;
    std::unique_ptr<RosVescMotor> body_left_;
    std::unique_ptr<RosVescMotor> body_right_;
    std::unique_ptr<RosVescMotor> body_right_flipper_;

    rclcpp::Subscription<robot_msgs::msg::ControlInput>::SharedPtr sub_input_;
    std::string config_path_;
    nlohmann::json config_data_;
};

#endif // SINGLE_MOTOR_NODE_HPP