#ifndef ARM_NODE_HPP
#define ARM_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_pkg/ros_vesc_motor.hpp"
#include "robot_msgs/msg/control_input.hpp"
#include <robot_msgs/msg/motor_config.hpp>
#include "robot_pkg/json.hpp"
#include <memory>
#include <string>

class ArmNode : public rclcpp::Node {
public:
    ArmNode();
    ~ArmNode();

private:
    void onControlInput(const robot_msgs::msg::ControlInput::SharedPtr msg);
    void loadConfig();

    std::unique_ptr<RosVescMotor> hipMotor;
    std::unique_ptr<RosVescMotor> shoulderMotor;
    std::unique_ptr<RosVescMotor> elbowMotor;
    std::unique_ptr<RosVescMotor> rollMotor;
    std::unique_ptr<RosVescMotor> pitchMotor;
    std::unique_ptr<RosVescMotor> clawMotor;

    rclcpp::Subscription<robot_msgs::msg::ControlInput>::SharedPtr sub_input_;
    rclcpp::Subscription<robot_msgs::msg::MotorConfig>::SharedPtr config_sub_;
    std::string config_path_;
    nlohmann::json config_data_;
    void handleMotorConfig(const robot_msgs::msg::MotorConfig::SharedPtr msg);
};

#endif