#include "robot_pkg/body.hpp"
#include "robot_pkg/ros_vesc_motor.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <robot_msgs/msg/motor_config.hpp>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>

BodyNode::BodyNode() : rclcpp::Node("single_motor_node") {
    
    body_left_flipper_  = std::make_unique<RosVescMotor>(this, "Flipper Delantero", "/body_left_flipper/telemetry",  [this]() { this->loadConfig(); }); 
    body_left_          = std::make_unique<RosVescMotor>(this, "Track Izquierdo",    "/body_left/telemetry",          [this]() { this->loadConfig(); }); 
    body_right_         = std::make_unique<RosVescMotor>(this, "Track Derecho",      "/body_right/telemetry",         [this]() { this->loadConfig(); }); 
    body_right_flipper_ = std::make_unique<RosVescMotor>(this, "Flipper Trasero",   "/body_right_flipper/telemetry", [this]() { this->loadConfig(); }); 

    loadConfig();
    VESC::setSuppressLogs(true);

    body_left_flipper_->autoConnectInit();
    body_left_->autoConnectInit();
    body_right_->autoConnectInit();
    body_right_flipper_->autoConnectInit();

    rclcpp::QoS input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    sub_input_ = this->create_subscription<robot_msgs::msg::ControlInput>(
        "/control/input", input_qos,
        [this](const robot_msgs::msg::ControlInput::SharedPtr msg) { this->onControlInput(msg); }
    );

    // Declare autonomous/cmd_vel parameters (defaults match cmd_vel_to_control)
    max_linear_  = this->declare_parameter<float>("max_linear_vel",  0.5f);
    max_angular_ = this->declare_parameter<float>("max_angular_vel", 1.0f);
    RCLCPP_INFO(this->get_logger(), "BodyNode ready. max_linear=%.2f max_angular=%.2f is_autonomous=%s",
                max_linear_, max_angular_, is_autonomous_ ? "true" : "false");

    rclcpp::QoS config_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    config_sub_ = this->create_subscription<robot_msgs::msg::MotorConfig>(
        "/robot_config/update", config_qos,
        [this](const robot_msgs::msg::MotorConfig::SharedPtr msg) { this->handleMotorConfig(msg); }
    );

    sub_autonomy_ = this->create_subscription<std_msgs::msg::Bool>(
        "/ground_station/is_autonomous", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {

            if(msg->data == true && joystick_override == false){
                is_autonomous_.store(msg->data);
                RCLCPP_INFO(this->get_logger(), "is_autonomous set to %s via topic", msg->data ? "true" : "false");

            } else if(msg->data == false){
                joystick_override.store(false);
                is_autonomous_.store(msg->data);

            }
        }
    );

    sub_CmdVel = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                onCmdVel(msg);
            });

}

BodyNode::~BodyNode() {}

void BodyNode::onControlInput(const robot_msgs::msg::ControlInput::SharedPtr msg) {
    float dx = msg->dpad_x;
    float dy = msg->dpad_y;
    //RCLCPP_WARN(this->get_logger(),"The status is: %s\n", joystick_override ? "true" : "false");

    if (is_autonomous_ && (msg->left_y == 0.0f && msg->left_x == 0.0f)) {

        RCLCPP_WARN(this->get_logger(), "Is Auto");
        shouldKillSystems.store(true);
        return;
    }

    if(msg->left_y != 0.0f || msg->left_x != 0.0f){
        joystick_override.store(true);
        is_autonomous_.store(false);
    }
            RCLCPP_WARN(this->get_logger(), "Is teleop");

    if (dx != 0.0f || dy != 0.0f) {

        float left_cmd  = std::clamp(dy + dx, -1.0f, 1.0f);
        float right_cmd = std::clamp(dy - dx, -1.0f, 1.0f);

        body_left_->setWithCustomLimits(left_cmd, 1500.0f, 0.30f);
        body_right_->setWithCustomLimits(right_cmd, 1500.0f, 0.30f);

        body_left_flipper_->set(0.0f);
        body_right_flipper_->set(0.0f);
    }
    else if (msg->mode == 0) {

        if(shouldKillSystems){
            executeSystemNukeAndReset();
        }

        float left_cmd  = std::clamp(msg->left_y + msg->left_x, -1.0f, 1.0f);
        float right_cmd = std::clamp(msg->left_y - msg->left_x, -1.0f, 1.0f);


        body_left_->set(left_cmd);
        body_right_->set(right_cmd);


        body_left_flipper_->set(msg->right_y);
        body_right_flipper_->set(msg->right_x);
    }

    else {
        body_left_->set(0.0f);
        body_right_->set(0.0f);
        body_left_flipper_->set(0.0f);
        body_right_flipper_->set(0.0f);
    }
}


void BodyNode::loadConfig() {
    try {
        config_path_ = ament_index_cpp::get_package_share_directory("robot_pkg") + "/config/config.json";
        std::ifstream f(config_path_);
        if (!f.is_open()) return;
        
        config_data_ = nlohmann::json::parse(f);
        // Read global settings
        if (config_data_.contains("global_settings")) {
            is_autonomous_ = config_data_["global_settings"].value("is_autonomous", false);
        }
        auto& motors = config_data_["motors"];
        
        auto readByName = [&](const std::string& target_name, RosVescMotor* m) {
            for (const auto& motor_entry : motors) {
                if (motor_entry.value("name", "") == target_name) {
                    MotorSettings s = m->getSettings();
                    s.vesc_id          = motor_entry.value("id", static_cast<int>(s.vesc_id));
                    s.rpm_limit        = motor_entry.value("rpm_limit", s.rpm_limit);
                    s.duty_cycle_limit = motor_entry.value("duty_cycle_limit", s.duty_cycle_limit);
                    s.control_mode     = motor_entry.value("control_mode", static_cast<int>(s.control_mode));
                    s.inverted         = motor_entry.value("inverted", s.inverted);
                    s.enabled          = motor_entry.value("enabled", s.enabled);
                    s.port             = motor_entry.value("port", s.port);
                    m->applySettings(s);
                    return; 
                }
            }
            RCLCPP_WARN(this->get_logger(), "Could not find config settings for motor name: %s", target_name.c_str());
        };

        readByName("Flipper Delantero", body_left_flipper_.get());  
        readByName("Track Izquierdo",    body_left_.get());          
        readByName("Track Derecho",      body_right_.get());         
        readByName("Flipper Trasero",   body_right_flipper_.get()); 

    } catch (...) {}
}

void BodyNode::handleMotorConfig(const robot_msgs::msg::MotorConfig::SharedPtr msg) {
    try {
        int idx = static_cast<int>(msg->config_index);
        // Only apply to body indices (0-3)
        if (idx < 0 || idx > 3) return;

        config_path_ = ament_index_cpp::get_package_share_directory("robot_pkg") + "/config/config.json";
        std::ifstream f(config_path_);
        if (!f.is_open()) return;
        auto j = nlohmann::json::parse(f);
        f.close();

        auto& motors = j["motors"];
        if (idx >= static_cast<int>(motors.size())) return;

        motors[idx]["id"] = msg->motor_vesc_id;
        motors[idx]["name"] = msg->motor_name;
        motors[idx]["rpm_limit"] = msg->rpm_limit;
        motors[idx]["duty_cycle_limit"] = msg->duty_cycle_limit;
        motors[idx]["control_mode"] = msg->control_mode;
        motors[idx]["inverted"] = msg->inverted;
        motors[idx]["enabled"] = msg->enabled;

        std::ofstream out(config_path_, std::ofstream::trunc);
        if (out.is_open()) {
            out << j.dump(2);
            out.close();
        }

        // Apply to in-memory motor
        MotorSettings s;
        switch (idx) {
            case 0: s = body_left_flipper_->getSettings(); break;
            case 1: s = body_left_->getSettings(); break;
            case 2: s = body_right_->getSettings(); break;
            case 3: s = body_right_flipper_->getSettings(); break;
            default: break;
        }
        s.vesc_id = msg->motor_vesc_id;
        s.rpm_limit = msg->rpm_limit;
        s.duty_cycle_limit = msg->duty_cycle_limit;
        s.control_mode = msg->control_mode;
        s.inverted = msg->inverted;
        s.enabled = msg->enabled;

        switch (idx) {
            case 0: body_left_flipper_->applySettings(s); break;
            case 1: body_left_->applySettings(s); break;
            case 2: body_right_->applySettings(s); break;
            case 3: body_right_flipper_->applySettings(s); break;
            default: break;
        }


        RCLCPP_INFO(this->get_logger(), "Applied MotorConfig for index %d and saved to disk.", idx);
    } catch (...) {
        // Ignore parsing/writing errors silently
    }
}

    void BodyNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!is_autonomous_) return;

                    RCLCPP_WARN(this->get_logger(), "Is Auto");

        float forward = std::clamp((float)(msg->linear.x / max_linear_), -1.0f, 1.0f);
        float turn = std::clamp((float)(-msg->angular.z / max_angular_), -1.0f, 1.0f);

        float left_cmd  = std::clamp(forward + turn, -1.0f, 1.0f);
        float right_cmd = std::clamp(forward - turn, -1.0f, 1.0f);

        body_left_->set(left_cmd);
        body_right_->set(right_cmd);

        body_left_flipper_->set(0.0f);
        body_right_flipper_->set(0.0f);
    }



    void BodyNode::executeSystemNukeAndReset(){
    RCLCPP_WARN(this->get_logger(),
        "Aborting autonomy and switching to teleop.");

    // Stop autonomy immediately
    is_autonomous_.store(false);
    joystick_override.store(true);

    // Kill mapping/localization stack
    RCLCPP_INFO(this->get_logger(),
        "Stopping LiDAR and SLAM processes...");

    std::system("killall -9 livox_ros_driver2_node 2>/dev/null");
    std::system("pkill -9 -f 'fastlio_pkg mapping.launch.py'");
    std::system("killall -9 octomap_server_node 2>/dev/null");
    std::system("killall -9 tracking_octomap_server_node 2>/dev/null");

    RCLCPP_INFO(this->get_logger(),
        "Clearing Nav2 costmaps...");

    /*std::system(
        "ros2 service call "
        "/local_costmap/clear_entirely_local_costmap "
        "nav2_msgs/srv/ClearEntireCostmap "
        "'{}' > /dev/null 2>&1");

    std::system(
        "ros2 service call "
        "/global_costmap/clear_entirely_global_costmap "
        "nav2_msgs/srv/ClearEntireCostmap "
        "'{}' > /dev/null 2>&1");*/

    RCLCPP_INFO(this->get_logger(),
        "Autonomy session terminated.");

        shouldKillSystems.store(false);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    
    rclcpp::shutdown();
    return 0;
}