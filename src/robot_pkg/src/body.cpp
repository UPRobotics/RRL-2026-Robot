#include "robot_pkg/body.hpp"
#include "robot_pkg/ros_vesc_motor.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <algorithm>

BodyNode::BodyNode() : rclcpp::Node("single_motor_node") {
    
    // Instantiated with precise, isolated hardware-level logging names and topics
    body_left_flipper_  = std::make_unique<RosVescMotor>(this, "Flipper Delantero", "/body_left_flipper/telemetry",  [this]() { this->loadConfig(); }); 
    body_left_          = std::make_unique<RosVescMotor>(this, "Track Izquierdo",    "/body_left/telemetry",          [this]() { this->loadConfig(); }); 
    body_right_         = std::make_unique<RosVescMotor>(this, "Track Derecho",      "/body_right/telemetry",         [this]() { this->loadConfig(); }); 
    body_right_flipper_ = std::make_unique<RosVescMotor>(this, "Flipper Trasero",   "/body_right_flipper/telemetry", [this]() { this->loadConfig(); }); 

    loadConfig();
    
    body_left_flipper_->autoConnectInit();
    body_left_->autoConnectInit();
    body_right_->autoConnectInit();
    body_right_flipper_->autoConnectInit();

    rclcpp::QoS input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    sub_input_ = this->create_subscription<robot_msgs::msg::ControlInput>(
        "/control/input", input_qos,
        [this](const robot_msgs::msg::ControlInput::SharedPtr msg) { this->onControlInput(msg); }
    );
}

BodyNode::~BodyNode() {}

void BodyNode::onControlInput(const robot_msgs::msg::ControlInput::SharedPtr msg) {
    float dx = msg->dpad_x;
    float dy = msg->dpad_y;

    // 1. D-PAD OVERRIDE MODE
    if (dx != 0.0f || dy != 0.0f) {
        // Tank mixing using your D-pad inputs
        float left_cmd  = std::clamp(dy + dx, -1.0f, 1.0f);
        float right_cmd = std::clamp(dy - dx, -1.0f, 1.0f);

        // Uses the wrapper's custom limit function to enforce your D-pad specific safety caps
        body_left_->setWithCustomLimits(left_cmd, 1500.0f, 0.30f);
        body_right_->setWithCustomLimits(right_cmd, 1500.0f, 0.30f);

        // Flippers are unmapped on D-pad control cycles, keep them at zero
        body_left_flipper_->set(0.0f);
        body_right_flipper_->set(0.0f);
    }
    // 2. NORMAL MOVEMENT MODE
    else if (msg->mode == 0) {

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
        auto& motors = config_data_["motors"];
        
        // Dynamic name-matching helper tool to cycle through your 10-motor array file securely
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

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    
    rclcpp::shutdown();
    return 0;
}