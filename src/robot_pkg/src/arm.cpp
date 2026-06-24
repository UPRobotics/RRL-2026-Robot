#include "robot_pkg/arm.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <robot_msgs/msg/motor_config.hpp>
#include <fstream>
#include <algorithm>

ArmNode::ArmNode() : rclcpp::Node("arm_node") {
    
    hipMotor      = std::make_unique<RosVescMotor>(this, "Cadera", "/arm_hip/telemetry",      [this]() { this->loadConfig(); }); 
    shoulderMotor = std::make_unique<RosVescMotor>(this, "Hombro", "/arm_shoulder/telemetry",  [this]() { this->loadConfig(); }); 
    elbowMotor    = std::make_unique<RosVescMotor>(this, "Codo",   "/arm_elbow/telemetry",    [this]() { this->loadConfig(); }); 
    rollMotor     = std::make_unique<RosVescMotor>(this, "Roll",   "/arm_roll/telemetry",     [this]() { this->loadConfig(); }); 
    pitchMotor    = std::make_unique<RosVescMotor>(this, "Pitch",  "/arm_pitch/telemetry",    [this]() { this->loadConfig(); }); 
    clawMotor     = std::make_unique<RosVescMotor>(this, "Grip",   "/arm_claw/telemetry",     [this]() { this->loadConfig(); }); 

    loadConfig();
    
    hipMotor->autoConnectInit();
    shoulderMotor->autoConnectInit();
    elbowMotor->autoConnectInit();
    rollMotor->autoConnectInit();
    pitchMotor->autoConnectInit();
    clawMotor->autoConnectInit();

    rclcpp::QoS input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    sub_input_ = this->create_subscription<robot_msgs::msg::ControlInput>(
        "/control/input", input_qos,
        [this](const robot_msgs::msg::ControlInput::SharedPtr msg) { this->onControlInput(msg); }
    );

    // Subscribe to config updates relayed to robot
    rclcpp::QoS config_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    config_sub_ = this->create_subscription<robot_msgs::msg::MotorConfig>(
        "/robot_config/update", config_qos,
        [this](const robot_msgs::msg::MotorConfig::SharedPtr msg) { this->handleMotorConfig(msg); }
    );
}

ArmNode::~ArmNode() {}

void ArmNode::onControlInput(const robot_msgs::msg::ControlInput::SharedPtr msg) {
    if (msg->mode == 1) {
        
        float hip_cmd      = msg->left_x;
        float shoulder_cmd = msg->left_y;
        float elbow_cmd    = msg->right_y;
        float roll_cmd     = msg->right_x;
        float pitch_cmd    = std::clamp(msg->trigger_right - msg->trigger_left, -1.0f, 1.0f);
        float claw_cmd     = msg->bumper_right ? 1.0f : (msg->bumper_left ? -1.0f : 0.0f);

        constexpr float DEADZONE = 0.1f;
        if (std::abs(hip_cmd)      < DEADZONE) hip_cmd      = 0.0f;
        if (std::abs(shoulder_cmd) < DEADZONE) shoulder_cmd = 0.0f;
        if (std::abs(elbow_cmd)    < DEADZONE) elbow_cmd    = 0.0f;
        if (std::abs(roll_cmd)     < DEADZONE) roll_cmd     = 0.0f;
        if (std::abs(pitch_cmd)    < DEADZONE) pitch_cmd    = 0.0f;
        if (std::abs(claw_cmd)     < DEADZONE) claw_cmd     = 0.0f;

        hipMotor->set(hip_cmd);
        shoulderMotor->set(shoulder_cmd);
        elbowMotor->set(elbow_cmd);
        rollMotor->set(roll_cmd);
        pitchMotor->set(pitch_cmd);
        clawMotor->set(claw_cmd);
    }
    else {
        hipMotor->set(0.0f);
        shoulderMotor->set(0.0f);
        elbowMotor->set(0.0f);
        rollMotor->set(0.0f);
        pitchMotor->set(0.0f);
        clawMotor->set(0.0f);
    }
}

void ArmNode::handleMotorConfig(const robot_msgs::msg::MotorConfig::SharedPtr msg) {
    try {
        int idx = static_cast<int>(msg->config_index);
        // Only apply to arm indices (4-9)
        if (idx < 4 || idx > 9) return;

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
            case 4: s = hipMotor->getSettings(); break;
            case 5: s = shoulderMotor->getSettings(); break;
            case 6: s = elbowMotor->getSettings(); break;
            case 7: s = rollMotor->getSettings(); break;
            case 8: s = pitchMotor->getSettings(); break;
            case 9: s = clawMotor->getSettings(); break;
            default: break;
        }
        s.vesc_id = msg->motor_vesc_id;
        s.rpm_limit = msg->rpm_limit;
        s.duty_cycle_limit = msg->duty_cycle_limit;
        s.control_mode = msg->control_mode;
        s.inverted = msg->inverted;
        s.enabled = msg->enabled;

        switch (idx) {
            case 4: hipMotor->applySettings(s); break;
            case 5: shoulderMotor->applySettings(s); break;
            case 6: elbowMotor->applySettings(s); break;
            case 7: rollMotor->applySettings(s); break;
            case 8: pitchMotor->applySettings(s); break;
            case 9: clawMotor->applySettings(s); break;
            default: break;
        }

        RCLCPP_INFO(this->get_logger(), "Applied MotorConfig for index %d and saved to disk.", idx);
    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Failed to apply incoming MotorConfig message");
    }
}

void ArmNode::loadConfig() {
    try {
        config_path_ = ament_index_cpp::get_package_share_directory("robot_pkg") + "/config/config.json";
        std::ifstream f(config_path_);
        if (!f.is_open()) return;
        
        config_data_ = nlohmann::json::parse(f);
        auto& motors = config_data_["motors"];
        
        auto readByName = [&](const std::string& target_name, const std::unique_ptr<RosVescMotor>& m) {
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

        readByName("Cadera", hipMotor);  
        readByName("Hombro", shoulderMotor);          
        readByName("Codo",   elbowMotor);         
        readByName("Roll",   rollMotor); 
        readByName("Pitch",  pitchMotor); 
        readByName("Grip",   clawMotor); 

    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON configuration file inside Arm Node.");
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    
    rclcpp::shutdown();
    return 0;
}
