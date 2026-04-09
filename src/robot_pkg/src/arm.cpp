#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_pkg/VESC.hpp"
#include "robot_pkg/json.hpp"
#include "robot_msgs/msg/control_input.hpp"
#include "robot_msgs/msg/motor_config.hpp"
#include "robot_msgs/msg/motor_telemetry.hpp"

using namespace std::chrono_literals;
using namespace LibSerial;

static const rclcpp::QoS CONTROL_QOS = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();

// QoS para telemetría: BEST_EFFORT evita retransmisiones innecesarias
static const rclcpp::QoS TELEM_QOS = rclcpp::QoS(rclcpp::KeepLast(5))
    .best_effort()
    .durability_volatile();

static constexpr float DEADZONE = 0.1f;

struct MotorSettings {
    uint8_t vesc_id          = 0;
    float   rpm_limit        = 5000.0f;
    float   duty_cycle_limit = 1.0f;
    uint8_t control_mode     = 0;    // 0 = RPM mode, 1 = duty cycle mode
    bool    inverted         = false;
};

class ArmNode : public rclcpp::Node
{
public:

    int hipMotorId = 4;
    int shoulderMotorId = 5;
    int elbowMotorId = 6;
    int rollMotorId = 7;
    int pitchMotorId = 8;
    int clawMotorId= 9;



ArmNode() : Node("arm_node"),
                hipMotor(
                    declare_parameter<uint8_t>("hip_id", hipMotorId),
                    declare_parameter<int>("hip_baudrate", 115200),
                    declare_parameter<int>("hip_timeout", 1000)),
                shoulderMotor(
                    declare_parameter<uint8_t>("shoulder_id", shoulderMotorId),
                    declare_parameter<int>("shoulder_baudrate", 115200),
                    declare_parameter<int>("shoulder_timeout", 1000)),
                elbowMotor(
                    declare_parameter<uint8_t>("elbow_id", elbowMotorId),
                    declare_parameter<int>("elbow_baudrate", 115200),
                    declare_parameter<int>("elbow_timeout", 1000)),
                rollMotor(
                    declare_parameter<uint8_t>("roll_id", rollMotorId),
                    declare_parameter<int>("roll_baudrate", 115200),
                    declare_parameter<int>("roll_timeout", 1000)),
                pitchMotor(
                    declare_parameter<uint8_t>("pitch_id", pitchMotorId),
                    declare_parameter<int>("pitch_baudrate", 115200),
                    declare_parameter<int>("pitch_timeout", 1000)),
                clawMotor(
                    declare_parameter<uint8_t>("claw_id", clawMotorId),
                    declare_parameter<int>("claw_baudrate", 115200),
                    declare_parameter<int>("claw_timeout", 1000)
                )                                        
    {

        loadConfig();
        hipMotor.setId(hip_.vesc_id);
        shoulderMotor.setId(shoulder_.vesc_id);
        elbowMotor.setId(elbow_.vesc_id);
        rollMotor.setId(roll_.vesc_id);
        pitchMotor.setId(pitch_.vesc_id);
        clawMotor.setId(claw_.vesc_id);

        if (hipMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Hip motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Hip motor.");
        }
        if (shoulderMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Shoulder motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Shoulder motor.");
        }
        if (elbowMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Elbow motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Elbow motor.");
        }
        if (rollMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Roll motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Roll motor.");
        }
        if (pitchMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Pitch motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Pitch motor.");
        }
        if (clawMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Claw motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Claw motor.");
        }

        config_update_sub_ = create_subscription<robot_msgs::msg::MotorConfig>(
            "/robot_config/update", 10,
            [this](const robot_msgs::msg::MotorConfig::SharedPtr msg) {
                onConfigUpdate(msg);
            });

        callback_group_hip_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_shoulder_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_roll_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_pitch_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_claw_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_elbow_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 

        timer_hip_ = this->create_wall_timer(
            10ms, bind(&ArmNode::setHipRPM, this), callback_group_hip_);
            
        timer_shoulder_ = this->create_wall_timer(
            10ms, bind(&ArmNode::setShoulderRPM, this), callback_group_shoulder_);

        timer_elbow_ = this->create_wall_timer(
            10ms, bind(&ArmNode::setElbowRPM, this), callback_group_elbow_);
            
        timer_roll_ = this->create_wall_timer(
            10ms, bind(&ArmNode::setRollRPM, this), callback_group_roll_);
            
        timer_pitch_ = this->create_wall_timer(
            10ms, bind(&ArmNode::setPitchRPM, this), callback_group_pitch_);
            
        timer_claw_ = this->create_wall_timer(
            10ms, bind(&ArmNode::setClawRPM, this), callback_group_claw_);
        
        // Control input: mode-aware arm mapping (ARM mode only)
        //   left_y  → hip         left_x  → shoulder
        //   right_y → elbow       right_x → roll
        //   trigger_right - trigger_left → pitch
        sub_axes_ = create_subscription<robot_msgs::msg::ControlInput>(
            "/control/input", CONTROL_QOS,
            [this](const robot_msgs::msg::ControlInput::SharedPtr msg) {
                control_mode_.store(msg->mode, std::memory_order_relaxed);
                if (msg->mode == 1) {  // ARM mode
                    target_hip_.store(msg->left_y,      std::memory_order_relaxed);
                    target_shoulder_.store(msg->left_x,  std::memory_order_relaxed);
                    target_elbow_.store(msg->right_y,    std::memory_order_relaxed);
                    target_roll_.store(msg->right_x,     std::memory_order_relaxed);
                    target_pitch_.store(
                        std::clamp(msg->trigger_right - msg->trigger_left, -1.0f, 1.0f),
                        std::memory_order_relaxed);
                } else {  // MOVEMENT mode: zero all arm targets
                    target_hip_.store(0.0f,      std::memory_order_relaxed);
                    target_shoulder_.store(0.0f, std::memory_order_relaxed);
                    target_elbow_.store(0.0f,    std::memory_order_relaxed);
                    target_roll_.store(0.0f,     std::memory_order_relaxed);
                    target_pitch_.store(0.0f,    std::memory_order_relaxed);
                }
            });

        hip_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_hip/telemetry", TELEM_QOS);
        shoulder_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_shoulder/telemetry", TELEM_QOS);
        elbow_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_elbow/telemetry", TELEM_QOS);
        roll_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_roll/telemetry", TELEM_QOS);
        pitch_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_pitch/telemetry", TELEM_QOS);
        claw_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_claw/telemetry", TELEM_QOS);

        // setup telemetry timer
        callback_group_telemetry_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        telemetry_timer_ = this->create_wall_timer(
            50ms, bind(&ArmNode::telemetry, this), callback_group_telemetry_);
    }

    ~ArmNode()
    {
        hipMotor.disconnect();
        shoulderMotor.disconnect();
        elbowMotor.disconnect();
        rollMotor.disconnect();
        pitchMotor.disconnect();
        clawMotor.disconnect();
    }

    // normalized: -1.0 = full reverse, 0.0 = stop, 1.0 = full forward
    void setHipTarget(float normalized)      { target_hip_.store(normalized); }
    void setShoulderTarget(float normalized) { target_shoulder_.store(normalized); }
    void setElbowTarget(float normalized)    { target_elbow_.store(normalized); }
    void setRollTarget(float normalized)     { target_roll_.store(normalized); }
    void setPitchTarget(float normalized)    { target_pitch_.store(normalized); }
    void setClawTarget(float normalized)     { target_claw_.store(normalized); }

private:
    void setHipRPM() {
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = hip_.rpm_limit; duty_lim = hip_.duty_cycle_limit; inv = hip_.inverted; mode = hip_.control_mode; }
        float cmd = std::clamp(target_hip_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { hipMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { hipMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setShoulderRPM() {
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = shoulder_.rpm_limit; duty_lim = shoulder_.duty_cycle_limit; inv = shoulder_.inverted; mode = shoulder_.control_mode; }
        float cmd = std::clamp(target_shoulder_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { shoulderMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { shoulderMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setElbowRPM() {
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = elbow_.rpm_limit; duty_lim = elbow_.duty_cycle_limit; inv = elbow_.inverted; mode = elbow_.control_mode; }
        float cmd = std::clamp(target_elbow_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { elbowMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { elbowMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setRollRPM() {
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = roll_.rpm_limit; duty_lim = roll_.duty_cycle_limit; inv = roll_.inverted; mode = roll_.control_mode; }
        float cmd = std::clamp(target_roll_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { rollMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { rollMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setPitchRPM() {
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = pitch_.rpm_limit; duty_lim = pitch_.duty_cycle_limit; inv = pitch_.inverted; mode = pitch_.control_mode; }
        float cmd = std::clamp(target_pitch_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { pitchMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { pitchMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setClawRPM() {
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = claw_.rpm_limit; duty_lim = claw_.duty_cycle_limit; inv = claw_.inverted; mode = claw_.control_mode; }
        float cmd = std::clamp(target_claw_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { clawMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { clawMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void telemetry(){
        VESCData m_telemetry;
        MotorSettings h, sh, el, ro, pi, cl;
        {
            std::lock_guard<std::mutex> lk(settings_mutex_);
            h = hip_; sh = shoulder_; el = elbow_;
            ro = roll_; pi = pitch_; cl = claw_;
        }

        if(hipMotor.isConnected() && hipMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Cadera";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.current_in   = m_telemetry.current_in;
            msg.voltage      = m_telemetry.input_voltage;
            msg.position     = m_telemetry.position;
            msg.fault_code   = m_telemetry.fault_code;
            msg.control_mode     = h.control_mode;
            msg.inverted         = h.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = h.rpm_limit;
            msg.duty_cycle_limit = h.duty_cycle_limit;
            hip_telemetry_pub->publish(msg);
        }

        if(shoulderMotor.isConnected() && shoulderMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m_telemetry.motor_controller_id;
            msg.motor_name       = "Hombro";
            msg.rpm              = m_telemetry.rpm;
            msg.duty_cycle       = m_telemetry.duty_cycle;
            msg.current_in       = m_telemetry.current_in;
            msg.voltage          = m_telemetry.input_voltage;
            msg.position         = m_telemetry.position;
            msg.fault_code       = m_telemetry.fault_code;
            msg.control_mode     = sh.control_mode;
            msg.inverted         = sh.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = sh.rpm_limit;
            msg.duty_cycle_limit = sh.duty_cycle_limit;
            shoulder_telemetry_pub->publish(msg);
        }

        if(elbowMotor.isConnected() && elbowMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m_telemetry.motor_controller_id;
            msg.motor_name       = "Codo";
            msg.rpm              = m_telemetry.rpm;
            msg.duty_cycle       = m_telemetry.duty_cycle;
            msg.current_in       = m_telemetry.current_in;
            msg.voltage          = m_telemetry.input_voltage;
            msg.position         = m_telemetry.position;
            msg.fault_code       = m_telemetry.fault_code;
            msg.control_mode     = el.control_mode;
            msg.inverted         = el.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = el.rpm_limit;
            msg.duty_cycle_limit = el.duty_cycle_limit;
            elbow_telemetry_pub->publish(msg);
        }

        if(rollMotor.isConnected() && rollMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m_telemetry.motor_controller_id;
            msg.motor_name       = "Roll";
            msg.rpm              = m_telemetry.rpm;
            msg.duty_cycle       = m_telemetry.duty_cycle;
            msg.current_in       = m_telemetry.current_in;
            msg.voltage          = m_telemetry.input_voltage;
            msg.position         = m_telemetry.position;
            msg.fault_code       = m_telemetry.fault_code;
            msg.control_mode     = ro.control_mode;
            msg.inverted         = ro.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = ro.rpm_limit;
            msg.duty_cycle_limit = ro.duty_cycle_limit;
            roll_telemetry_pub->publish(msg);
        }

        if(pitchMotor.isConnected() && pitchMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m_telemetry.motor_controller_id;
            msg.motor_name       = "Pitch";
            msg.rpm              = m_telemetry.rpm;
            msg.duty_cycle       = m_telemetry.duty_cycle;
            msg.current_in       = m_telemetry.current_in;
            msg.voltage          = m_telemetry.input_voltage;
            msg.position         = m_telemetry.position;
            msg.fault_code       = m_telemetry.fault_code;
            msg.control_mode     = pi.control_mode;
            msg.inverted         = pi.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = pi.rpm_limit;
            msg.duty_cycle_limit = pi.duty_cycle_limit;
            pitch_telemetry_pub->publish(msg);
        }

        if(clawMotor.isConnected() && clawMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m_telemetry.motor_controller_id;
            msg.motor_name       = "Grip";
            msg.rpm              = m_telemetry.rpm;
            msg.duty_cycle       = m_telemetry.duty_cycle;
            msg.current_in       = m_telemetry.current_in;
            msg.voltage          = m_telemetry.input_voltage;
            msg.position         = m_telemetry.position;
            msg.fault_code       = m_telemetry.fault_code;
            msg.control_mode     = cl.control_mode;
            msg.inverted         = cl.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = cl.rpm_limit;
            msg.duty_cycle_limit = cl.duty_cycle_limit;
            claw_telemetry_pub->publish(msg);
        }
    }

    // ---- Config helpers ----
    void loadConfig() {
        try {
            config_path_ = ament_index_cpp::get_package_share_directory("robot_pkg")
                           + "/config/config.json";
            std::ifstream f(config_path_);
            if (!f.is_open()) {
                RCLCPP_WARN(get_logger(), "Config not found at %s, using defaults", config_path_.c_str());
                return;
            }
            config_data_ = nlohmann::json::parse(f);
            auto& motors = config_data_["motors"];
            auto read = [&](int idx, MotorSettings& s) {
                if (idx < static_cast<int>(motors.size())) {
                    s.vesc_id          = static_cast<uint8_t>(motors[idx].value("id",               static_cast<int>(s.vesc_id)));
                    s.rpm_limit        = motors[idx].value("rpm_limit",        s.rpm_limit);
                    s.duty_cycle_limit = motors[idx].value("duty_cycle_limit", s.duty_cycle_limit);
                    s.control_mode     = static_cast<uint8_t>(motors[idx].value("control_mode", static_cast<int>(s.control_mode)));
                    s.inverted         = motors[idx].value("inverted",         s.inverted);
                }
            };
            std::lock_guard<std::mutex> lk(settings_mutex_);
            read(4, hip_); read(5, shoulder_); read(6, elbow_);
            read(7, roll_); read(8, pitch_);   read(9, claw_);
            RCLCPP_INFO(get_logger(), "Config loaded from %s",
                std::filesystem::canonical(config_path_).string().c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "loadConfig failed: %s", e.what());
        }
    }

    void saveConfig() {
        try {
            auto apply = [&](int idx, const MotorSettings& s) {
                config_data_["motors"][idx]["id"]               = static_cast<int>(s.vesc_id);
                config_data_["motors"][idx]["rpm_limit"]        = s.rpm_limit;
                config_data_["motors"][idx]["duty_cycle_limit"] = s.duty_cycle_limit;
                config_data_["motors"][idx]["control_mode"]     = static_cast<int>(s.control_mode);
                config_data_["motors"][idx]["inverted"]         = s.inverted;
            };
            MotorSettings h, sh, el, ro, pi, cl;
            {
                std::lock_guard<std::mutex> lk(settings_mutex_);
                h = hip_; sh = shoulder_; el = elbow_;
                ro = roll_; pi = pitch_; cl = claw_;
            }
            apply(4, h); apply(5, sh); apply(6, el);
            apply(7, ro); apply(8, pi); apply(9, cl);
            std::ofstream f(config_path_);
            f << config_data_.dump(2);
            RCLCPP_INFO(get_logger(), "Config saved.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "saveConfig failed: %s", e.what());
        }
    }

    void onConfigUpdate(const robot_msgs::msg::MotorConfig::SharedPtr msg) {
        MotorSettings* t = nullptr;
        VESC*          v = nullptr;
        switch (msg->config_index) {
            case 4: t = &hip_;      v = &hipMotor;      break;
            case 5: t = &shoulder_; v = &shoulderMotor; break;
            case 6: t = &elbow_;    v = &elbowMotor;    break;
            case 7: t = &roll_;     v = &rollMotor;     break;
            case 8: t = &pitch_;    v = &pitchMotor;    break;
            case 9: t = &claw_;     v = &clawMotor;     break;
            default: return;  // Not an arm motor
        }
        {
            std::lock_guard<std::mutex> lk(settings_mutex_);
            t->vesc_id          = msg->motor_vesc_id;
            t->rpm_limit        = msg->rpm_limit;
            t->duty_cycle_limit = msg->duty_cycle_limit;
            t->control_mode     = msg->control_mode;
            t->inverted         = msg->inverted;
            v->setId(msg->motor_vesc_id);
        }
        RCLCPP_INFO(get_logger(),
            "Config update [%u] %s: vesc_id=%u  rpm_limit=%.1f  duty=%.3f  mode=%u  inv=%s",
            msg->config_index, msg->motor_name.c_str(),
            msg->motor_vesc_id, msg->rpm_limit, msg->duty_cycle_limit, msg->control_mode,
            msg->inverted ? "yes" : "no");
        saveConfig();
    }

    VESC hipMotor;  //Cadera
    VESC shoulderMotor; // hombro
    VESC elbowMotor; //COdo
    VESC rollMotor;
    VESC pitchMotor;
    VESC clawMotor;

    rclcpp::CallbackGroup::SharedPtr callback_group_hip_;
    rclcpp::CallbackGroup::SharedPtr callback_group_shoulder_;
    rclcpp::CallbackGroup::SharedPtr callback_group_elbow_;
    rclcpp::CallbackGroup::SharedPtr callback_group_roll_;
    rclcpp::CallbackGroup::SharedPtr callback_group_pitch_;
    rclcpp::CallbackGroup::SharedPtr callback_group_claw_;

    rclcpp::TimerBase::SharedPtr timer_hip_;
    rclcpp::TimerBase::SharedPtr timer_shoulder_;
    rclcpp::TimerBase::SharedPtr timer_elbow_;
    rclcpp::TimerBase::SharedPtr timer_roll_;
    rclcpp::TimerBase::SharedPtr timer_pitch_;
    rclcpp::TimerBase::SharedPtr timer_claw_;

    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr hip_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr shoulder_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr elbow_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr roll_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr pitch_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr claw_telemetry_pub;


    std::atomic<float>   target_hip_{0.0f};
    std::atomic<float>   target_shoulder_{0.0f};
    std::atomic<float>   target_elbow_{0.0f};
    std::atomic<float>   target_roll_{0.0f};
    std::atomic<float>   target_pitch_{0.0f};
    std::atomic<float>   target_claw_{0.0f};
    std::atomic<uint8_t> control_mode_{0};  // 0=MOVEMENT, 1=ARM


    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telemetry_;

    rclcpp::Subscription<robot_msgs::msg::ControlInput>::SharedPtr sub_axes_;

    // ---- Config state ----
    std::string         config_path_;
    nlohmann::json      config_data_;
    std::mutex          settings_mutex_;
    MotorSettings       hip_, shoulder_, elbow_, roll_, pitch_, claw_;
    rclcpp::Subscription<robot_msgs::msg::MotorConfig>::SharedPtr config_update_sub_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    rclcpp::shutdown();
    return 0;
}