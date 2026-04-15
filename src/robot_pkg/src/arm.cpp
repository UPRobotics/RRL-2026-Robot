#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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
    uint8_t     vesc_id          = 0;
    float       rpm_limit        = 5000.0f;
    float       duty_cycle_limit = 1.0f;
    uint8_t     control_mode     = 0;    // 0 = RPM mode, 1 = duty cycle mode
    bool        inverted         = false;
    std::string port             = "";   // last known ttyACM port (used as hint for autoConnect)
};

class ArmNode : public rclcpp::Node
{
public:

    std::thread config_thread_;
    std::mutex config_mutex_;
    std::condition_variable config_cv_;
    bool save_requested_ = false;
    bool stop_thread_ = false;

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
        
        config_thread_ = std::thread([this]() {
        std::unique_lock<std::mutex> lock(config_mutex_);
        while (!stop_thread_) {
            config_cv_.wait(lock, [this]() { return save_requested_ || stop_thread_; });

            if (stop_thread_) break;

            save_requested_ = false;
            lock.unlock();

            saveConfig(); 

            lock.lock();
        }
    });

        if (hipMotor.autoConnect(hip_.port)){
            RCLCPP_INFO(this->get_logger(), "Hip motor connected.");
            hip_.port = hipMotor.getPortName(); save_config_();
        } else { RCLCPP_ERROR(this->get_logger(), "Failed to connect to Hip motor."); }
        if (shoulderMotor.autoConnect(shoulder_.port)){
            RCLCPP_INFO(this->get_logger(), "Shoulder motor connected.");
            {
                std::lock_guard<std::mutex> lk(settings_mutex_);
                shoulder_.port = shoulderMotor.getPortName();
            }
            save_config_();
        } else { RCLCPP_ERROR(this->get_logger(), "Failed to connect to Shoulder motor."); }
        if (elbowMotor.autoConnect(elbow_.port)){
            RCLCPP_INFO(this->get_logger(), "Elbow motor connected.");
            {
                std::lock_guard<std::mutex> lk(settings_mutex_);
                elbow_.port = elbowMotor.getPortName();
            }
            save_config_();
        } else { RCLCPP_ERROR(this->get_logger(), "Failed to connect to Elbow motor."); }
        if (rollMotor.autoConnect(roll_.port)){
            RCLCPP_INFO(this->get_logger(), "Roll motor connected.");
            {
                std::lock_guard<std::mutex> lk(settings_mutex_);
                roll_.port = rollMotor.getPortName();
            } 
            save_config_();
        } else { RCLCPP_ERROR(this->get_logger(), "Failed to connect to Roll motor."); }
        if (pitchMotor.autoConnect(pitch_.port)){
            RCLCPP_INFO(this->get_logger(), "Pitch motor connected.");
            {
                std::lock_guard<std::mutex> lk(settings_mutex_);
                pitch_.port = pitchMotor.getPortName();
            } 
            save_config_();
        } else { RCLCPP_ERROR(this->get_logger(), "Failed to connect to Pitch motor."); }
        if (clawMotor.autoConnect(claw_.port)){
            RCLCPP_INFO(this->get_logger(), "Claw motor connected.");
            {
                std::lock_guard<std::mutex> lk(settings_mutex_);
                claw_.port = clawMotor.getPortName();
            } 
            save_config_();
        } else { RCLCPP_ERROR(this->get_logger(), "Failed to connect to Claw motor."); }

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
                last_input_ns_.store(
                    std::chrono::steady_clock::now().time_since_epoch().count(),
                    std::memory_order_relaxed);
                control_mode_.store(msg->mode, std::memory_order_relaxed);
                if (msg->mode == 1) {  // ARM mode
                    target_hip_.store(msg->left_y,      std::memory_order_relaxed);
                    target_shoulder_.store(msg->left_x,  std::memory_order_relaxed);
                    target_elbow_.store(msg->right_y,    std::memory_order_relaxed);
                    target_roll_.store(msg->right_x,     std::memory_order_relaxed);
                    target_pitch_.store(
                        std::clamp(msg->trigger_right - msg->trigger_left, -1.0f, 1.0f),
                        std::memory_order_relaxed);
                    target_claw_.store(
                        msg->bumper_right ? 1.0f : (msg->bumper_left ? -1.0f : 0.0f),
                        std::memory_order_relaxed);
                } else {  // MOVEMENT mode: zero all arm targets
                    target_hip_.store(0.0f,      std::memory_order_relaxed);
                    target_shoulder_.store(0.0f, std::memory_order_relaxed);
                    target_elbow_.store(0.0f,    std::memory_order_relaxed);
                    target_roll_.store(0.0f,     std::memory_order_relaxed);
                    target_pitch_.store(0.0f,    std::memory_order_relaxed);
                    target_claw_.store(0.0f,     std::memory_order_relaxed);
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

        // One callback group + timer per motor so each serial read is independent
        callback_group_telem_hip_      = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_telem_shoulder_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_telem_elbow_    = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_telem_roll_     = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_telem_pitch_    = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_telem_claw_     = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        telemetry_timer_hip_      = create_wall_timer(50ms, bind(&ArmNode::telemetryHip,      this), callback_group_telem_hip_);
        telemetry_timer_shoulder_ = create_wall_timer(50ms, bind(&ArmNode::telemetryShoulder, this), callback_group_telem_shoulder_);
        telemetry_timer_elbow_    = create_wall_timer(50ms, bind(&ArmNode::telemetryElbow,    this), callback_group_telem_elbow_);
        telemetry_timer_roll_     = create_wall_timer(50ms, bind(&ArmNode::telemetryRoll,     this), callback_group_telem_roll_);
        telemetry_timer_pitch_    = create_wall_timer(50ms, bind(&ArmNode::telemetryPitch,    this), callback_group_telem_pitch_);
        telemetry_timer_claw_     = create_wall_timer(50ms, bind(&ArmNode::telemetryClaw,     this), callback_group_telem_claw_);

        callback_group_watchdog_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        watchdog_timer_ = create_wall_timer(
            100ms, bind(&ArmNode::watchdog, this), callback_group_watchdog_);
    }

    ~ArmNode()
    {
        hipMotor.disconnect();
        shoulderMotor.disconnect();
        elbowMotor.disconnect();
        rollMotor.disconnect();
        pitchMotor.disconnect();
        clawMotor.disconnect();

        {
        std::lock_guard<std::mutex> lock(config_mutex_);
        stop_thread_ = true;
        }
        config_cv_.notify_one();

        if (config_thread_.joinable())
            config_thread_.join();
    }

    // normalized: -1.0 = full reverse, 0.0 = stop, 1.0 = full forward
    void setHipTarget(float normalized)      { target_hip_.store(normalized); }
    void setShoulderTarget(float normalized) { target_shoulder_.store(normalized); }
    void setElbowTarget(float normalized)    { target_elbow_.store(normalized); }
    void setRollTarget(float normalized)     { target_roll_.store(normalized); }
    void setPitchTarget(float normalized)    { target_pitch_.store(normalized); }
    void setClawTarget(float normalized)     { target_claw_.store(normalized); }

    void save_config_(){
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        save_requested_ = true;
    }
    config_cv_.notify_one();
}

private:
    void setHipRPM() {
        if (!hipMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Hip motor disconnected, reconnecting...");
            std::string hint; { std::lock_guard<std::mutex> lk(settings_mutex_); hint = hip_.port; }
            if (hipMotor.autoConnect(hint)) { std::lock_guard<std::mutex> lk(settings_mutex_); hip_.port = hipMotor.getPortName(); }
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = hip_.rpm_limit; duty_lim = hip_.duty_cycle_limit; inv = hip_.inverted; mode = hip_.control_mode; }
        float cmd = std::clamp(target_hip_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { hipMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { hipMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setShoulderRPM() {
        if (!shoulderMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Shoulder motor disconnected, reconnecting...");
            std::string hint; { std::lock_guard<std::mutex> lk(settings_mutex_); hint = shoulder_.port; }
            if (shoulderMotor.autoConnect(hint)) { std::lock_guard<std::mutex> lk(settings_mutex_); shoulder_.port = shoulderMotor.getPortName(); }
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = shoulder_.rpm_limit; duty_lim = shoulder_.duty_cycle_limit; inv = shoulder_.inverted; mode = shoulder_.control_mode; }
        float cmd = std::clamp(target_shoulder_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { shoulderMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { shoulderMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setElbowRPM() {
        if (!elbowMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Elbow motor disconnected, reconnecting...");
            std::string hint; { std::lock_guard<std::mutex> lk(settings_mutex_); hint = elbow_.port; }
            if (elbowMotor.autoConnect(hint)) { std::lock_guard<std::mutex> lk(settings_mutex_); elbow_.port = elbowMotor.getPortName(); }
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = elbow_.rpm_limit; duty_lim = elbow_.duty_cycle_limit; inv = elbow_.inverted; mode = elbow_.control_mode; }
        float cmd = std::clamp(target_elbow_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { elbowMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { elbowMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setRollRPM() {
        if (!rollMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Roll motor disconnected, reconnecting...");
            std::string hint; { std::lock_guard<std::mutex> lk(settings_mutex_); hint = roll_.port; }
            if (rollMotor.autoConnect(hint)) { std::lock_guard<std::mutex> lk(settings_mutex_); roll_.port = rollMotor.getPortName(); }
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = roll_.rpm_limit; duty_lim = roll_.duty_cycle_limit; inv = roll_.inverted; mode = roll_.control_mode; }
        float cmd = std::clamp(target_roll_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { rollMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { rollMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setPitchRPM() {
        if (!pitchMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Pitch motor disconnected, reconnecting...");
            std::string hint; { std::lock_guard<std::mutex> lk(settings_mutex_); hint = pitch_.port; }
            if (pitchMotor.autoConnect(hint)) { std::lock_guard<std::mutex> lk(settings_mutex_); pitch_.port = pitchMotor.getPortName(); }
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = pitch_.rpm_limit; duty_lim = pitch_.duty_cycle_limit; inv = pitch_.inverted; mode = pitch_.control_mode; }
        float cmd = std::clamp(target_pitch_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { pitchMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { pitchMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    void setClawRPM() {
        if (!clawMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Claw motor disconnected, reconnecting...");
            std::string hint; { std::lock_guard<std::mutex> lk(settings_mutex_); hint = claw_.port; }
            if (clawMotor.autoConnect(hint)) { std::lock_guard<std::mutex> lk(settings_mutex_); claw_.port = clawMotor.getPortName(); }
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        { std::lock_guard<std::mutex> lk(settings_mutex_); rpm_lim = claw_.rpm_limit; duty_lim = claw_.duty_cycle_limit; inv = claw_.inverted; mode = claw_.control_mode; }
        float cmd = std::clamp(target_claw_.load(), -1.0f, 1.0f);
        if (std::abs(cmd) < DEADZONE) cmd = 0.0f;
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) { clawMotor.set_duty_cycle(cmd * duty_lim * sign); }
        else           { clawMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign)); }
    }

    using TelemPub = rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr;

    void publishTelemetry(VESC& motor, const std::string& name,
                          const MotorSettings& s, const TelemPub& pub) {
        VESCData t;
        if (!motor.isConnected() || !motor.get_telemetry(t)) return;
        robot_msgs::msg::MotorTelemetry msg;
        msg.motor_id         = t.motor_controller_id;
        msg.motor_name       = name;
        msg.rpm              = t.rpm;
        msg.duty_cycle       = t.duty_cycle;
        msg.current_in       = t.current_in;
        msg.voltage          = t.input_voltage;
        msg.position         = t.position;
        msg.fault_code       = t.fault_code;
        msg.control_mode     = s.control_mode;
        msg.inverted         = s.inverted;
        msg.current_motor    = t.current_motor;
        msg.rpm_limit        = s.rpm_limit;
        msg.duty_cycle_limit = s.duty_cycle_limit;
        pub->publish(msg);
    }

    void telemetryHip() {
        MotorSettings s; { std::lock_guard<std::mutex> lk(settings_mutex_); s = hip_; }
        publishTelemetry(hipMotor, "Cadera", s, hip_telemetry_pub);
    }
    void telemetryShoulder() {
        MotorSettings s; { std::lock_guard<std::mutex> lk(settings_mutex_); s = shoulder_; }
        publishTelemetry(shoulderMotor, "Hombro", s, shoulder_telemetry_pub);
    }
    void telemetryElbow() {
        MotorSettings s; { std::lock_guard<std::mutex> lk(settings_mutex_); s = elbow_; }
        publishTelemetry(elbowMotor, "Codo", s, elbow_telemetry_pub);
    }
    void telemetryRoll() {
        MotorSettings s; { std::lock_guard<std::mutex> lk(settings_mutex_); s = roll_; }
        publishTelemetry(rollMotor, "Roll", s, roll_telemetry_pub);
    }
    void telemetryPitch() {
        MotorSettings s; { std::lock_guard<std::mutex> lk(settings_mutex_); s = pitch_; }
        publishTelemetry(pitchMotor, "Pitch", s, pitch_telemetry_pub);
    }
    void telemetryClaw() {
        MotorSettings s; { std::lock_guard<std::mutex> lk(settings_mutex_); s = claw_; }
        publishTelemetry(clawMotor, "Grip", s, claw_telemetry_pub);
    }

    // ---- Watchdog ----
    void watchdog() {
        auto last = last_input_ns_.load(std::memory_order_relaxed);
        if (last == 0) return;  // no message received yet, nothing to guard
        constexpr int64_t TIMEOUT_NS = 200'000'000LL;  // 200 ms
        auto now = std::chrono::steady_clock::now().time_since_epoch().count();
        if (now - last > TIMEOUT_NS) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "No /control/input for >200 ms — zeroing all arm commands");
            target_hip_.store(0.0f,      std::memory_order_relaxed);
            target_shoulder_.store(0.0f, std::memory_order_relaxed);
            target_elbow_.store(0.0f,    std::memory_order_relaxed);
            target_roll_.store(0.0f,     std::memory_order_relaxed);
            target_pitch_.store(0.0f,    std::memory_order_relaxed);
            target_claw_.store(0.0f,     std::memory_order_relaxed);
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
                    s.port             = motors[idx].value("port",             s.port);
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
            MotorSettings h, sh, el, ro, pi, cl;
            {
                std::lock_guard<std::mutex> lk(settings_mutex_);
                h = hip_; sh = shoulder_; el = elbow_;
                ro = roll_; pi = pitch_; cl = claw_;
            }
            // Re-read the file before writing so we don't overwrite body_node's section
            // (indices 0-3). Both nodes share the same config.json; writing from a stale
            // in-memory copy would clobber the other node's port hints.
            try {
                std::ifstream fin(config_path_);
                if (fin.is_open()) config_data_ = nlohmann::json::parse(fin);
            } catch (...) { /* keep existing config_data_ as fallback */ }

            auto apply = [&](int idx, const MotorSettings& s) {
                config_data_["motors"][idx]["id"]               = static_cast<int>(s.vesc_id);
                config_data_["motors"][idx]["rpm_limit"]        = s.rpm_limit;
                config_data_["motors"][idx]["duty_cycle_limit"] = s.duty_cycle_limit;
                config_data_["motors"][idx]["control_mode"]     = static_cast<int>(s.control_mode);
                config_data_["motors"][idx]["inverted"]         = s.inverted;
                config_data_["motors"][idx]["port"]             = s.port;
            };
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
        save_config_();
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
    std::atomic<int64_t> last_input_ns_{0}; // steady_clock ns of last /control/input
    rclcpp::TimerBase::SharedPtr     watchdog_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_watchdog_;

    rclcpp::TimerBase::SharedPtr telemetry_timer_hip_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_shoulder_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_elbow_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_roll_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_pitch_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_claw_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telem_hip_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telem_shoulder_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telem_elbow_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telem_roll_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telem_pitch_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telem_claw_;

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