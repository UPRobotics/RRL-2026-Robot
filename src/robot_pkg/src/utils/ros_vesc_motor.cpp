#include "robot_pkg/ros_vesc_motor.hpp"
#include <algorithm>

using namespace std::chrono_literals;

RosVescMotor::RosVescMotor(rclcpp::Node* node, const std::string& name, const std::string& telemetry_topic, std::function<void()> on_port_updated)
    : node_(node), name_(name),
      motor_(
          node->declare_parameter<uint8_t>(name + "_id", 0),
          node->declare_parameter<int>(name + "_baudrate", 115200),
          node->declare_parameter<int>(name + "_timeout", 1000)),
      on_port_updated_cb_(on_port_updated)
{
    control_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    telem_cb_group_  = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::QoS telem_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    telem_pub_ = node_->create_publisher<robot_msgs::msg::MotorTelemetry>(telemetry_topic, telem_qos);

    control_timer_ = node_->create_wall_timer(10ms, [this]() { this->controlLoop(); }, control_cb_group_);
    telem_timer_   = node_->create_wall_timer(50ms, [this]() { this->telemetryLoop(); }, telem_cb_group_);
}

RosVescMotor::~RosVescMotor() {
    disconnect();
}

void RosVescMotor::disconnect() { motor_.disconnect(); }

void RosVescMotor::set(float power) {
    current_cmd_.store(power, std::memory_order_relaxed);
    use_custom_limits_ = false; 
}

void RosVescMotor::setWithCustomLimits(float power, float custom_rpm, float custom_duty) {
    current_cmd_.store(power, std::memory_order_relaxed);
    active_rpm_limit_ = custom_rpm;
    active_duty_limit_ = custom_duty;
    use_custom_limits_ = true;
}

void RosVescMotor::applySettings(const MotorSettings& new_settings) {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    settings_ = new_settings;
    motor_.setId(settings_.vesc_id);
}

MotorSettings RosVescMotor::getSettings() {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    return settings_;
}

bool RosVescMotor::autoConnectInit() {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    if (!settings_.enabled) return false;
    
    if (motor_.autoConnect(settings_.port)) {
        settings_.port = motor_.getPortName();
        return true;
    }
    return false;
}

void RosVescMotor::controlLoop() {
    {
        std::lock_guard<std::mutex> lk(settings_mutex_);
        if (!settings_.enabled) return;
    }

    // AUTO-RECONNECT SAFETY NET
    if (!motor_.isConnected()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "[%s] Disconnected, attempting auto-reconnect...", name_.c_str());
        std::string hint; 
        { std::lock_guard<std::mutex> lk(settings_mutex_); hint = settings_.port; }
        
        try {
            if (motor_.autoConnect(hint)) { 
                std::lock_guard<std::mutex> lk(settings_mutex_); 
                settings_.port = motor_.getPortName(); 
                if (on_port_updated_cb_) on_port_updated_cb_(); // Trigger JSON save
            }
        } catch (...) {}
        return;
    }

    // NORMAL OPERATION
    float rpm_lim, duty_lim; bool inv; uint8_t c_mode;
    {
        std::lock_guard<std::mutex> lk(settings_mutex_);
        rpm_lim = use_custom_limits_ ? active_rpm_limit_ : settings_.rpm_limit; 
        duty_lim = use_custom_limits_ ? active_duty_limit_ : settings_.duty_cycle_limit;
        inv = settings_.inverted; 
        c_mode = settings_.control_mode; // 0 = RPM, 1 = Duty
    }

    float cmd = std::clamp(current_cmd_.load(std::memory_order_relaxed), -1.0f, 1.0f);
    float sign = inv ? -1.0f : 1.0f;

    if (c_mode == 1) {
        motor_.set_duty_cycle(cmd * duty_lim * sign);
    } else {
        motor_.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign));
    }
}

void RosVescMotor::telemetryLoop() {
    robot_msgs::msg::MotorTelemetry msg;
    MotorSettings s = getSettings();

    msg.motor_name       = name_;
    msg.control_mode     = s.control_mode;
    msg.inverted         = s.inverted;
    msg.rpm_limit        = s.rpm_limit;
    msg.duty_cycle_limit = s.duty_cycle_limit;
    msg.enabled          = s.enabled;
    msg.motor_id         = s.vesc_id;

    if (!s.enabled) {
        telem_pub_->publish(msg);
        return;
    }

    VESCData t;
    if (!motor_.isConnected() || !motor_.get_telemetry(t)) return;

    msg.motor_id         = t.motor_controller_id;
    msg.rpm              = t.rpm;
    msg.duty_cycle       = t.duty_cycle;
    msg.current_in       = t.current_in;
    msg.voltage          = t.input_voltage;
    msg.position         = t.position;
    msg.fault_code       = t.fault_code;
    msg.current_motor    = t.current_motor;
    telem_pub_->publish(msg);
}