#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_pkg/VESC.hpp"
#include "robot_pkg/json.hpp"
#include "robot_msgs/msg/motor_config.hpp"
#include "robot_msgs/msg/motor_telemetry.hpp"
#include "robot_msgs/msg/control_input.hpp"
#include "robot_msgs/msg/d_pad_config.hpp"
#include <algorithm>
#include <chrono>
#include <functional>
#include <fstream>
#include <atomic>
#include <filesystem>
#include <mutex>

using namespace std::chrono_literals;

// QoS para control en tiempo real: sin retransmisión, siempre el dato más reciente
static const rclcpp::QoS CONTROL_QOS = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();

// QoS para telemetría: también sin retransmisión pero con algo de buffer
static const rclcpp::QoS TELEM_QOS = rclcpp::QoS(rclcpp::KeepLast(5))
    .best_effort()
    .durability_volatile();

struct MotorSettings {
    uint8_t vesc_id          = 0;
    float   rpm_limit        = 5000.0f;
    float   duty_cycle_limit = 1.0f;
    uint8_t control_mode     = 0;
    bool    inverted         = false;
};

class BodyNode : public rclcpp::Node{
    public:

        BodyNode() : Node("body_node"),
            leftMotor(
                declare_parameter<uint8_t>("left_id", left_.vesc_id),
                declare_parameter<int>("left_baudrate", 115200),
                declare_parameter<int>("left_timeout", 1000)),
                
            rightMotor(
                declare_parameter<uint8_t>("right_id", right_.vesc_id),
                declare_parameter<int>("right_baudrate", 115200),
                declare_parameter<int>("right_timeout", 1000)),
            leftFlipperMotor(
                declare_parameter<uint8_t>("left_flipper_id", left_flipper_.vesc_id),
                declare_parameter<int>("left_flipper_baudrate", 115200),
                declare_parameter<int>("left_flipper_timeout", 1000)),
            rightFlipperMotor(
                declare_parameter<uint8_t>("right_flipper_id", right_flipper_.vesc_id),
                declare_parameter<int>("right_flipper_baudrate", 115200),
                declare_parameter<int>("right_flipper_timeout", 1000))
                {

                loadConfig();

                leftMotor.setId(left_.vesc_id);
                rightMotor.setId(right_.vesc_id);
                leftFlipperMotor.setId(left_flipper_.vesc_id);
                rightFlipperMotor.setId(right_flipper_.vesc_id);

                if(leftMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Left motor connected.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to connect to left motor.");
                }
                if(rightMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Right motor connected.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to connect to right motor.");
                }
                if(leftFlipperMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Flipper Trasero connected.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to connect to Flipper Trasero.");
                }
                if(rightFlipperMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Flipper Delantero connected.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to connect to Flipper Delantero.");
                }

                config_update_sub_ = create_subscription<robot_msgs::msg::MotorConfig>(
                    "/robot_config/update", 10,
                    [this](const robot_msgs::msg::MotorConfig::SharedPtr msg) {
                        onConfigUpdate(msg);
                    });

                // Control input: mode + all axes, pre-compute flipper mixing here
                sub_axes_ = create_subscription<robot_msgs::msg::ControlInput>(
                    "/control/input", CONTROL_QOS,
                    [this](const robot_msgs::msg::ControlInput::SharedPtr msg) {
                        control_mode_.store(msg->mode, std::memory_order_relaxed);
                        joystick_left_y.store(msg->left_y, std::memory_order_relaxed);
                        joystick_left_x.store(msg->left_x, std::memory_order_relaxed);
                        dpad_x_.store(msg->dpad_x, std::memory_order_relaxed);
                        dpad_y_.store(msg->dpad_y, std::memory_order_relaxed);
                        if (msg->mode == 0) {  // MOVEMENT: compute flipper mixing
                            float ry = msg->right_y, rx = msg->right_x;
                            // front (delantero): full up + rightward contribution
                            joystick_right_y.store(
                                std::clamp(ry + std::max(0.0f,  rx), -1.0f, 1.0f),
                                std::memory_order_relaxed);
                            // rear (trasero): full up + leftward contribution
                            joystick_right_x.store(
                                std::clamp(ry + std::max(0.0f, -rx), -1.0f, 1.0f),
                                std::memory_order_relaxed);
                        } else {  // ARM mode: zero flippers
                            joystick_right_y.store(0.0f, std::memory_order_relaxed);
                            joystick_right_x.store(0.0f, std::memory_order_relaxed);
                        }
                    });

                // D-pad limits config (from ground station UI)
                sub_dpad_config_ = create_subscription<robot_msgs::msg::DPadConfig>(
                    "/ground_station/dpad_config",
                    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
                    [this](const robot_msgs::msg::DPadConfig::SharedPtr msg) {
                        std::lock_guard<std::mutex> lk(dpad_mutex_);
                        dpad_limits_.rpm  = msg->rpm_limit;
                        dpad_limits_.duty = msg->duty_cycle_limit;
                        RCLCPP_INFO(get_logger(), "DPad limits updated: rpm=%.0f duty=%.3f",
                                    dpad_limits_.rpm, dpad_limits_.duty);
                    });


            left_full_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/body_left/telemetry", TELEM_QOS);
            right_full_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/body_right/telemetry", TELEM_QOS);

            left_flipper_full_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/body_left_flipper/telemetry", TELEM_QOS);
            right_flipper_full_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/body_right_flipper/telemetry", TELEM_QOS);

            // Drive timers (con callback groups para ejecución paralela)
            callback_group_left_          = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_right_         = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_left_flipper_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_right_flipper_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            timer_left_ = create_wall_timer(
                10ms, bind(&BodyNode::driveLeft, this), callback_group_left_);
            timer_right_ = create_wall_timer(
                10ms, bind(&BodyNode::driveRight, this), callback_group_right_);
            timer_left_flipper_ = create_wall_timer(
                10ms, bind(&BodyNode::driveFrontFlipper, this), callback_group_left_flipper_);
            timer_right_flipper_ = create_wall_timer(
                10ms, bind(&BodyNode::driveRearFlipper, this), callback_group_right_flipper_);

            // setup telemetry timer
            callback_group_telemetry_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            telemetry_timer_ = this->create_wall_timer(
                50ms, bind(&BodyNode::telemetry, this), callback_group_telemetry_);
            }

        ~BodyNode() {
            leftMotor.disconnect();
            rightMotor.disconnect();
            leftFlipperMotor.disconnect();
            rightFlipperMotor.disconnect();
        }


    private:

    void driveLeft(){
        if (!leftMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Left motor disconnected, reconnecting...");
            leftMotor.autoConnect();
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        {
            std::lock_guard<std::mutex> lk(settings_mutex_);
            rpm_lim = left_.rpm_limit; duty_lim = left_.duty_cycle_limit;
            inv = left_.inverted; mode = left_.control_mode;
        }
        float dx = dpad_x_.load(std::memory_order_relaxed);
        float dy = dpad_y_.load(std::memory_order_relaxed);
        float cmd;
        if (dx != 0.0f || dy != 0.0f) {
            // D-pad override (active in any mode): tank mix with D-pad limits
            cmd = std::clamp(dy + dx, -1.0f, 1.0f);
            std::lock_guard<std::mutex> lk(dpad_mutex_);
            rpm_lim = dpad_limits_.rpm; duty_lim = dpad_limits_.duty;
        } else if (control_mode_.load(std::memory_order_relaxed) == 0) {
            // MOVEMENT mode: joystick tank mix left = Y + X
            cmd = std::clamp(
                joystick_left_y.load(std::memory_order_relaxed) +
                joystick_left_x.load(std::memory_order_relaxed), -1.0f, 1.0f);
        } else {
            // ARM mode, no D-pad: tracks stop
            cmd = 0.0f;
        }
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) {
            leftMotor.set_duty_cycle(cmd * duty_lim * sign);
        } else {
            leftMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign));
        }
    }

    void driveRight(){
        if (!rightMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Right motor disconnected, reconnecting...");
            rightMotor.autoConnect();
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        {
            std::lock_guard<std::mutex> lk(settings_mutex_);
            rpm_lim = right_.rpm_limit; duty_lim = right_.duty_cycle_limit;
            inv = right_.inverted; mode = right_.control_mode;
        }
        float dx = dpad_x_.load(std::memory_order_relaxed);
        float dy = dpad_y_.load(std::memory_order_relaxed);
        float cmd;
        if (dx != 0.0f || dy != 0.0f) {
            // D-pad override: tank mix right = dy - dx
            cmd = std::clamp(dy - dx, -1.0f, 1.0f);
            std::lock_guard<std::mutex> lk(dpad_mutex_);
            rpm_lim = dpad_limits_.rpm; duty_lim = dpad_limits_.duty;
        } else if (control_mode_.load(std::memory_order_relaxed) == 0) {
            // MOVEMENT mode: joystick tank mix right = Y - X
            cmd = std::clamp(
                joystick_left_y.load(std::memory_order_relaxed) -
                joystick_left_x.load(std::memory_order_relaxed), -1.0f, 1.0f);
        } else {
            // ARM mode, no D-pad: tracks stop
            cmd = 0.0f;
        }
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) {
            rightMotor.set_duty_cycle(cmd * duty_lim * sign);
        } else {
            rightMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign));
        }
    }

    void driveFrontFlipper(){   // flipper delantero ← right joystick Y
        if (!rightFlipperMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Flipper Delantero disconnected, reconnecting...");
            rightFlipperMotor.autoConnect();
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        {
            std::lock_guard<std::mutex> lk(settings_mutex_);
            rpm_lim = right_flipper_.rpm_limit; duty_lim = right_flipper_.duty_cycle_limit;
            inv = right_flipper_.inverted; mode = right_flipper_.control_mode;
        }
        float cmd = std::clamp(joystick_right_y.load(std::memory_order_relaxed), -1.0f, 1.0f);
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) {
            rightFlipperMotor.set_duty_cycle(cmd * duty_lim * sign);
        } else {
            rightFlipperMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign));
        }
    }

    void driveRearFlipper(){    // flipper trasero ← right joystick X
        if (!leftFlipperMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Flipper Trasero disconnected, reconnecting...");
            leftFlipperMotor.autoConnect();
            return;
        }
        float rpm_lim, duty_lim; bool inv; uint8_t mode;
        {
            std::lock_guard<std::mutex> lk(settings_mutex_);
            rpm_lim = left_flipper_.rpm_limit; duty_lim = left_flipper_.duty_cycle_limit;
            inv = left_flipper_.inverted; mode = left_flipper_.control_mode;
        }
        float cmd = std::clamp(joystick_right_x.load(std::memory_order_relaxed), -1.0f, 1.0f);
        float sign = inv ? -1.0f : 1.0f;
        if (mode == 1) {
            leftFlipperMotor.set_duty_cycle(cmd * duty_lim * sign);
        } else {
            leftFlipperMotor.set_rpm(static_cast<int32_t>(cmd * rpm_lim * sign));
        }
    }

    void telemetry(){
        VESCData m_telemetry;
        MotorSettings l, r, lf, rf;
        {
            std::lock_guard<std::mutex> lk(settings_mutex_);
            l = left_; r = right_; lf = left_flipper_; rf = right_flipper_;
        }

        if(leftMotor.isConnected() && leftMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Track Izquierdo";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.current_in   = m_telemetry.current_in;
            msg.voltage      = m_telemetry.input_voltage;
            msg.position     = m_telemetry.position;
            msg.fault_code   = m_telemetry.fault_code;
            msg.control_mode     = l.control_mode;
            msg.inverted         = l.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = l.rpm_limit;
            msg.duty_cycle_limit = l.duty_cycle_limit;
            left_full_telemetry_pub->publish(msg);
        }

        if(rightMotor.isConnected() && rightMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m_telemetry.motor_controller_id;
            msg.motor_name       = "Track Derecho";
            msg.rpm              = m_telemetry.rpm;
            msg.duty_cycle       = m_telemetry.duty_cycle;
            msg.current_in       = m_telemetry.current_in;
            msg.voltage          = m_telemetry.input_voltage;
            msg.position         = m_telemetry.position;
            msg.fault_code       = m_telemetry.fault_code;
            msg.control_mode     = r.control_mode;
            msg.inverted         = r.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = r.rpm_limit;
            msg.duty_cycle_limit = r.duty_cycle_limit;
            right_full_telemetry_pub->publish(msg);
        }

        if(leftFlipperMotor.isConnected() && leftFlipperMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m_telemetry.motor_controller_id;
            msg.motor_name       = "Flipper Trasero";
            msg.rpm              = m_telemetry.rpm;
            msg.duty_cycle       = m_telemetry.duty_cycle;
            msg.current_in       = m_telemetry.current_in;
            msg.voltage          = m_telemetry.input_voltage;
            msg.position         = m_telemetry.position;
            msg.fault_code       = m_telemetry.fault_code;
            msg.control_mode     = lf.control_mode;
            msg.inverted         = lf.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = lf.rpm_limit;
            msg.duty_cycle_limit = lf.duty_cycle_limit;
            left_flipper_full_telemetry_pub->publish(msg);
        }

        if(rightFlipperMotor.isConnected() && rightFlipperMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m_telemetry.motor_controller_id;
            msg.motor_name       = "Flipper Delantero";
            msg.rpm              = m_telemetry.rpm;
            msg.duty_cycle       = m_telemetry.duty_cycle;
            msg.current_in       = m_telemetry.current_in;
            msg.voltage          = m_telemetry.input_voltage;
            msg.position         = m_telemetry.position;
            msg.fault_code       = m_telemetry.fault_code;
            msg.control_mode     = rf.control_mode;
            msg.inverted         = rf.inverted;
            msg.current_motor    = m_telemetry.current_motor;
            msg.rpm_limit        = rf.rpm_limit;
            msg.duty_cycle_limit = rf.duty_cycle_limit;
            right_flipper_full_telemetry_pub->publish(msg);
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
            read(0, left_flipper_); read(1, left_); read(2, right_); read(3, right_flipper_);
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
            MotorSettings lf, l, r, rf;
            {
                std::lock_guard<std::mutex> lk(settings_mutex_);
                lf = left_flipper_; l = left_; r = right_; rf = right_flipper_;
            }
            apply(0, lf); apply(1, l); apply(2, r); apply(3, rf);
            std::ofstream f(config_path_);
            f << config_data_.dump(2);
            RCLCPP_INFO(get_logger(), "Config saved.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "saveConfig failed: %s", e.what());
        }
    }

    void onConfigUpdate(const robot_msgs::msg::MotorConfig::SharedPtr msg) {
        MotorSettings* t  = nullptr;
        VESC*          v  = nullptr;
        switch (msg->config_index) {
            case 0: t = &left_flipper_; v = &leftFlipperMotor;  break;
            case 1: t = &left_;         v = &leftMotor;          break;
            case 2: t = &right_;        v = &rightMotor;         break;
            case 3: t = &right_flipper_;v = &rightFlipperMotor;  break;
            default: return;  // Not a body motor
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

    VESC leftMotor;
    VESC rightMotor;
    VESC leftFlipperMotor;
    VESC rightFlipperMotor;


    rclcpp::CallbackGroup::SharedPtr callback_group_left_;
    rclcpp::CallbackGroup::SharedPtr callback_group_right_;
    rclcpp::CallbackGroup::SharedPtr callback_group_left_flipper_;
    rclcpp::CallbackGroup::SharedPtr callback_group_right_flipper_;

    rclcpp::TimerBase::SharedPtr timer_left_;
    rclcpp::TimerBase::SharedPtr timer_right_;
    rclcpp::TimerBase::SharedPtr timer_left_flipper_;
    rclcpp::TimerBase::SharedPtr timer_right_flipper_;
    
    rclcpp::Subscription<robot_msgs::msg::ControlInput>::SharedPtr sub_axes_;
    rclcpp::Subscription<robot_msgs::msg::DPadConfig>::SharedPtr sub_dpad_config_;
    
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr left_full_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr right_full_telemetry_pub;
    // telemetry publishers for flipper motors
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr left_flipper_full_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr right_flipper_full_telemetry_pub;
    // timer and callback group for periodic telemetry
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telemetry_;

    std::atomic<float>   joystick_left_y  {0.0f};
    std::atomic<float>   joystick_left_x  {0.0f};
    std::atomic<float>   joystick_right_y {0.0f};  // pre-mixed front flipper cmd
    std::atomic<float>   joystick_right_x {0.0f};  // pre-mixed rear flipper cmd
    std::atomic<uint8_t> control_mode_   {0};       // 0=MOVEMENT, 1=ARM
    std::atomic<float>   dpad_x_         {0.0f};
    std::atomic<float>   dpad_y_         {0.0f};
    struct DPadLimits { float rpm = 500.0f; float duty = 0.10f; };
    DPadLimits dpad_limits_;
    std::mutex dpad_mutex_;

    // ---- Config state ----
    std::string         config_path_;
    nlohmann::json      config_data_;
    std::mutex          settings_mutex_;
    MotorSettings       left_flipper_, left_, right_, right_flipper_;
    rclcpp::Subscription<robot_msgs::msg::MotorConfig>::SharedPtr config_update_sub_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    rclcpp::shutdown();
    return 0;
}