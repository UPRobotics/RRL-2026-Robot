#include "rclcpp/rclcpp.hpp"
#include "robot_pkg/VESC.hpp"
#include "std_msgs/msg/float32.hpp" 
#include "robot_msgs/msg/motor_telemetry.hpp" 
#include <chrono>
#include <functional>

using namespace std;
using namespace std::chrono_literals;

struct MotorSettings {
    float   rpm_limit        = 5000.0f;
    float   duty_cycle_limit = 1.0f;
    uint8_t control_mode     = 0;
    bool    inverted         = false;
};

class BodyNode : public rclcpp::Node{
    public:
        //TODO: For now placeholder values
        int leftMotorId    = 1;
        int rightMotorId   = 2;
        int leftFlipperId  = 0;
        int rightFlipperId = 3;

        BodyNode() : Node("body_node"),
            leftMotor(
                declare_parameter<uint8_t>("left_id", leftMotorId),
                declare_parameter<int>("left_baudrate", 115200),
                declare_parameter<int>("left_timeout", 1000)),
                
            rightMotor(
                declare_parameter<uint8_t>("right_id", rightMotorId),
                declare_parameter<int>("right_baudrate", 115200),
                declare_parameter<int>("right_timeout", 1000)),
            leftFlipperMotor(
                declare_parameter<uint8_t>("left_flipper_id", leftFlipperId),
                declare_parameter<int>("left_flipper_baudrate", 115200),
                declare_parameter<int>("left_flipper_timeout", 1000)),
            rightFlipperMotor(
                declare_parameter<uint8_t>("right_flipper_id", rightFlipperId),
                declare_parameter<int>("right_flipper_baudrate", 115200),
                declare_parameter<int>("right_flipper_timeout", 1000))
                {

                if(leftMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Left motor body connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to left motor");
                }
                if(rightMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Right motor body connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to right motor");
                }
                if(leftFlipperMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Left flipper motor connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to left flipper motor");
                }
                if(rightFlipperMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Right flipper motor connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to right flipper motor");
                }

                loadConfig();

                config_update_sub_ = create_subscription<robot_msgs::msg::MotorConfig>(
                    "/robot_config/update", 10,
                    [this](const robot_msgs::msg::MotorConfig::SharedPtr msg) {
                        onConfigUpdate(msg);
                    });

                callback_group_left_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
                callback_group_right_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                callback_group_left_flipper_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                callback_group_right_flipper_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                timer_left_ = this->create_wall_timer(
                    20ms, bind(&BodyNode::driveLeft, this), callback_group_left_);
                    
                timer_right_ = this->create_wall_timer(
                    20ms, bind(&BodyNode::driveRight, this), callback_group_right_);

                timer_left_flipper_ = this->create_wall_timer(
                    20ms, bind(&BodyNode::driveLeftFlipper, this), callback_group_left_flipper_);
                    
                timer_right_flipper_ = this->create_wall_timer(
                    20ms, bind(&BodyNode::driveRightFlipper, this), callback_group_right_flipper_);
                
                y_left_Axis_subscriber = create_subscription<std_msgs::msg::Float32>(
                "/arm/y_left_axis", 10,
                [this](const std_msgs::msg::Float32::SharedPtr msg) {
                    this->joystick_left_y = msg->data;
                });

                flipper_axis_subscriber = create_subscription<std_msgs::msg::Float32>(
                "/arm/flipper_axis", 10,
                [this](const std_msgs::msg::Float32::SharedPtr msg) {
                    this->joystick_flipper = msg->data;
                });


            left_full_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/body_left/telemetry", 10);
            right_full_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/body_right/telemetry", 10);

            left_flipper_full_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/body_left_flipper/telemetry", 10);
            right_flipper_full_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/body_right_flipper/telemetry", 10);

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
        float lim; bool inv;
        { std::lock_guard<std::mutex> lk(settings_mutex_); lim = left_.rpm_limit; inv = left_.inverted; }
        leftMotor.set_rpm(static_cast<int32_t>(joystick_left_y * lim * (inv ? -1.0f : 1.0f)));
    }

    void driveRight(){
        if (!rightMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Right motor disconnected, reconnecting...");
            rightMotor.autoConnect();
            return;
        }
        float lim; bool inv;
        { std::lock_guard<std::mutex> lk(settings_mutex_); lim = right_.rpm_limit; inv = right_.inverted; }
        rightMotor.set_rpm(static_cast<int32_t>(joystick_left_y * lim * (inv ? -1.0f : 1.0f)));
    }

    void driveLeftFlipper(){
        if (!leftFlipperMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Left flipper disconnected, reconnecting...");
            leftFlipperMotor.autoConnect();
            return;
        }
        float lim; bool inv;
        { std::lock_guard<std::mutex> lk(settings_mutex_); lim = left_flipper_.rpm_limit; inv = left_flipper_.inverted; }
        leftFlipperMotor.set_rpm(static_cast<int32_t>(joystick_flipper * lim * (inv ? -1.0f : 1.0f)));
    }

    void driveRightFlipper(){
        if (!rightFlipperMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Right flipper disconnected, reconnecting...");
            rightFlipperMotor.autoConnect();
            return;
        }
        float lim; bool inv;
        { std::lock_guard<std::mutex> lk(settings_mutex_); lim = right_flipper_.rpm_limit; inv = right_flipper_.inverted; }
        rightFlipperMotor.set_rpm(static_cast<int32_t>(joystick_flipper * lim * (inv ? -1.0f : 1.0f)));
    }

    void telemetry(){
        VESCData m_telemetry;

        if(leftMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Track Izquierdo";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 0;
            msg.inverted     = false;
            left_full_telemetry_pub->publish(msg);
        }

        if(rightMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Track Derecho";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 0;
            msg.inverted     = true;
            right_full_telemetry_pub->publish(msg);
        }

        // flipper telemetry
        if(leftFlipperMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Flipper Trasero";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 0;
            msg.inverted     = false;
            left_flipper_full_telemetry_pub->publish(msg);
        }
        if(rightFlipperMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Flipper Delantero";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 0;
            msg.inverted     = true;
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
                    s.rpm_limit        = motors[idx].value("rpm_limit",        s.rpm_limit);
                    s.duty_cycle_limit = motors[idx].value("duty_cycle_limit", s.duty_cycle_limit);
                    s.control_mode     = static_cast<uint8_t>(motors[idx].value("control_mode", static_cast<int>(s.control_mode)));
                    s.inverted         = motors[idx].value("inverted",         s.inverted);
                }
            };
            std::lock_guard<std::mutex> lk(settings_mutex_);
            read(0, left_flipper_); read(1, left_); read(2, right_); read(3, right_flipper_);
            RCLCPP_INFO(get_logger(), "Config loaded from %s", config_path_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "loadConfig failed: %s", e.what());
        }
    }

    void saveConfig() {
        try {
            auto apply = [&](int idx, const MotorSettings& s) {
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
        MotorSettings* t = nullptr;
        switch (msg->config_index) {
            case 0: t = &left_flipper_; break;
            case 1: t = &left_;         break;
            case 2: t = &right_;        break;
            case 3: t = &right_flipper_;break;
            default: return;  // Not a body motor
        }
        {
            std::lock_guard<std::mutex> lk(settings_mutex_);
            t->rpm_limit        = msg->rpm_limit;
            t->duty_cycle_limit = msg->duty_cycle_limit;
            t->control_mode     = msg->control_mode;
            t->inverted         = msg->inverted;
        }
        RCLCPP_INFO(get_logger(),
            "Config update [%u] %s: rpm_limit=%.1f  duty=%.3f  mode=%u  inv=%s",
            msg->config_index, msg->motor_name.c_str(),
            msg->rpm_limit, msg->duty_cycle_limit, msg->control_mode,
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
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_left_Axis_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr flipper_axis_subscriber;
    
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr left_full_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr right_full_telemetry_pub;
    // telemetry publishers for flipper motors
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr left_flipper_full_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr right_flipper_full_telemetry_pub;
    // timer and callback group for periodic telemetry
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telemetry_;

    float joystick_left_y = 0.0f;
    float joystick_flipper = 0.0f;

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