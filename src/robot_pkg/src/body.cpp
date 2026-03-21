#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_pkg/VESC.hpp"
#include "robot_pkg/json.hpp"
#include "robot_msgs/msg/motor_config.hpp"
#include "robot_msgs/msg/motor_telemetry.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>
#include <chrono>
#include <functional>
#include <fstream>
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
                

                loadConfig();

                config_update_sub_ = create_subscription<robot_msgs::msg::MotorConfig>(
                    "/robot_config/update", 10,
                    [this](const robot_msgs::msg::MotorConfig::SharedPtr msg) {
                        onConfigUpdate(msg);
                    });

                // Joystick: BEST_EFFORT para mínima latencia
                sub_left_y_ = create_subscription<std_msgs::msg::Float32>(
                    "/joystick/left_y", CONTROL_QOS,
                    [this](const std_msgs::msg::Float32::SharedPtr msg) {
                        joystick_left_y = msg->data;
                    });

                sub_left_x_ = create_subscription<std_msgs::msg::Float32>(
                    "/joystick/left_x", CONTROL_QOS,
                    [this](const std_msgs::msg::Float32::SharedPtr msg) {
                        joystick_left_x = msg->data;
                    });

                sub_right_y_ = create_subscription<std_msgs::msg::Float32>(
                    "/joystick/right_y", CONTROL_QOS,
                    [this](const std_msgs::msg::Float32::SharedPtr msg) {
                        joystick_right_y = msg->data;  // flipper delantero
                    });

                sub_right_x_ = create_subscription<std_msgs::msg::Float32>(
                    "/joystick/right_x", CONTROL_QOS,
                    [this](const std_msgs::msg::Float32::SharedPtr msg) {
                        joystick_right_x = msg->data;  // flipper trasero
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
        float lim; bool inv;
        { std::lock_guard<std::mutex> lk(settings_mutex_); lim = left_.rpm_limit; inv = left_.inverted; }
        // Diferencial tipo tanque: left = Y + X
        float cmd = std::clamp(joystick_left_y + joystick_left_x, -1.0f, 1.0f);
        leftMotor.set_rpm(static_cast<int32_t>(cmd * lim * (inv ? -1.0f : 1.0f)));
    }

    void driveRight(){
        if (!rightMotor.isConnected()) {
        //    RCLCPP_WARN(get_logger(), "Right motor disconnected, reconnecting...");
         //   rightMotor.autoConnect();
            return;
        }
        float lim; bool inv;
        { std::lock_guard<std::mutex> lk(settings_mutex_); lim = right_.rpm_limit; inv = right_.inverted; }
        // Diferencial tipo tanque: right = Y - X
        float cmd = std::clamp(joystick_left_y - joystick_left_x, -1.0f, 1.0f);
        rightMotor.set_rpm(static_cast<int32_t>(cmd * lim * (inv ? -1.0f : 1.0f)));
    }

    void driveFrontFlipper(){   // flipper delantero ← right joystick Y
        if (!leftFlipperMotor.isConnected()) {
         //   RCLCPP_WARN(get_logger(), "Front flipper disconnected, reconnecting...");
           // leftFlipperMotor.autoConnect();
            return;
        }
        float lim; bool inv;
        { std::lock_guard<std::mutex> lk(settings_mutex_); lim = left_flipper_.rpm_limit; inv = left_flipper_.inverted; }
        float cmd = std::clamp(joystick_right_y, -1.0f, 1.0f);
        leftFlipperMotor.set_rpm(static_cast<int32_t>(cmd * lim * (inv ? -1.0f : 1.0f)));
    }

    void driveRearFlipper(){    // flipper trasero ← right joystick X
        if (!rightFlipperMotor.isConnected()) {
        //    RCLCPP_WARN(get_logger(), "Rear flipper disconnected, reconnecting...");
            //rightFlipperMotor.autoConnect();
            return;
        }
        float lim; bool inv;
        { std::lock_guard<std::mutex> lk(settings_mutex_); lim = right_flipper_.rpm_limit; inv = right_flipper_.inverted; }
        float cmd = std::clamp(joystick_right_x, -1.0f, 1.0f);
        rightFlipperMotor.set_rpm(static_cast<int32_t>(cmd * lim * (inv ? -1.0f : 1.0f)));
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
                    s.vesc_id          = static_cast<uint8_t>(motors[idx].value("id",               static_cast<int>(s.vesc_id)));
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
            t->vesc_id          = msg->motor_vesc_id;
            t->rpm_limit        = msg->rpm_limit;
            t->duty_cycle_limit = msg->duty_cycle_limit;
            t->control_mode     = msg->control_mode;
            t->inverted         = msg->inverted;
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
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_left_y_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_left_x_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_right_y_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_right_x_;
    
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr left_full_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr right_full_telemetry_pub;
    // telemetry publishers for flipper motors
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr left_flipper_full_telemetry_pub;
    rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr right_flipper_full_telemetry_pub;
    // timer and callback group for periodic telemetry
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telemetry_;

    float joystick_left_y  = 0.0f;
    float joystick_left_x  = 0.0f;
    float joystick_right_y = 0.0f;  // flipper delantero
    float joystick_right_x = 0.0f;  // flipper trasero

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