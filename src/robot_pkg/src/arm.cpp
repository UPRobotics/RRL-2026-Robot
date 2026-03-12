#include <algorithm>
#include <chrono>
#include <functional>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_pkg/VESC.hpp"
#include "robot_pkg/json.hpp"
#include "std_msgs/msg/float32.hpp"
#include "robot_msgs/msg/motor_config.hpp"
#include "robot_msgs/msg/motor_telemetry.hpp"

using namespace std::chrono_literals;
using namespace LibSerial;

struct MotorSettings {
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

        if (hipMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Hip motor connected.");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to HIp motor");
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

        loadConfig();

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
            20ms, bind(&ArmNode::setHipRPM, this), callback_group_hip_);
            
        timer_shoulder_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setShoulderRPM, this), callback_group_shoulder_);

        timer_elbow_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setElbowRPM, this), callback_group_elbow_);
            
        timer_roll_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setRollRPM, this), callback_group_roll_);
            
        timer_pitch_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setPitchRPM, this), callback_group_pitch_);
            
        timer_claw_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setClawRPM, this), callback_group_claw_);
        
        y_left_axis_subscriber = create_subscription<std_msgs::msg::Float32>(
            "/y_left_axis", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                this->desired_rpms = msg->data;
            });

        max_rpm_subscriber = create_subscription<std_msgs::msg::Float32>(
            "/telemetryJSON/arm_max_rpm", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                this->logged_rpm = msg->data;
            });

        hip_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_hip/telemetry", 10);
        shoulder_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_shoulder/telemetry", 10);
        elbow_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_elbow/telemetry", 10);
        roll_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_roll/telemetry", 10);
        pitch_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_pitch/telemetry", 10);
        claw_telemetry_pub = create_publisher<robot_msgs::msg::MotorTelemetry>(
            "/arm_claw/telemetry", 10);

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

    void setHipTarget(int32_t rpm)      { target_hip_rpm.store(rpm); }
    void setShoulderTarget(int32_t rpm) { target_shoulder_rpm.store(rpm); }
    void setElbowTarget(int32_t rpm)    { target_elbow_rpm.store(rpm); }
    void setRollTarget(int32_t rpm)     { target_roll_rpm.store(rpm); }
    void setPitchTarget(int32_t rpm)    { target_pitch_rpm.store(rpm); }
    void setClawTarget(int32_t rpm)     { target_claw_rpm.store(rpm); }

private:
    void setShoulderRPM(){
        float lim; { std::lock_guard<std::mutex> lk(settings_mutex_); lim = shoulder_.rpm_limit; }
        shoulderMotor.set_rpm(static_cast<int32_t>(
            std::clamp(static_cast<float>(target_shoulder_rpm.load()), -lim, lim)));
    }

    void setElbowRPM() {
        float lim; { std::lock_guard<std::mutex> lk(settings_mutex_); lim = elbow_.rpm_limit; }
        elbowMotor.set_rpm(static_cast<int32_t>(
            std::clamp(static_cast<float>(target_elbow_rpm.load()), -lim, lim)));
    }

    void setRollRPM() {
        float lim; { std::lock_guard<std::mutex> lk(settings_mutex_); lim = roll_.rpm_limit; }
        rollMotor.set_rpm(static_cast<int32_t>(
            std::clamp(static_cast<float>(target_roll_rpm.load()), -lim, lim)));
    }

    void setPitchRPM() {
        float lim; { std::lock_guard<std::mutex> lk(settings_mutex_); lim = pitch_.rpm_limit; }
        pitchMotor.set_rpm(static_cast<int32_t>(
            std::clamp(static_cast<float>(target_pitch_rpm.load()), -lim, lim)));
    }

    void setClawRPM() {
        float lim; { std::lock_guard<std::mutex> lk(settings_mutex_); lim = claw_.rpm_limit; }
        clawMotor.set_rpm(static_cast<int32_t>(
            std::clamp(static_cast<float>(target_claw_rpm.load()), -lim, lim)));
    }

    void setHipRPM() {
        float lim; { std::lock_guard<std::mutex> lk(settings_mutex_); lim = hip_.rpm_limit; }
        hipMotor.set_rpm(static_cast<int32_t>(
            std::clamp(static_cast<float>(target_hip_rpm.load()), -lim, lim)));
    }

    void telemetry(){
        VESCData m_telemetry;

        if(hipMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Cadera";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 0;
            msg.inverted     = false;
            hip_telemetry_pub->publish(msg);
        }

        if(shoulderMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Hombro";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 1;
            msg.inverted     = false;
            shoulder_telemetry_pub->publish(msg);
        }

        if(elbowMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Codo";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 1;
            msg.inverted     = false;
            elbow_telemetry_pub->publish(msg);
        }

        if(rollMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Roll";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 0;
            msg.inverted     = false;
            roll_telemetry_pub->publish(msg);
        }

        if(pitchMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Pitch";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 1;
            msg.inverted     = false;
            pitch_telemetry_pub->publish(msg);
        }

        if(clawMotor.get_telemetry(m_telemetry)){
            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id     = m_telemetry.motor_controller_id;
            msg.motor_name   = "Grip";
            msg.rpm          = m_telemetry.rpm;
            msg.duty_cycle   = m_telemetry.duty_cycle;
            msg.voltage      = m_telemetry.input_voltage;
            msg.control_mode = 1;
            msg.inverted     = false;
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
                    s.rpm_limit        = motors[idx].value("rpm_limit",        s.rpm_limit);
                    s.duty_cycle_limit = motors[idx].value("duty_cycle_limit", s.duty_cycle_limit);
                    s.control_mode     = static_cast<uint8_t>(motors[idx].value("control_mode", static_cast<int>(s.control_mode)));
                    s.inverted         = motors[idx].value("inverted",         s.inverted);
                }
            };
            std::lock_guard<std::mutex> lk(settings_mutex_);
            read(4, hip_); read(5, shoulder_); read(6, elbow_);
            read(7, roll_); read(8, pitch_);   read(9, claw_);
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
        switch (msg->config_index) {
            case 4: t = &hip_;      break;
            case 5: t = &shoulder_; break;
            case 6: t = &elbow_;    break;
            case 7: t = &roll_;     break;
            case 8: t = &pitch_;    break;
            case 9: t = &claw_;     break;
            default: return;  // Not an arm motor
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


    std::atomic<int32_t> target_hip_rpm{0};
    std::atomic<int32_t> target_shoulder_rpm{0};
    std::atomic<int32_t> target_elbow_rpm{0};
    std::atomic<int32_t> target_roll_rpm{0};
    std::atomic<int32_t> target_pitch_rpm{0};
    std::atomic<int32_t> target_claw_rpm{0};


    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telemetry_;

        /* Publishers and subscribers */

    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_A_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_left_axis_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr max_rpm_subscriber;

    /* A button test*/
    float desired_rpms = 0.0f;
    float logged_rpm = 0.0f;

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