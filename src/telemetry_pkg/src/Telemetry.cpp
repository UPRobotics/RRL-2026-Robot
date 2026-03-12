#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_msgs/msg/motor_config.hpp"
#include "robot_msgs/msg/motor_telemetry.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <fstream>
#include <chrono>
#include "telemetry_pkg/json.hpp"

using json = nlohmann::json;

// Telemetría: BEST_EFFORT evita colas de retransmisión que congestionan WiFi
static const rclcpp::QoS TELEM_QOS = rclcpp::QoS(rclcpp::KeepLast(5))
    .best_effort()
    .durability_volatile();

// Config: RELIABLE (debe llegar, no es tiempo real)
static const rclcpp::QoS CONFIG_QOS = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable();

class Telemetry_publisher : public rclcpp::Node{
    public:
    Telemetry_publisher() : Node("telemetry_node"){

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&Telemetry_publisher::callback, this));


        std::ifstream input_file(path);
        data = json::parse(input_file);

        // Publishers  
        armMaxRpm = create_publisher<std_msgs::msg::Float32>(
            "/telemetryJSON/arm_max_rpm", 10
        );

        // Config: relay from ground station UI → robot (RELIABLE)
        robotConfigPub_ = create_publisher<robot_msgs::msg::MotorConfig>(
            "/robot_config/update", CONFIG_QOS
        );
        groundStationConfigSub_ = create_subscription<robot_msgs::msg::MotorConfig>(
            "/ground_station/motor_config", CONFIG_QOS,
            [this](const robot_msgs::msg::MotorConfig::SharedPtr msg) {
                robotConfigPub_->publish(*msg);
                RCLCPP_INFO(this->get_logger(),
                    "Relaying config update for motor [%u] %s: rpm_limit=%.1f  duty=%.3f  mode=%u  inv=%s",
                    msg->config_index, msg->motor_name.c_str(),
                    msg->rpm_limit, msg->duty_cycle_limit, msg->control_mode,
                    msg->inverted ? "yes" : "no");
            }
        );

        // Subscribers for arm telemetry
        armHipTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/arm_hip/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::armHipTelemetryCallback, this, std::placeholders::_1)
        );
        armShoulderTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/arm_shoulder/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::armShoulderTelemetryCallback, this, std::placeholders::_1)
        );
        armElbowTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/arm_elbow/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::armElbowTelemetryCallback, this, std::placeholders::_1)
        );
        armRollTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/arm_roll/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::armRollTelemetryCallback, this, std::placeholders::_1)
        );
        armPitchTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/arm_pitch/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::armPitchTelemetryCallback, this, std::placeholders::_1)
        );
        armClawTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/arm_claw/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::armClawTelemetryCallback, this, std::placeholders::_1)
        );

        // Subscribers for body telemetry
        bodyLeftTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/body_left/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::bodyLeftTelemetryCallback, this, std::placeholders::_1)
        );
        bodyRightTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/body_right/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::bodyRightTelemetryCallback, this, std::placeholders::_1)
        );
        bodyLeftFlipperTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/body_left_flipper/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::bodyLeftFlipperTelemetryCallback, this, std::placeholders::_1)
        );
        bodyRightFlipperTelemSub = create_subscription<robot_msgs::msg::MotorTelemetry>(
            "/body_right_flipper/telemetry", TELEM_QOS,
            std::bind(&Telemetry_publisher::bodyRightFlipperTelemetryCallback, this, std::placeholders::_1)
        );
    }

    private:
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("telemetry_pkg");
    std::string path = package_share_directory + "/config/config.json";
    void callback(){
        auto msg = std_msgs::msg::Float32();
        msg.data = data["motors"][4]["rpm_limit"].get<float>();
        armMaxRpm->publish(msg);
    }

    // callbacks for arm telemetry topics
    void armHipTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }
    void armShoulderTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }
    void armElbowTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }
    void armRollTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }
    void armPitchTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }
    void armClawTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }

    // callbacks for body telemetry topics
    void bodyLeftTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }
    void bodyRightTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }
    void bodyLeftFlipperTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }
    void bodyRightFlipperTelemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "[%s] id=%u  rpm=%d  duty=%.3f  voltage=%.1fV  mode=%u  inverted=%s",
            msg->motor_name.c_str(), msg->motor_id, msg->rpm,
            msg->duty_cycle, msg->voltage, msg->control_mode,
            msg->inverted ? "true" : "false");
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr armMaxRpm;
    rclcpp::Publisher<robot_msgs::msg::MotorConfig>::SharedPtr robotConfigPub_;
    rclcpp::Subscription<robot_msgs::msg::MotorConfig>::SharedPtr groundStationConfigSub_;
    // arm telemetry subscriptions
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr armHipTelemSub;
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr armShoulderTelemSub;
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr armElbowTelemSub;
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr armRollTelemSub;
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr armPitchTelemSub;
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr armClawTelemSub;
    // body telemetry subscriptions
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr bodyLeftTelemSub;
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr bodyRightTelemSub;
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr bodyLeftFlipperTelemSub;
    rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr bodyRightFlipperTelemSub;
    json data;

    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry_publisher>());
  rclcpp::shutdown();
  return 0;
}