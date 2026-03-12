#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_msgs/msg/motor_config.hpp"
#include "robot_msgs/msg/motor_telemetry.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <fstream>
#include <chrono>
#include <mutex>
#include <array>
#include "telemetry_pkg/json.hpp"

using json = nlohmann::json;

// Telemetria: BEST_EFFORT evita colas de retransmision que congestionan WiFi
static const rclcpp::QoS TELEM_QOS = rclcpp::QoS(rclcpp::KeepLast(5))
    .best_effort()
    .durability_volatile();

// Config: RELIABLE (debe llegar, no es tiempo real)
static const rclcpp::QoS CONFIG_QOS = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable();

static constexpr int NUM_MOTORS = 10;

// Topic-to-config_index mapping (matches config.json motor array order)
struct MotorTopicInfo {
    const char* topic;
    int config_index;
};

static const std::array<MotorTopicInfo, NUM_MOTORS> MOTOR_TOPICS = {{
    {"/body_left_flipper/telemetry",  0},
    {"/body_left/telemetry",          1},
    {"/body_right/telemetry",         2},
    {"/body_right_flipper/telemetry", 3},
    {"/arm_hip/telemetry",            4},
    {"/arm_shoulder/telemetry",       5},
    {"/arm_elbow/telemetry",          6},
    {"/arm_roll/telemetry",           7},
    {"/arm_pitch/telemetry",          8},
    {"/arm_claw/telemetry",           9},
}};

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

        aggregatedPub_ = create_publisher<std_msgs::msg::String>(
            "/telemetry/aggregated", TELEM_QOS
        );

        // Config: relay from ground station UI -> robot (RELIABLE)
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

        // Subscribe to all motor telemetry topics using a generic callback
        for (const auto& info : MOTOR_TOPICS) {
            telemSubs_.push_back(
                create_subscription<robot_msgs::msg::MotorTelemetry>(
                    info.topic, TELEM_QOS,
                    [this, idx = info.config_index](const robot_msgs::msg::MotorTelemetry::SharedPtr msg) {
                        telemetryCallback(msg, idx);
                    }
                )
            );
        }
    }

    private:
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("telemetry_pkg");
    std::string path = package_share_directory + "/config/config.json";

    void telemetryCallback(const robot_msgs::msg::MotorTelemetry::SharedPtr msg, int configIndex) {
        std::lock_guard<std::mutex> lock(telemMutex_);
        auto& m = latestTelemetry_[configIndex];
        m = *msg;
        motorReceived_[configIndex] = true;
    }

    void callback(){
        // Publish arm max RPM (existing behavior)
        auto rpm_msg = std_msgs::msg::Float32();
        rpm_msg.data = data["motors"][4]["rpm_limit"].get<float>();
        armMaxRpm->publish(rpm_msg);

        // Publish aggregated JSON with all motor telemetry
        json j;
        j["timestamp_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        j["motors"] = json::array();

        {
            std::lock_guard<std::mutex> lock(telemMutex_);
            for (int i = 0; i < NUM_MOTORS; ++i) {
                if (!motorReceived_[i]) continue;
                const auto& m = latestTelemetry_[i];
                j["motors"].push_back({
                    {"config_index",  i},
                    {"motor_id",      m.motor_id},
                    {"motor_name",    m.motor_name},
                    {"rpm",           m.rpm},
                    {"duty_cycle",    m.duty_cycle},
                    {"voltage",       m.voltage},
                    {"control_mode",  m.control_mode},
                    {"inverted",      m.inverted}
                });
            }
        }

        auto str_msg = std_msgs::msg::String();
        str_msg.data = j.dump();
        aggregatedPub_->publish(str_msg);
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr armMaxRpm;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aggregatedPub_;
    rclcpp::Publisher<robot_msgs::msg::MotorConfig>::SharedPtr robotConfigPub_;

    // Subscriptions
    rclcpp::Subscription<robot_msgs::msg::MotorConfig>::SharedPtr groundStationConfigSub_;
    std::vector<rclcpp::Subscription<robot_msgs::msg::MotorTelemetry>::SharedPtr> telemSubs_;

    // Aggregated motor state
    std::mutex telemMutex_;
    std::array<robot_msgs::msg::MotorTelemetry, NUM_MOTORS> latestTelemetry_;
    std::array<bool, NUM_MOTORS> motorReceived_{};

    json data;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry_publisher>());
  rclcpp::shutdown();
  return 0;
}
