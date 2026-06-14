#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/motor_telemetry.hpp"
#include <chrono>
#include <random>
#include <vector>
#include <string>

// Same QoS as real arm_node / body_node
static const rclcpp::QoS TELEM_QOS = rclcpp::QoS(rclcpp::KeepLast(5))
    .best_effort()
    .durability_volatile();

struct SimMotor {
    std::string topic;
    uint8_t     motor_id;
    std::string motor_name;
    uint8_t     control_mode;
    bool        inverted;

    // Current simulated values (refreshed every 5s)
    int32_t rpm            = 0;
    float   duty_cycle     = 0.0f;
    float   voltage        = 0.0f;
    float   current_in     = 0.0f;
    float   current_motor  = 0.0f;
    float   position       = 0.0f;
    uint8_t fault_code     = 0;
    float   rpm_limit      = 5000.0f;
    float   duty_cycle_limit = 0.5f;
};

// All 10 possible motors matching the real system
// Fields: topic, motor_id, motor_name, control_mode, inverted,
//         rpm, duty_cycle, voltage, current_in, current_motor, position, fault_code, rpm_limit, duty_cycle_limit
static const std::vector<SimMotor> ALL_MOTORS = {
    {"/body_left_flipper/telemetry",  0, "Flipper Trasero",   0, true,  0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 5000.0f, 0.1f},
    {"/body_left/telemetry",          1, "Track Izquierdo",   0, false, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 5000.0f, 0.5f},
    {"/body_right/telemetry",         2, "Track Derecho",     0, true,  0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 5000.0f, 0.5f},
    {"/body_right_flipper/telemetry", 3, "Flipper Delantero", 0, false, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 5000.0f, 0.1f},
    {"/arm_hip/telemetry",            4, "Cadera",            0, false, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 3000.0f, 0.1f},
    {"/arm_shoulder/telemetry",       5, "Hombro",            1, false, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 3000.0f, 0.1f},
    {"/arm_elbow/telemetry",          6, "Codo",              1, false, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 3000.0f, 0.1f},
    {"/arm_roll/telemetry",           7, "Roll",              0, false, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 2000.0f, 0.1f},
    {"/arm_pitch/telemetry",          8, "Pitch",             1, false, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1000.0f, 0.1f},
    {"/arm_claw/telemetry",           9, "Grip",              1, false, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1000.0f, 0.1f},
};

class MotorSimNode : public rclcpp::Node {
public:
    MotorSimNode() : Node("motor_sim_node") {
        // Parameter: number of motors to simulate (1-10)
        this->declare_parameter<int>("num_motors", 10);
        int numMotors = this->get_parameter("num_motors").as_int();
        numMotors = std::clamp(numMotors, 1, 10);

        RCLCPP_INFO(this->get_logger(), "Motor simulator starting with %d motors", numMotors);

        // Set up active motors and publishers
        for (int i = 0; i < numMotors; ++i) {
            activeMotors_.push_back(ALL_MOTORS[i]);
            publishers_.push_back(
                this->create_publisher<robot_msgs::msg::MotorTelemetry>(
                    ALL_MOTORS[i].topic, TELEM_QOS)
            );
        }

        // Generate initial random values
        refreshRandomValues();

        // Publish telemetry every 50ms (same rate as real nodes)
        publishTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MotorSimNode::publishTelemetry, this)
        );

        // Refresh random values every 5 seconds
        refreshTimer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&MotorSimNode::refreshRandomValues, this)
        );

        for (int i = 0; i < numMotors; ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%d] %s -> %s",
                activeMotors_[i].motor_id,
                activeMotors_[i].motor_name.c_str(),
                activeMotors_[i].topic.c_str());
        }
    }

private:
    void refreshRandomValues() {
        std::uniform_int_distribution<int32_t> rpmDist(-5000, 5000);
        std::uniform_real_distribution<float>  dutyDist(0.0f, 1.0f);
        std::uniform_real_distribution<float>  voltDist(18.0f, 25.5f);
        std::uniform_real_distribution<float>  currentInDist(0.0f, 15.0f);
        std::uniform_real_distribution<float>  currentMotorDist(0.0f, 30.0f);
        std::uniform_real_distribution<float>  posDist(0.0f, 360.0f);

        for (auto& m : activeMotors_) {
            m.rpm           = rpmDist(rng_);
            m.duty_cycle    = dutyDist(rng_);
            m.voltage       = voltDist(rng_);
            m.current_in    = currentInDist(rng_);
            m.current_motor = currentMotorDist(rng_);
            m.position      = posDist(rng_);
        }

        RCLCPP_INFO(this->get_logger(), "Refreshed random values for %zu motors",
            activeMotors_.size());
    }

    void publishTelemetry() {
        // Add small per-tick noise so values aren't static between refreshes
        std::uniform_int_distribution<int32_t> rpmNoise(-50, 50);
        std::uniform_real_distribution<float>  dutyNoise(-0.01f, 0.01f);
        std::uniform_real_distribution<float>  voltNoise(-0.1f, 0.1f);
        std::uniform_real_distribution<float>  currentNoise(-0.2f, 0.2f);

        for (size_t i = 0; i < activeMotors_.size(); ++i) {
            const auto& m = activeMotors_[i];

            robot_msgs::msg::MotorTelemetry msg;
            msg.motor_id         = m.motor_id;
            msg.motor_name       = m.motor_name;
            msg.rpm              = m.rpm + rpmNoise(rng_);
            msg.duty_cycle       = std::clamp(m.duty_cycle + dutyNoise(rng_), 0.0f, 1.0f);
            msg.voltage          = std::max(0.0f, m.voltage + voltNoise(rng_));
            msg.control_mode     = m.control_mode;
            msg.inverted         = m.inverted;
            msg.current_in       = std::max(0.0f, m.current_in + currentNoise(rng_));
            msg.current_motor    = std::max(0.0f, m.current_motor + currentNoise(rng_));
            msg.position         = m.position;
            msg.fault_code       = m.fault_code;
            msg.rpm_limit        = m.rpm_limit;
            msg.duty_cycle_limit = m.duty_cycle_limit;

            publishers_[i]->publish(msg);
        }
    }

    std::vector<SimMotor> activeMotors_;
    std::vector<rclcpp::Publisher<robot_msgs::msg::MotorTelemetry>::SharedPtr> publishers_;
    rclcpp::TimerBase::SharedPtr publishTimer_;
    rclcpp::TimerBase::SharedPtr refreshTimer_;
    std::mt19937 rng_{std::random_device{}()};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorSimNode>());
    rclcpp::shutdown();
    return 0;
}
