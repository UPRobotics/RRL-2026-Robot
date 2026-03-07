#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_pkg/VESC.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <iostream>


using namespace std::chrono_literals;
using namespace LibSerial;

class ArmNode : public rclcpp::Node
{
public:
     ArmNode() : Node("arm_node"),
       armMotor(
      declare_parameter<uint8_t>("id", 4),
      declare_parameter<int>("baudrate", 115200),
      declare_parameter<int>("timeout", 1000)){



        if (armMotor.autoConnect()) {
            RCLCPP_INFO(this->get_logger(), "VESC connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to VESC!");
        }

        // Create a timer to send commands at 20Hz (every 50ms)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ArmNode::timer_callback, this));

        y_left_axis_subscriber = create_subscription<std_msgs::msg::Float32>(
            "/y_left_axis", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->desired_rpms = msg->data;
            }
        );

        max_rpm_subscriber = create_subscription<std_msgs::msg::Float32>(
            "/telemetryJSON/arm_max_rpm", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->logged_rpm = msg->data;
            }
        );

        vesc_full_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/arm/telemetry", 10);
    }
    ~ArmNode() {
            armMotor.set_rpm(0); 
            armMotor.disconnect();
    }

private:
    void timer_callback() {
                    RCLCPP_INFO(get_logger(),"Logged rpms: %f",logged_rpm);

        if(armMotor.isConnected()){
        telemetry();
        armMotor.set_rpm(desired_rpms * 1500);
        }else{
            RCLCPP_INFO(get_logger(),"Not connected");
            armMotor.autoConnect();
            return;
        }  
    }

    void telemetry(){
            VESCData m_telemetry;

            if(armMotor.get_telemetry(m_telemetry)){
      std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            static_cast<float>(m_telemetry.rpm),
            static_cast<float>(m_telemetry.motor_controller_id),
            m_telemetry.input_voltage,
            m_telemetry.current_motor,
        };
        vesc_full_telemetry_pub->publish(msg);

            RCLCPP_INFO(get_logger(),
            "RPM: %d | Subscription: %.2f | Motor id: %f",
            m_telemetry.rpm,
            desired_rpms,
            m_telemetry.motor_controller_id);
            }
        }    

    VESC armMotor;

    /* Publishers and subscribers */
    rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_A_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_left_axis_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr max_rpm_subscriber;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vesc_full_telemetry_pub;
    /* A button test*/
    float desired_rpms = 0.0f;
    float logged_rpm = 0.0f;

    // ---- Parameters ----
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}