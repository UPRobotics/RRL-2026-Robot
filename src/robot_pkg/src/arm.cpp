#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_pkg/VESC.hpp"
#include "std_msgs/msg/float32.hpp" 


using namespace std::chrono_literals;
using namespace LibSerial;

class ArmNode : public rclcpp::Node
{
public:
     ArmNode() : Node("arm_node"),
       vesc_(
      declare_parameter<std::string>("port", "/dev/ttyACM0"),
      declare_parameter<uint8_t>("id", 0),
      declare_parameter<int>("baudrate", 115200),
      declare_parameter<int>("timeout", 1000)){

        if (vesc_.connect()) {
            RCLCPP_INFO(this->get_logger(), "VESC connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to VESC!");
        }


        // Create a timer to send commands at 20Hz (every 50ms)
        timer_ = this->create_wall_timer(
            50ms, std::bind(&ArmNode::timer_callback, this));


        y_Axis_subscriber = create_subscription<std_msgs::msg::Float32>(
        "/arm/y_axis", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            this->desired_rpms = msg->data;
        }
);
    }
    ~ArmNode() {
            vesc_.set_rpm(0); 
            vesc_.disconnect();
        
    }

private:
    void timer_callback() {

        telemetry();
//        int32_t target_rpm = 1500; 
      //  if(test){
        vesc_.set_rpm(desired_rpms * 1500);
      //  }else{
//vesc_.set_rpm(0);
//        }     
    }

    void telemetry(){
            VESCData m_telemetry;

            if(vesc_.get_telemetry(m_telemetry)){
            RCLCPP_INFO(get_logger(),
            "RPM: %d | Subscription: %.2f",
            m_telemetry.rpm,
            desired_rpms);
            //RCLCPP_DEBUG(this->get_logger(), "Sending RPM: %d", 15000);
            }
        }    


    VESC vesc_;

    /* Publishers and subscribers */
    rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_A_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_Axis_subscriber;

    /* A button test*/
    bool test = false;

    float desired_rpms = 0.0f;

    // ---- Parameters ----
    std::string port_;
    int baudrate_;
    int run_rpm_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}