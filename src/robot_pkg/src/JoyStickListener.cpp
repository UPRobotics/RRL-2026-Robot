#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp" //To publush integer or double values
#include "std_msgs/msg/bool.hpp" //To publush integer or double values

#include <functional>

class Joystick_Listener : public rclcpp::Node{
    public:
    Joystick_Listener() : Node("joystick_node"){
        subscriber__ =  this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",10,
            std::bind(&Joystick_Listener::callback, this, std::placeholders::_1));
    
    
        arm_cmd_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/arm/y_axis", 10
        );
    }

    private:
    void callback(sensor_msgs::msg::Joy::SharedPtr message){
       float updownJoyState = message->axes[1];

      RCLCPP_INFO(this->get_logger(),
       "Recieved - A button : '%d', Joystick up : '%.2f'",
       message->buttons[0],updownJoyState);


       float buttonAState = message->buttons[0]; // a button
        /*std_msgs::msg::Bool cmd;
        cmd.data = buttonAState;*/

        std_msgs::msg::Float32 yAxis;
        yAxis.data = updownJoyState;
        arm_cmd_pub_->publish(yAxis);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber__;
    //PUblisher of arm VAlues for logic
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr arm_cmd_pub_;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joystick_Listener>());
  rclcpp::shutdown();
  return 0;
}