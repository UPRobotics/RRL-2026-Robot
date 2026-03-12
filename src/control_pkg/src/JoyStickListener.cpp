#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include <functional>

class Joystick_Listener : public rclcpp::Node{
    public:
    Joystick_Listener() : Node("joystick_node"){
        // Joystick SUbscriber
        subscriber__ =  this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",10,
            std::bind(&Joystick_Listener::callback, this, std::placeholders::_1));
    
        // Publishers  
        yLeftAxis = create_publisher<std_msgs::msg::Float32>(
            "y_left_axis", 10
        );

        xLeftAxis = create_publisher<std_msgs::msg::Float32>(
            "x_left_axis", 10
        );

        yRightAxis = create_publisher<std_msgs::msg::Float32>(
            "y_right_axis", 10
        );

        xRightAxis = create_publisher<std_msgs::msg::Float32>(
            "x_right_axis", 10
        );
    }

    private:
    void callback(sensor_msgs::msg::Joy::SharedPtr message){
        std_msgs::msg::Float32 lx, ly, rx, ry;

        ly.data = message->axes[1]; // Left Y
        lx.data = message->axes[0]; // Left X
        ry.data = message->axes[4]; // Right Y
        rx.data = message->axes[3]; // Right X

        yLeftAxis->publish(ly);
        xLeftAxis->publish(lx);
        yRightAxis->publish(ry);
        xRightAxis->publish(rx);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber__;
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yLeftAxis;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr xLeftAxis;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yRightAxis;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr xRightAxis;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joystick_Listener>());
  rclcpp::shutdown();
  return 0;
}