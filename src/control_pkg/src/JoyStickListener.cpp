#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "robot_msgs/msg/joystick_axes.hpp"

// BEST_EFFORT + depth=1: always the latest value, no retransmission delay
static const rclcpp::QoS CONTROL_QOS = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();

class JoystickListener : public rclcpp::Node {
public:
    JoystickListener() : Node("joystick_node") {
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", CONTROL_QOS,
            std::bind(&JoystickListener::onJoy, this, std::placeholders::_1));

        pub_ = create_publisher<robot_msgs::msg::JoystickAxes>("/joystick/axes", CONTROL_QOS);
    }

private:
    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 5) return;

        robot_msgs::msg::JoystickAxes out;
        out.left_x  = msg->axes[0]; // rotation
        out.left_y  = msg->axes[1]; // forward / back
        out.right_x = msg->axes[3]; // rear flipper / arm roll
        out.right_y = msg->axes[4]; // front flipper / arm elbow
        pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<robot_msgs::msg::JoystickAxes>::SharedPtr pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickListener>());
    rclcpp::shutdown();
    return 0;
}
