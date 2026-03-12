#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

// QoS para control en tiempo real:
// - BEST_EFFORT: sin retransmisión, menor latencia sobre WiFi
// - depth=1:    el robot siempre lee el valor más reciente, nunca uno viejo
static const rclcpp::QoS CONTROL_QOS = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();

class JoystickListener : public rclcpp::Node {
public:
    JoystickListener() : Node("joystick_node") {
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", CONTROL_QOS,
            std::bind(&JoystickListener::onJoy, this, std::placeholders::_1));

        // Tracks (diferencial tipo tanque)
        pub_left_y_  = create_publisher<std_msgs::msg::Float32>("/joystick/left_y",  CONTROL_QOS);
        pub_left_x_  = create_publisher<std_msgs::msg::Float32>("/joystick/left_x",  CONTROL_QOS);

        // Flippers (independientes)
        pub_right_y_ = create_publisher<std_msgs::msg::Float32>("/joystick/right_y", CONTROL_QOS); // flipper delantero
        pub_right_x_ = create_publisher<std_msgs::msg::Float32>("/joystick/right_x", CONTROL_QOS); // flipper trasero
    }

private:
    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 5) return;

        std_msgs::msg::Float32 left_y, left_x, right_y, right_x;
        left_y.data  = msg->axes[1]; // Left  joystick Y → tracks adelante/atrás
        left_x.data  = msg->axes[0]; // Left  joystick X → tracks rotación
        right_y.data = msg->axes[4]; // Right joystick Y → flipper delantero
        right_x.data = msg->axes[3]; // Right joystick X → flipper trasero

        pub_left_y_->publish(left_y);
        pub_left_x_->publish(left_x);
        pub_right_y_->publish(right_y);
        pub_right_x_->publish(right_x);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_left_y_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_left_x_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_right_y_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_right_x_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickListener>());
    rclcpp::shutdown();
    return 0;
}
