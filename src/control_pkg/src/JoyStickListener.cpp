#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "robot_msgs/msg/control_input.hpp"

#include <algorithm>

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

        ctrl_pub_ = create_publisher<robot_msgs::msg::ControlInput>(
            "/control/input", CONTROL_QOS);

        mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
            "/robot/mode", CONTROL_QOS);

        RCLCPP_INFO(get_logger(), "Joystick node started. Mode: MOVEMENT (0). Toggle: B button.");
    }

private:
    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 8 || msg->buttons.size() < 6) return;

        // Rising-edge detection for mode toggle (buttons[1] = B on Xbox)
        bool mode_btn = (msg->buttons[1] != 0);
        if (mode_btn && !prev_mode_btn_) {
            mode_ = (mode_ == 0) ? 1 : 0;
            std_msgs::msg::UInt8 m;
            m.data = mode_;
            mode_pub_->publish(m);
            RCLCPP_INFO(get_logger(), "Mode switched to %s", mode_ == 0 ? "MOVEMENT" : "ARM");
        }
        prev_mode_btn_ = mode_btn;

        // Trigger normalization: raw=1.0 (not pressed) → 0.0, raw=-1.0 (fully pressed) → 1.0
        // Works for PS4/Xbox controllers via joy_node on Linux.
        auto normTrigger = [](float raw) {
            return std::max(0.0f, (1.0f - raw) / 2.0f);
        };

        robot_msgs::msg::ControlInput out;
        out.mode          = mode_;
        out.left_x        = msg->axes[0];
        out.left_y        = msg->axes[1];
        out.right_x       = msg->axes[3];
        out.right_y       = msg->axes[4];
        out.trigger_left  = normTrigger(msg->axes[2]);
        out.trigger_right = normTrigger(msg->axes[5]);
        out.dpad_x        = msg->axes[6];
        out.dpad_y        = msg->axes[7];
        out.bumper_left   = (msg->buttons.size() > 4 && msg->buttons[4] != 0);
        out.bumper_right  = (msg->buttons.size() > 5 && msg->buttons[5] != 0);
        ctrl_pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<robot_msgs::msg::ControlInput>::SharedPtr ctrl_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;

    uint8_t mode_          = 0;     // 0=MOVEMENT, 1=ARM
    bool    prev_mode_btn_ = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickListener>());
    rclcpp::shutdown();
    return 0;
}
