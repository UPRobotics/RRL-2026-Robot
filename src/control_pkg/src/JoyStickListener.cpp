#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "robot_msgs/msg/control_input.hpp"

#include <algorithm>
#include <string>

// BEST_EFFORT + depth=1: always the latest value, no retransmission delay
static const rclcpp::QoS CONTROL_QOS = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();

struct ControllerLayout {
    // Axis indices
    int right_x;
    int right_y;
    int trigger_left;
    int trigger_right;
    // Button indices
    int bumper_left;
    int bumper_right;
};

//                              rx  ry  tL  tR  bL  bR
static constexpr ControllerLayout XBOX_AXES = {3, 4, 2, 5, 4, 5};
// PS4: triggers are also exposed as buttons[4]/[5], so L1/R1 bumpers shift to 6/7
static constexpr ControllerLayout PS4_AXES  = {2, 3, 5, 4, 6, 7};

class JoystickListener : public rclcpp::Node {
public:
    JoystickListener() : Node("joystick_node") {
        // "auto"  — detect from initial trigger values on first Joy message
        // "xbox"  — force Xbox layout
        // "ps4"   — force PS4 layout
        declare_parameter("controller_type", std::string("auto"));
        // Override bumper button indices if auto-detection gets them wrong.
        // Set to -1 to use the layout defaults.
        declare_parameter("bumper_left_btn",  -1);
        declare_parameter("bumper_right_btn", -1);

        std::string ctrl_type = get_parameter("controller_type").as_string();
        bumper_left_override_  = get_parameter("bumper_left_btn").as_int();
        bumper_right_override_ = get_parameter("bumper_right_btn").as_int();

        if (ctrl_type == "xbox") {
            applyLayout(XBOX_AXES, "Xbox (manual override)");
        } else if (ctrl_type == "ps4") {
            applyLayout(PS4_AXES, "PS4 (manual override)");
        }
        // "auto" → layout_ stays default-initialised; detected on first Joy message

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", CONTROL_QOS,
            std::bind(&JoystickListener::onJoy, this, std::placeholders::_1));

        ctrl_pub_ = create_publisher<robot_msgs::msg::ControlInput>(
            "/control/input", CONTROL_QOS);

        mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
            "/robot/mode", CONTROL_QOS);

        RCLCPP_INFO(get_logger(), "Joystick node started. Mode: MOVEMENT (0). Toggle: B button.");
        if (ctrl_type == "auto") {
            RCLCPP_INFO(get_logger(), "Controller layout: auto-detecting from first Joy message...");
        }
    }

private:
    void applyLayout(const ControllerLayout & l, const char * label) {
        layout_         = l;
        layout_ready_   = true;
        if (bumper_left_override_  >= 0) layout_.bumper_left  = bumper_left_override_;
        if (bumper_right_override_ >= 0) layout_.bumper_right = bumper_right_override_;
        RCLCPP_INFO(get_logger(),
            "Controller layout: %s  |  axes: rx=%d ry=%d tL=%d tR=%d  |  bumpers: L=%d R=%d",
            label,
            layout_.right_x, layout_.right_y, layout_.trigger_left, layout_.trigger_right,
            layout_.bumper_left, layout_.bumper_right);
    }

    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 8 || msg->buttons.size() < 6) return;

        // Lazy auto-detection: PS4 triggers initialize at 1.0; Xbox triggers at 0.0.
        // axes[4] is RS_Y on Xbox (rests at 0) or RT on PS4 (rests at 1).
        if (!layout_ready_) {
            bool is_ps4 = (msg->axes[4] > 0.5f);
            applyLayout(is_ps4 ? PS4_AXES : XBOX_AXES,
                        is_ps4 ? "PS4 (auto-detected)" : "Xbox (auto-detected)");
        }

        // Rising-edge detection for mode toggle (buttons[1] = B on Xbox / Circle on PS4)
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
        auto normTrigger = [](float raw) {
            return std::max(0.0f, (1.0f - raw) / 2.0f);
        };

        const int bl = layout_.bumper_left;
        const int br = layout_.bumper_right;

        robot_msgs::msg::ControlInput out;
        out.mode          = mode_;
        out.left_x        = msg->axes[0];
        out.left_y        = msg->axes[1];
        out.right_x       = msg->axes[layout_.right_x];
        out.right_y       = msg->axes[layout_.right_y];
        out.trigger_left  = normTrigger(msg->axes[layout_.trigger_left]);
        out.trigger_right = normTrigger(msg->axes[layout_.trigger_right]);
        out.dpad_x        = msg->axes[6];
        out.dpad_y        = msg->axes[7];
        out.bumper_left   = (static_cast<int>(msg->buttons.size()) > bl && msg->buttons[bl] != 0);
        out.bumper_right  = (static_cast<int>(msg->buttons.size()) > br && msg->buttons[br] != 0);
        ctrl_pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<robot_msgs::msg::ControlInput>::SharedPtr ctrl_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;

    ControllerLayout layout_         = PS4_AXES;   // overwritten on first message if "auto"
    bool             layout_ready_   = false;
    int              bumper_left_override_  = -1;
    int              bumper_right_override_ = -1;
    uint8_t          mode_           = 0;           // 0=MOVEMENT, 1=ARM
    bool             prev_mode_btn_  = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickListener>());
    rclcpp::shutdown();
    return 0;
}
