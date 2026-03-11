#include "rclcpp/rclcpp.hpp"
#include "robot_pkg/VESC.hpp"
#include "std_msgs/msg/float32.hpp" 
#include "std_msgs/msg/float32_multi_array.hpp" 
#include <chrono>
#include <functional>

using namespace std;
using namespace std::chrono_literals;

class BodyNode : public rclcpp::Node{
    public:
        //TODO: For now placeholder values
        int leftMotorId = 0;
        int rightMotorId = 1;
        int leftFlipperId = 2;
        int rightFlipperId = 3;

        int leftMaxRPM = 1000, rightMaxRPM = 1000, leftFlipperMaxRPM = 1000, rightFlipperMaxRPM = 1000;

        BodyNode() : Node("body_node"),
            leftMotor(
                declare_parameter<uint8_t>("left_id", leftMotorId),
                declare_parameter<int>("left_baudrate", 115200),
                declare_parameter<int>("left_timeout", 1000)),
                
            rightMotor(
                declare_parameter<uint8_t>("right_id", rightMotorId),
                declare_parameter<int>("right_baudrate", 115200),
                declare_parameter<int>("right_timeout", 1000)),
            leftFlipperMotor(
                declare_parameter<uint8_t>("left_flipper_id", leftFlipperId),
                declare_parameter<int>("left_flipper_baudrate", 115200),
                declare_parameter<int>("left_flipper_timeout", 1000)),
            rightFlipperMotor(
                declare_parameter<uint8_t>("right_flipper_id", rightFlipperId),
                declare_parameter<int>("right_flipper_baudrate", 115200),
                declare_parameter<int>("right_flipper_timeout", 1000))
                {

                if(leftMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Left motor body connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to left motor");
                }
                if(rightMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Right motor body connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to right motor");
                }
                if(leftFlipperMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Left flipper motor connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to left flipper motor");
                }
                if(rightFlipperMotor.autoConnect()){
                    RCLCPP_INFO(this->get_logger(), "Right flipper motor connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to right flipper motor");
                }

                callback_group_left_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
                callback_group_right_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                callback_group_left_flipper_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                callback_group_right_flipper_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                timer_left_ = this->create_wall_timer(
                    20ms, bind(&BodyNode::driveLeft, this), callback_group_left_);
                    
                timer_right_ = this->create_wall_timer(
                    20ms, bind(&BodyNode::driveRight, this), callback_group_right_);

                timer_left_flipper_ = this->create_wall_timer(
                    20ms, bind(&BodyNode::driveLeftFlipper, this), callback_group_left_flipper_);
                    
                timer_right_flipper_ = this->create_wall_timer(
                    20ms, bind(&BodyNode::driveRightFlipper, this), callback_group_right_flipper_);
                
                y_left_Axis_subscriber = create_subscription<std_msgs::msg::Float32>(
                "/arm/y_left_axis", 10,
                [this](const std_msgs::msg::Float32::SharedPtr msg) {
                    this->joystick_left_y = msg->data;
                });

                flipper_axis_subscriber = create_subscription<std_msgs::msg::Float32>(
                "/arm/flipper_axis", 10,
                [this](const std_msgs::msg::Float32::SharedPtr msg) {
                    this->joystick_flipper = msg->data;
                });


            left_full_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/body_left/telemetry", 10);
            right_full_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/body_right/telemetry", 10);

            left_flipper_full_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/body_left_flipper/telemetry", 10);
            right_flipper_full_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/body_right_flipper/telemetry", 10);

            // setup telemetry timer
            callback_group_telemetry_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            telemetry_timer_ = this->create_wall_timer(
                50ms, bind(&BodyNode::telemetry, this), callback_group_telemetry_);
            }
        ~BodyNode() {
            leftMotor.disconnect();
            rightMotor.disconnect();
            leftFlipperMotor.disconnect();
            rightFlipperMotor.disconnect();
        }


    private:

    void driveLeft(){
        if (!leftMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Left motor disconnected, reconnecting...");
            leftMotor.autoConnect();
            return;
        }
        leftMotor.set_rpm(joystick_left_y * leftMaxRPM);
    }

    void driveRight(){
        if (!rightMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Right motor disconnected, reconnecting...");
            rightMotor.autoConnect();
            return;
        }
        rightMotor.set_rpm(joystick_left_y * -rightMaxRPM);
    }

    void driveLeftFlipper(){
        if (!leftFlipperMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Left flipper disconnected, reconnecting...");
            leftFlipperMotor.autoConnect();
            return;
        }
        leftFlipperMotor.set_rpm(joystick_flipper * leftFlipperMaxRPM);
    }

    void driveRightFlipper(){
        if (!rightFlipperMotor.isConnected()) {
            RCLCPP_WARN(get_logger(), "Right flipper disconnected, reconnecting...");
            rightFlipperMotor.autoConnect();
            return;
        }
        rightFlipperMotor.set_rpm(joystick_flipper * -rightFlipperMaxRPM);
    }

    void telemetry(){
        VESCData m_telemetry;

        if(leftMotor.get_telemetry(m_telemetry)){
          std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(m_telemetry.motor_controller_id),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            left_full_telemetry_pub->publish(msg);
        }

        if(rightMotor.get_telemetry(m_telemetry)){
          std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(m_telemetry.motor_controller_id),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            right_full_telemetry_pub->publish(msg);
        }

        // flipper telemetry
        if(leftFlipperMotor.get_telemetry(m_telemetry)){
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(m_telemetry.motor_controller_id),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            left_flipper_full_telemetry_pub->publish(msg);
        }
        if(rightFlipperMotor.get_telemetry(m_telemetry)){
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(m_telemetry.motor_controller_id),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            right_flipper_full_telemetry_pub->publish(msg);
        }

    }

    VESC leftMotor;
    VESC rightMotor;
    VESC leftFlipperMotor;
    VESC rightFlipperMotor;


    rclcpp::CallbackGroup::SharedPtr callback_group_left_;
    rclcpp::CallbackGroup::SharedPtr callback_group_right_;
    rclcpp::CallbackGroup::SharedPtr callback_group_left_flipper_;
    rclcpp::CallbackGroup::SharedPtr callback_group_right_flipper_;

    rclcpp::TimerBase::SharedPtr timer_left_;
    rclcpp::TimerBase::SharedPtr timer_right_;
    rclcpp::TimerBase::SharedPtr timer_left_flipper_;
    rclcpp::TimerBase::SharedPtr timer_right_flipper_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_left_Axis_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr flipper_axis_subscriber;
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr left_full_telemetry_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr right_full_telemetry_pub;
    // telemetry publishers for flipper motors
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr left_flipper_full_telemetry_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr right_flipper_full_telemetry_pub;
    // timer and callback group for periodic telemetry
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telemetry_;

    float joystick_left_y = 0.0f;
    float joystick_flipper = 0.0f;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    rclcpp::shutdown();
    return 0;
}