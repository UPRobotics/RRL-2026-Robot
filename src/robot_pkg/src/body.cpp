#include "rclcpp/rclcpp.hpp"
#include "robot_pkg/VESC.hpp"
#include "std_msgs/msg/float32.hpp" 

using namespace std;

class BodyNode : public rclcpp::Node{
    public:
        //TODO: Remember to check for each port
        string leftPortName = "/dev/ttyACM0";
        string rightPortName = "/dev/ttyACM1";
        string leftFlipperPortName = "/dev/ttyACM2";
        string rightFlipperPortName = "/dev/ttyACM3";

        int leftMotorId = 0;
        int rightMotorId = 1;
        int leftFlipperId = 2;
        int rightFlipperId = 3;

        int leftMaxRPM = 1000, rightMaxRPM = 1000, leftFlipperMaxRPM = 1000, rightFlipperMaxRPM = 1000;

        BodyNode() : Node("body_node"),
            leftMotor(
                declare_parameter<string>("left_port", leftPortName),
                declare_parameter<uint8_t>("left_id", leftMotorId),
                declare_parameter<int>("left_baudrate", 115200),
                declare_parameter<int>("left_timeout", 1000)),
                
            rightMotor(
                declare_parameter<string>("right_port", rightPortName),
                declare_parameter<uint8_t>("right_id", rightMotorId),
                declare_parameter<int>("right_baudrate", 115200),
                declare_parameter<int>("right_timeout", 1000)),
            leftFlipperMotor(
                declare_parameter<string>("left_flipper_port", leftFlipperPortName),
                declare_parameter<uint8_t>("left_flipper_id", leftFlipperId),
                declare_parameter<int>("left_flipper_baudrate", 115200),
                declare_parameter<int>("left_flipper_timeout", 1000)),
            rightFlipperMotor(
                declare_parameter<string>("right_flipper_port", rightFlipperPortName),
                declare_parameter<uint8_t>("right_flipper_id", rightFlipperId),
                declare_parameter<int>("right_flipper_baudrate", 115200),
                declare_parameter<int>("right_flipper_timeout", 1000))
                {

                if(leftMotor.connect()){
                    RCLCPP_INFO(this->get_logger(), "Left motor body connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to left motor");
                }
                if(rightMotor.connect()){
                    RCLCPP_INFO(this->get_logger(), "Right motor body connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to right motor");
                }
                if(leftFlipperMotor.connect()){
                    RCLCPP_INFO(this->get_logger(), "Left flipper motor connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to left flipper motor");
                }
                if(rightFlipperMotor.connect()){
                    RCLCPP_INFO(this->get_logger(), "Right flipper motor connected.");
                } else{
                    RCLCPP_INFO(this->get_logger(), "Failed to connect to right flipper motor");
                }

                callback_group_left_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
                callback_group_right_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                callback_group_left_flipper_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                callback_group_right_flipper_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                timer_left_ = this->create_wall_timer(
                    50ms, bind(&BodyNode::driveLeft, this), callback_group_left_);
                    
                timer_right_ = this->create_wall_timer(
                    50ms, bind(&BodyNode::driveRight, this), callback_group_right_);

                timer_left_flipper_ = this->create_wall_timer(
                    50ms, bind(&BodyNode::driveLeftFlipper, this), callback_group_left_flipper_);
                    
                timer_right_flipper_ = this->create_wall_timer(
                    50ms, bind(&BodyNode::driveRightFlipper, this), callback_group_right_flipper_);
                
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
            }
        ~BodyNode() {
            leftMotor.disconnect();
            rightMotor.disconnect();
            leftFlipperMotor.disconnect();
            rightFlipperMotor.disconnect();
        }


    private:

    void driveLeft(){
        leftMotor.set_rpm(joystick_left_y * leftMaxRPM);
        RCLCPP_INFO(get_logger(),"left");

    }

    void driveRight(){
        rightMotor.set_rpm(joystick_left_y * -rightMaxRPM);
        RCLCPP_INFO(get_logger(),"right");
    }

    void driveLeftFlipper(){
        leftFlipperMotor.set_rpm(joystick_flipper * leftFlipperMaxRPM);
        RCLCPP_INFO(get_logger(),"left flipper");
    }

    void driveRightFlipper(){
        rightFlipperMotor.set_rpm(joystick_flipper * -rightFlipperMaxRPM);
        RCLCPP_INFO(get_logger(),"right flipper");
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