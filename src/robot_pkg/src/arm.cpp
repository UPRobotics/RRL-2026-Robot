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

    int hipMotorId = 4;
    int shoulderMotorId = 5;
    int elbowMotorId = 6;
    int rollMotorId = 7;
    int pitchMotorId = 8;
    int clawMotorId= 9;



ArmNode() : Node("arm_node"),
                hipMotor(
                    declare_parameter<uint8_t>("hip_id", hipMotorId),
                    declare_parameter<int>("hip_baudrate", 115200),
                    declare_parameter<int>("hip_timeout", 1000)),
                shoulderMotor(
                    declare_parameter<uint8_t>("shoulder_id", shoulderMotorId),
                    declare_parameter<int>("shoulder_baudrate", 115200),
                    declare_parameter<int>("shoulder_timeout", 1000)),
                elbowMotor(
                    declare_parameter<uint8_t>("elbow_id", elbowMotorId),
                    declare_parameter<int>("elbow_baudrate", 115200),
                    declare_parameter<int>("elbow_timeout", 1000)),
                rollMotor(
                    declare_parameter<uint8_t>("roll_id", rollMotorId),
                    declare_parameter<int>("roll_baudrate", 115200),
                    declare_parameter<int>("roll_timeout", 1000)),
                pitchMotor(
                    declare_parameter<uint8_t>("pitch_id", pitchMotorId),
                    declare_parameter<int>("pitch_baudrate", 115200),
                    declare_parameter<int>("pitch_timeout", 1000)),
                clawMotor(
                    declare_parameter<uint8_t>("claw_id", clawMotorId),
                    declare_parameter<int>("claw_baudrate", 115200),
                    declare_parameter<int>("claw_timeout", 1000)
                )                                        
    {

        if (hipMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Hip motor connected.");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to HIp motor");
        }
        if (shoulderMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Shoulder motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Shoulder motor.");
        }
        if (elbowMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Elbow motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Elbow motor.");
        }
        if (rollMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Roll motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Roll motor.");
        }
        if (pitchMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Pitch motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Pitch motor.");
        }
        if (clawMotor.autoConnect()){
            RCLCPP_INFO(this->get_logger(), "Claw motor connected.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Claw motor.");
        }

        callback_group_hip_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_shoulder_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_roll_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_pitch_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_claw_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        callback_group_elbow_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 

        timer_hip_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setHipRPM, this), callback_group_hip_);
            
        timer_shoulder_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setShoulderRPM, this), callback_group_shoulder_);

        timer_elbow_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setElbowRPM, this), callback_group_elbow_);
            
        timer_roll_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setRollRPM, this), callback_group_roll_);
            
        timer_pitch_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setPitchRPM, this), callback_group_pitch_);
            
        timer_claw_ = this->create_wall_timer(
            20ms, bind(&ArmNode::setClawRPM, this), callback_group_claw_);
        
        y_left_axis_subscriber = create_subscription<std_msgs::msg::Float32>(
            "/y_left_axis", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                this->desired_rpms = msg->data;
            });

        max_rpm_subscriber = create_subscription<std_msgs::msg::Float32>(
            "/telemetryJSON/arm_max_rpm", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                this->logged_rpm = msg->data;
            });

        hip_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/arm_hip/telemetry", 10);
        shoulder_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/arm_shoulder/telemetry", 10);
        elbow_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/arm_elbow/telemetry", 10);
        roll_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/arm_roll/telemetry", 10);
        pitch_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/arm_pitch/telemetry", 10);
        claw_telemetry_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/arm_claw/telemetry", 10);

        // setup telemetry timer
        callback_group_telemetry_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        telemetry_timer_ = this->create_wall_timer(
            50ms, bind(&ArmNode::telemetry, this), callback_group_telemetry_);
    }

    ~ArmNode()
    {
        hipMotor.disconnect();
        shoulderMotor.disconnect();
        elbowMotor.disconnect();
        rollMotor.disconnect();
        pitchMotor.disconnect();
        clawMotor.disconnect();
    }

    void setHipTarget(int32_t rpm)      { target_hip_rpm.store(rpm); }
    void setShoulderTarget(int32_t rpm) { target_shoulder_rpm.store(rpm); }
    void setElbowTarget(int32_t rpm)    { target_elbow_rpm.store(rpm); }
    void setRollTarget(int32_t rpm)     { target_roll_rpm.store(rpm); }
    void setPitchTarget(int32_t rpm)    { target_pitch_rpm.store(rpm); }
    void setClawTarget(int32_t rpm)     { target_claw_rpm.store(rpm); }

private:
    void setShoulderRPM(){
        shoulderMotor.set_rpm(target_shoulder_rpm); // will add button logic later
    }

    void setElbowRPM() {
        elbowMotor.set_rpm(target_elbow_rpm);
    }

    void setRollRPM() {
        rollMotor.set_rpm(target_roll_rpm);
    }

    void setPitchRPM() {
        pitchMotor.set_rpm(target_pitch_rpm);
    }

    void setClawRPM() {
        clawMotor.set_rpm(target_claw_rpm);
    }

    void setHipRPM() {
        hipMotor.set_rpm(target_hip_rpm);
    }

    void telemetry(){
        VESCData m_telemetry;

        if(hipMotor.get_telemetry(m_telemetry)){
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(1.0f),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            hip_telemetry_pub->publish(msg);
        }

        if(shoulderMotor.get_telemetry(m_telemetry)){
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(1.0f),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            shoulder_telemetry_pub->publish(msg);
        }

        if(elbowMotor.get_telemetry(m_telemetry)){
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(m_telemetry.motor_controller_id),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            elbow_telemetry_pub->publish(msg);
        }

        if(rollMotor.get_telemetry(m_telemetry)){
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(m_telemetry.motor_controller_id),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            roll_telemetry_pub->publish(msg);
        }

        if(pitchMotor.get_telemetry(m_telemetry)){
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(m_telemetry.motor_controller_id),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            pitch_telemetry_pub->publish(msg);
        }

        if(clawMotor.get_telemetry(m_telemetry)){
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {
                static_cast<float>(m_telemetry.rpm),
                static_cast<float>(m_telemetry.motor_controller_id),
                m_telemetry.input_voltage,
                m_telemetry.current_motor,
            };
            claw_telemetry_pub->publish(msg);
        }

    }

    VESC hipMotor;  //Cadera
    VESC shoulderMotor; // hombro
    VESC elbowMotor; //COdo
    VESC rollMotor;
    VESC pitchMotor;
    VESC clawMotor;

    rclcpp::CallbackGroup::SharedPtr callback_group_hip_;
    rclcpp::CallbackGroup::SharedPtr callback_group_shoulder_;
    rclcpp::CallbackGroup::SharedPtr callback_group_elbow_;
    rclcpp::CallbackGroup::SharedPtr callback_group_roll_;
    rclcpp::CallbackGroup::SharedPtr callback_group_pitch_;
    rclcpp::CallbackGroup::SharedPtr callback_group_claw_;

    rclcpp::TimerBase::SharedPtr timer_hip_;
    rclcpp::TimerBase::SharedPtr timer_shoulder_;
    rclcpp::TimerBase::SharedPtr timer_elbow_;
    rclcpp::TimerBase::SharedPtr timer_roll_;
    rclcpp::TimerBase::SharedPtr timer_pitch_;
    rclcpp::TimerBase::SharedPtr timer_claw_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hip_telemetry_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr shoulder_telemetry_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr elbow_telemetry_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr roll_telemetry_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pitch_telemetry_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr claw_telemetry_pub;


    std::atomic<int32_t> target_hip_rpm{0};
    std::atomic<int32_t> target_shoulder_rpm{0};
    std::atomic<int32_t> target_elbow_rpm{0};
    std::atomic<int32_t> target_roll_rpm{0};
    std::atomic<int32_t> target_pitch_rpm{0};
    std::atomic<int32_t> target_claw_rpm{0};


    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_telemetry_;

        /* Publishers and subscribers */

    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_A_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_left_axis_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr max_rpm_subscriber;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vesc_full_telemetry_pub;
    /* A button test*/
    float desired_rpms = 0.0f;
    float logged_rpm = 0.0f;

    // ---- Parameters ----
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    rclcpp::shutdown();
    return 0;
}