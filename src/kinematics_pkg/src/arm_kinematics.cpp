#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>

#include "kinematics_pkg/forward_kinematics.hpp"
#include "kinematics_pkg/inverse_kinematics.hpp"

using namespace std::chrono_literals;

class ArmKinematicsNode : public rclcpp::Node {
public:
  ArmKinematicsNode() : rclcpp::Node("arm_kinematics") {
    // Declare parameters
    this->declare_parameter<std::string>("arm_base_frame", "base_link");
    this->declare_parameter<std::string>("arm_end_effector_frame", "claw_tip");

    // Get parameters
    base_frame_ = this->get_parameter("arm_base_frame").as_string();
    end_effector_frame_ = this->get_parameter("arm_end_effector_frame").as_string();

    // Initialize kinematics solvers
    fk_ = std::make_shared<kinematics::ForwardKinematics>();
    ik_ = std::make_shared<kinematics::InverseKinematics>(fk_);

    // TF2 setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/arm/end_effector_pose", 10);

    ik_solution_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/arm/inverse_kinematics_solution", 10);

    // Subscribers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          this->onJointStates(msg);
        });

    // Timer for continuous FK computation
    timer_ = this->create_wall_timer(
        50ms, [this]() { this->computeForwardKinematics(); });

    RCLCPP_INFO(this->get_logger(),
                "Arm Kinematics node initialized (base_frame: %s, end_effector: %s)",
                base_frame_.c_str(), end_effector_frame_.c_str());
  }

private:
  // Kinematics solvers
  std::shared_ptr<kinematics::ForwardKinematics> fk_;
  std::shared_ptr<kinematics::InverseKinematics> ik_;

  // ROS2 components
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ik_solution_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Frame parameters
  std::string base_frame_;
  std::string end_effector_frame_;

  // Storage for current joint angles
  std::vector<double> current_joint_angles_{0.0, 0.0, 0.0, 0.0, 0.0};
  std::mutex angles_mutex_;

  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(angles_mutex_);

    // Extract arm joint angles (Cadera, Hombro, Codo, Roll, Pitch)
    // Ignore Grip and body joints
    std::vector<double> angles(5, 0.0);

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "Cadera") angles[0] = msg->position[i];
      else if (msg->name[i] == "Hombro") angles[1] = msg->position[i];
      else if (msg->name[i] == "Codo") angles[2] = msg->position[i];
      else if (msg->name[i] == "Roll") angles[3] = msg->position[i];
      else if (msg->name[i] == "Pitch") angles[4] = msg->position[i];
    }

    current_joint_angles_ = angles;
  }

  void computeForwardKinematics() {
    std::lock_guard<std::mutex> lock(angles_mutex_);

    // Compute FK
    geometry_msgs::msg::Pose pose;
    if (!fk_->computeFK(current_joint_angles_, pose)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "FK computation failed");
      return;
    }

    // Publish result
    auto pose_stamped = std::make_unique<geometry_msgs::msg::PoseStamped>();
    pose_stamped->header.stamp = this->get_clock()->now();
    pose_stamped->header.frame_id = base_frame_;
    pose_stamped->pose = pose;

    pose_pub_->publish(std::move(pose_stamped));
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
