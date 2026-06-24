#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace kinematics {

class ForwardKinematics {
public:
  ForwardKinematics();
  ~ForwardKinematics() = default;

  struct DHParameters {
    double a;        // Link length (m)
    double alpha;    // Link twist - rotation around X (rad)
    double d;        // Link offset (m)
    double theta;    // Joint angle (rad) - this changes

    // Additional parameters for non-standard joints (like roll_to_claw)
    double roll;     // Rotation around X (rad) - complementary to alpha
    double pitch;    // Rotation around Y (rad)
    double yaw;      // Rotation around Z (rad)
    double x_offset; // Position offset in X (m)
    double y_offset; // Position offset in Y (m)
    double z_offset; // Position offset in Z (m)

    // Constructor with defaults
    DHParameters()
      : a(0), alpha(0), d(0), theta(0),
        roll(0), pitch(0), yaw(0),
        x_offset(0), y_offset(0), z_offset(0) {}
  };

  // Compute forward kinematics given joint angles
  // joint_angles: [cadera, hombro, codo, roll, pitch] (in radians)
  // Returns: Pose at claw_tip (end effector from URDF joint roll_to_claw)
  // Note: The claw offset is defined in URDF, not hardcoded
  bool computeFK(
    const std::vector<double>& joint_angles,
    geometry_msgs::msg::Pose& end_effector_pose);

  // Get the full transformation matrix T (4x4)
  bool getTransformationMatrix(
    const std::vector<double>& joint_angles,
    Eigen::Matrix4d& T);

  // Get individual DH-A matrices for each joint
  std::vector<Eigen::Matrix4d> getDHMatrices(
    const std::vector<double>& joint_angles);

  // Getters
  const std::vector<DHParameters>& getDHParams() const { return dh_params_; }

private:
  std::vector<DHParameters> dh_params_;

  // Compute DH transformation matrix for a single joint
  // T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
  Eigen::Matrix4d computeDHTMatrix(
    double theta, double d, double a, double alpha);

  // Compute full transformation including additional rotations (rpy) and offsets
  Eigen::Matrix4d computeFullTransform(const DHParameters& dh);

  // Convert 4x4 transformation matrix to geometry_msgs Pose
  void matrixToPose(
    const Eigen::Matrix4d& T,
    geometry_msgs::msg::Pose& pose);
};

} // namespace kinematics

#endif // FORWARD_KINEMATICS_HPP
