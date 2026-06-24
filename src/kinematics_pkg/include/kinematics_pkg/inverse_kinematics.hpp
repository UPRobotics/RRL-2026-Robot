#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include "forward_kinematics.hpp"

namespace kinematics {

class InverseKinematics {
public:
  InverseKinematics(std::shared_ptr<ForwardKinematics> fk);
  ~InverseKinematics() = default;

  struct IKConfig {
    int max_iterations = 100;
    double tolerance = 0.01;        // 1cm tolerance
    double step_size = 0.01;        // Learning rate for gradient descent
    std::vector<double> joint_limits_min = {-3.14, -0.52, -3.23, -3.05, -1.66};
    std::vector<double> joint_limits_max = {3.05, 1.40, 0.35, 3.05, 1.48};
  };

  // Compute inverse kinematics using numerical method (Jacobian-based)
  // desired_pose: Target end-effector pose (x, y, z, orientation)
  // joint_angles: Output solution (initial guess and result)
  // Returns: true if converged, false if failed or singular
  bool computeIK(
    const geometry_msgs::msg::Pose& desired_pose,
    std::vector<double>& joint_angles);

  // Compute IK with custom configuration
  bool computeIKWithConfig(
    const geometry_msgs::msg::Pose& desired_pose,
    std::vector<double>& joint_angles,
    const IKConfig& config);

  // Compute numerical Jacobian (5x3 or 5x6 depending on DOF)
  Eigen::MatrixXd computeJacobian(
    const std::vector<double>& joint_angles);

  // Check if solution is within joint limits
  bool isWithinJointLimits(const std::vector<double>& joint_angles) const;

  // Clamp joint angles to limits
  void clampToJointLimits(std::vector<double>& joint_angles) const;

  // Get configuration
  const IKConfig& getConfig() const { return config_; }
  void setConfig(const IKConfig& config) { config_ = config; }

private:
  std::shared_ptr<ForwardKinematics> fk_;
  IKConfig config_;

  // Position error between desired and current
  Eigen::Vector3d computePositionError(
    const geometry_msgs::msg::Pose& desired,
    const geometry_msgs::msg::Pose& current);

  // Pseudo-inverse using SVD
  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J);

  // Convert quaternion to Euler angles (for orientation control if needed)
  Eigen::Vector3d quaternionToEuler(
    double qx, double qy, double qz, double qw);

  // Convert Euler angles to quaternion
  void eulerToQuaternion(
    double roll, double pitch, double yaw,
    double& qx, double& qy, double& qz, double& qw);

  // Numerical differentiation for Jacobian
  double DELTA = 0.001;  // Small delta for numerical derivative
};

} // namespace kinematics

#endif // INVERSE_KINEMATICS_HPP
