#include "kinematics_pkg/inverse_kinematics.hpp"
#include <cmath>
#include <iostream>
#include <Eigen/SVD>

namespace kinematics {

InverseKinematics::InverseKinematics(std::shared_ptr<ForwardKinematics> fk)
    : fk_(fk) {}

Eigen::Vector3d InverseKinematics::computePositionError(
    const geometry_msgs::msg::Pose& desired,
    const geometry_msgs::msg::Pose& current) {

  Eigen::Vector3d error;
  error(0) = desired.position.x - current.position.x;
  error(1) = desired.position.y - current.position.y;
  error(2) = desired.position.z - current.position.z;
  return error;
}

Eigen::MatrixXd InverseKinematics::pseudoInverse(const Eigen::MatrixXd& J) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

  return svd.matrixV() *
         svd.singularValues().cwiseInverse().asDiagonal() *
         svd.matrixU().transpose();
}

Eigen::MatrixXd InverseKinematics::computeJacobian(
    const std::vector<double>& joint_angles) {

  // Jacobian is 3 x n_joints (position only, not orientation)
  int n_joints = joint_angles.size();
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, n_joints);

  std::vector<double> q_plus = joint_angles;
  std::vector<double> q_minus = joint_angles;

  for (int i = 0; i < n_joints; ++i) {
    q_plus[i] = joint_angles[i] + DELTA;
    q_minus[i] = joint_angles[i] - DELTA;

    geometry_msgs::msg::Pose pose_plus, pose_minus;
    fk_->computeFK(q_plus, pose_plus);
    fk_->computeFK(q_minus, pose_minus);

    J(0, i) = (pose_plus.position.x - pose_minus.position.x) / (2.0 * DELTA);
    J(1, i) = (pose_plus.position.y - pose_minus.position.y) / (2.0 * DELTA);
    J(2, i) = (pose_plus.position.z - pose_minus.position.z) / (2.0 * DELTA);

    q_plus[i] = joint_angles[i];
    q_minus[i] = joint_angles[i];
  }

  return J;
}

bool InverseKinematics::isWithinJointLimits(
    const std::vector<double>& joint_angles) const {

  if (joint_angles.size() != config_.joint_limits_min.size()) {
    return false;
  }

  for (size_t i = 0; i < joint_angles.size(); ++i) {
    if (joint_angles[i] < config_.joint_limits_min[i] ||
        joint_angles[i] > config_.joint_limits_max[i]) {
      return false;
    }
  }
  return true;
}

void InverseKinematics::clampToJointLimits(
    std::vector<double>& joint_angles) const {

  for (size_t i = 0; i < joint_angles.size(); ++i) {
    if (i < config_.joint_limits_min.size()) {
      joint_angles[i] = std::max(config_.joint_limits_min[i],
                                std::min(config_.joint_limits_max[i], joint_angles[i]));
    }
  }
}

Eigen::Vector3d InverseKinematics::quaternionToEuler(
    double qx, double qy, double qz, double qw) {

  Eigen::Vector3d euler;

  // Roll (x-axis rotation)
  double sinr_cosp = 2 * (qw * qx + qy * qz);
  double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  euler(0) = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1)
    euler(1) = std::copysign(M_PI / 2, sinp);
  else
    euler(1) = std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  euler(2) = std::atan2(siny_cosp, cosy_cosp);

  return euler;
}

void InverseKinematics::eulerToQuaternion(
    double roll, double pitch, double yaw,
    double& qx, double& qy, double& qz, double& qw) {

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

bool InverseKinematics::computeIK(
    const geometry_msgs::msg::Pose& desired_pose,
    std::vector<double>& joint_angles) {

  return computeIKWithConfig(desired_pose, joint_angles, config_);
}

bool InverseKinematics::computeIKWithConfig(
    const geometry_msgs::msg::Pose& desired_pose,
    std::vector<double>& joint_angles,
    const IKConfig& config) {

  // Initialize if empty
  if (joint_angles.empty()) {
    joint_angles = std::vector<double>(5, 0.0);
  }

  clampToJointLimits(joint_angles);

  for (int iter = 0; iter < config.max_iterations; ++iter) {
    // Compute current pose
    geometry_msgs::msg::Pose current_pose;
    if (!fk_->computeFK(joint_angles, current_pose)) {
      return false;
    }

    // Compute position error
    Eigen::Vector3d error = computePositionError(desired_pose, current_pose);
    double error_norm = error.norm();

    if (error_norm < config.tolerance) {
      clampToJointLimits(joint_angles);
      return true;
    }

    // Compute Jacobian
    Eigen::MatrixXd J = computeJacobian(joint_angles);

    // Compute pseudo-inverse
    Eigen::MatrixXd J_pinv = pseudoInverse(J);

    // Update joint angles: q = q + alpha * J_pinv * error
    Eigen::VectorXd dq = J_pinv * error * config.step_size;

    for (size_t i = 0; i < joint_angles.size(); ++i) {
      joint_angles[i] += dq(i);
    }

    clampToJointLimits(joint_angles);
  }

  // Converged to tolerance or max iterations reached
  geometry_msgs::msg::Pose current_pose;
  if (!fk_->computeFK(joint_angles, current_pose)) {
    return false;
  }

  Eigen::Vector3d error = computePositionError(desired_pose, current_pose);
  return error.norm() < (config.tolerance * 2.0);
}

} // namespace kinematics
