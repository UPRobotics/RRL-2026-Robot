#include "kinematics_pkg/forward_kinematics.hpp"
#include <cmath>
#include <iostream>

namespace kinematics {

ForwardKinematics::ForwardKinematics() {
  // Initialize DH parameters for RRL-2026 arm (6 joints total)
  // Based on ensamble_flaca.xacro URDF measurements
  // Units: meters for a, d; radians for alpha, theta, roll, pitch, yaw

  dh_params_.resize(6);

  // Joint 1: Cadera (Hip) - Revolute, Z-axis rotation
  dh_params_[0].a = 0.0;
  dh_params_[0].alpha = M_PI / 2;
  dh_params_[0].d = 0.0;
  dh_params_[0].theta = 0.0;

  // Joint 2: Hombro (Shoulder) - Revolute, Y-axis rotation
  dh_params_[1].a = 0.249;
  dh_params_[1].alpha = 0.0;
  dh_params_[1].d = 0.0;
  dh_params_[1].theta = 0.0;

  // Joint 3: Codo (Elbow) - Revolute, Y-axis rotation
  dh_params_[2].a = 0.254;
  dh_params_[2].alpha = M_PI;
  dh_params_[2].d = 0.0;
  dh_params_[2].theta = 0.0;

  // Joint 4: Roll - Revolute, X-axis rotation (diagonal axis)
  dh_params_[3].a = 0.0;
  dh_params_[3].alpha = M_PI / 2;
  dh_params_[3].d = 0.0;
  dh_params_[3].theta = 0.0;

  // Joint 5: Pitch - Revolute, diagonal rotation
  dh_params_[4].a = 0.0;
  dh_params_[4].alpha = 0.0;
  dh_params_[4].d = 0.0;
  dh_params_[4].theta = 0.0;

  // Joint 6: roll_to_claw (Fixed joint - transforms from Componente5 to claw_tip)
  // From URDF: <origin xyz="0.225 0.01 -0.225" rpy="0 .80 0"/>
  dh_params_[5].a = 0.0;
  dh_params_[5].alpha = 0.0;
  dh_params_[5].d = 0.0;
  dh_params_[5].theta = 0.0;  // Fixed joint, no rotation
  dh_params_[5].x_offset = 0.225;   // +22.5 cm toward tip
  dh_params_[5].y_offset = 0.01;    // +1 cm lateral offset
  dh_params_[5].z_offset = -0.225;  // -22.5 cm down
  dh_params_[5].pitch = 0.80;       // Pitch rotation (0.80 rad ≈ 45.8°)
}

Eigen::Matrix4d ForwardKinematics::computeDHTMatrix(
    double theta, double d, double a, double alpha) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // DH Transformation: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
  double c_theta = cos(theta);
  double s_theta = sin(theta);
  double c_alpha = cos(alpha);
  double s_alpha = sin(alpha);

  // T = [cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)  a*cos(θ)]
  //     [sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)  a*sin(θ)]
  //     [0        sin(α)         cos(α)        d       ]
  //     [0        0              0             1       ]

  T(0, 0) = c_theta;
  T(0, 1) = -s_theta * c_alpha;
  T(0, 2) = s_theta * s_alpha;
  T(0, 3) = a * c_theta;

  T(1, 0) = s_theta;
  T(1, 1) = c_theta * c_alpha;
  T(1, 2) = -c_theta * s_alpha;
  T(1, 3) = a * s_theta;

  T(2, 0) = 0.0;
  T(2, 1) = s_alpha;
  T(2, 2) = c_alpha;
  T(2, 3) = d;

  T(3, 0) = 0.0;
  T(3, 1) = 0.0;
  T(3, 2) = 0.0;
  T(3, 3) = 1.0;

  return T;
}

Eigen::Matrix4d ForwardKinematics::computeFullTransform(
    const DHParameters& dh) {
  // Compute standard DH matrix
  Eigen::Matrix4d T = computeDHTMatrix(dh.theta, dh.d, dh.a, dh.alpha);

  // Apply additional rotation (roll, pitch, yaw) for non-standard joints
  if (dh.roll != 0 || dh.pitch != 0 || dh.yaw != 0) {
    Eigen::AngleAxisd rollAngle(dh.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(dh.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(dh.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d R = q.toRotationMatrix();

    // Apply rotation to the transformation matrix
    T.block<3, 3>(0, 0) = T.block<3, 3>(0, 0) * R;
  }

  // Apply position offset (x_offset, y_offset, z_offset)
  if (dh.x_offset != 0 || dh.y_offset != 0 || dh.z_offset != 0) {
    Eigen::Vector3d offset(dh.x_offset, dh.y_offset, dh.z_offset);
    // Apply offset in the current frame
    T.block<3, 1>(0, 3) += T.block<3, 3>(0, 0) * offset;
  }

  return T;
}

bool ForwardKinematics::getTransformationMatrix(
    const std::vector<double>& joint_angles,
    Eigen::Matrix4d& T) {

  // For 6 joints: first 5 are movable (from joint_angles), 6th is fixed (roll_to_claw)
  if (joint_angles.size() != 5) {
    std::cerr << "Error: Expected 5 joint angles, got " << joint_angles.size() << std::endl;
    return false;
  }

  T = Eigen::Matrix4d::Identity();

  // Compute transformation through all 6 joints
  // Joints 0-4: Movable joints (Cadera, Hombro, Codo, Roll, Pitch)
  // Joint 5: Fixed offset to claw_tip (roll_to_claw)

  for (size_t i = 0; i < 5; ++i) {
    double theta = joint_angles[i] + dh_params_[i].theta;
    double d = dh_params_[i].d;
    double a = dh_params_[i].a;
    double alpha = dh_params_[i].alpha;

    Eigen::Matrix4d Ti = computeDHTMatrix(theta, d, a, alpha);
    T = T * Ti;
  }

  // Apply the 6th joint (fixed offset from Componente5 to claw_tip)
  Eigen::Matrix4d T_claw = computeFullTransform(dh_params_[5]);
  T = T * T_claw;

  return true;
}

std::vector<Eigen::Matrix4d> ForwardKinematics::getDHMatrices(
    const std::vector<double>& joint_angles) {

  std::vector<Eigen::Matrix4d> matrices;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // Process 5 movable joints
  for (size_t i = 0; i < 5 && i < joint_angles.size(); ++i) {
    double theta = joint_angles[i] + dh_params_[i].theta;
    double d = dh_params_[i].d;
    double a = dh_params_[i].a;
    double alpha = dh_params_[i].alpha;

    Eigen::Matrix4d Ti = computeDHTMatrix(theta, d, a, alpha);
    T = T * Ti;
    matrices.push_back(T);
  }

  // Apply the 6th joint (fixed offset to claw_tip)
  Eigen::Matrix4d T_claw = computeFullTransform(dh_params_[5]);
  T = T * T_claw;
  matrices.push_back(T);

  return matrices;
}

void ForwardKinematics::matrixToPose(
    const Eigen::Matrix4d& T,
    geometry_msgs::msg::Pose& pose) {

  // Extract position
  pose.position.x = T(0, 3);
  pose.position.y = T(1, 3);
  pose.position.z = T(2, 3);

  // Extract rotation matrix and convert to quaternion
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Quaterniond q(R);
  q.normalize();

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
}

bool ForwardKinematics::computeFK(
    const std::vector<double>& joint_angles,
    geometry_msgs::msg::Pose& end_effector_pose) {

  Eigen::Matrix4d T;
  if (!getTransformationMatrix(joint_angles, T)) {
    return false;
  }

  matrixToPose(T, end_effector_pose);
  return true;
}

} // namespace kinematics
