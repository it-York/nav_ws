#pragma once

#include <array>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Transform.h>

namespace go2w_nav2_navigation
{
// Finite-difference velocity of the BASE origin, expressed in the current
// base frame. Transform the poses before differentiating so mounting lever
// arm motion is not mistaken for translation of the robot centre.
inline geometry_msgs::msg::Twist baseTwist(
  const tf2::Transform & previous, const tf2::Transform & current, double dt)
{
  geometry_msgs::msg::Twist result;
  const auto velocity = current.getBasis().transpose() *
    ((current.getOrigin() - previous.getOrigin()) / dt);
  auto rotation = previous.getRotation().inverse() * current.getRotation();
  rotation.normalize();
  if (rotation.w() < 0.0) rotation *= -1.0;
  const tf2::Vector3 imaginary(rotation.x(), rotation.y(), rotation.z());
  const double length = imaginary.length();
  const tf2::Vector3 omega_previous = length > 1e-10 ?
    imaginary * (2.0 * std::atan2(length, rotation.w()) / (length * dt)) :
    imaginary * (2.0 / dt);
  const auto omega = current.getBasis().transpose() * previous.getBasis() * omega_previous;
  result.linear.x = velocity.x();
  result.linear.y = velocity.y();
  result.linear.z = velocity.z();
  result.angular.x = omega.x();
  result.angular.y = omega.y();
  result.angular.z = omega.z();
  return result;
}

// Position/orientation covariance in header-frame fixed axes. Moving the
// reference point from IMU to base couples orientation error into position.
inline std::array<double, 36> shiftPoseCovariance(
  const std::array<double, 36> & covariance, const tf2::Vector3 & world_offset)
{
  double jacobian[6][6]{};
  for (int i = 0; i < 6; ++i) jacobian[i][i] = 1.0;
  jacobian[0][4] = world_offset.z();
  jacobian[0][5] = -world_offset.y();
  jacobian[1][3] = -world_offset.z();
  jacobian[1][5] = world_offset.x();
  jacobian[2][3] = world_offset.y();
  jacobian[2][4] = -world_offset.x();
  std::array<double, 36> output{};
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      for (int k = 0; k < 6; ++k)
        for (int l = 0; l < 6; ++l)
          output[i * 6 + j] += jacobian[i][k] * covariance[k * 6 + l] * jacobian[j][l];
  return output;
}
}  // namespace go2w_nav2_navigation
