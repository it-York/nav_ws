#include <gtest/gtest.h>
#include "go2w_nav2_navigation/odom_kinematics.hpp"

using go2w_nav2_navigation::baseTwist;
using go2w_nav2_navigation::shiftPoseCovariance;

tf2::Transform pose(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, 0));
}

TEST(OdomKinematics, TranslationIsInCurrentBaseFrame)
{
  const auto velocity = baseTwist(pose(0, 0, M_PI / 2), pose(0, 0.05, M_PI / 2), 0.1);
  EXPECT_NEAR(velocity.linear.x, 0.5, 1e-9);
  EXPECT_NEAR(velocity.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(velocity.angular.z, 0.0, 1e-9);
}

TEST(OdomKinematics, RotationAcrossPiHasNoWrapSpike)
{
  const auto velocity = baseTwist(pose(0, 0, M_PI - 0.02), pose(0, 0, -M_PI + 0.03), 0.1);
  EXPECT_NEAR(velocity.angular.z, 0.5, 1e-9);
}

TEST(OdomKinematics, QuaternionSignDoesNotChangeVelocity)
{
  auto previous = pose(0, 0, 0.2);
  auto current = previous;
  current.setRotation(current.getRotation() * -1.0);
  const auto velocity = baseTwist(previous, current, 0.1);
  EXPECT_NEAR(velocity.angular.z, 0.0, 1e-9);
}

TEST(OdomKinematics, MountedImuDoesNotInventBaseTranslationWhenTurning)
{
  tf2::Quaternion mounting;
  mounting.setRPY(0.0, M_PI / 6, 0.0);
  tf2::Transform base_T_imu(mounting, tf2::Vector3(0.278256279, 0.02329, 0.112620959));
  const auto old_imu = pose(0, 0, 0) * base_T_imu;
  const auto new_imu = pose(0, 0, 0.07) * base_T_imu;
  const auto velocity = baseTwist(old_imu * base_T_imu.inverse(), new_imu * base_T_imu.inverse(), 0.1);
  EXPECT_NEAR(velocity.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(velocity.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(velocity.angular.z, 0.7, 1e-9);
}

TEST(OdomKinematics, LeverArmPropagatesOrientationUncertainty)
{
  std::array<double, 36> covariance{};
  covariance[35] = 0.04;
  const auto shifted = shiftPoseCovariance(covariance, tf2::Vector3(-0.3, 0, 0));
  EXPECT_NEAR(shifted[7], 0.0036, 1e-12);
  EXPECT_NEAR(shifted[11], -0.012, 1e-12);
  EXPECT_NEAR(shifted[31], shifted[11], 1e-12);
  EXPECT_NEAR(shifted[35], 0.04, 1e-12);
}
