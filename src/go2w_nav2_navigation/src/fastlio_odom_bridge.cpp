#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float32.hpp>
#include "go2w_nav2_navigation/odom_kinematics.hpp"

namespace go2w_nav2_navigation
{
class FastlioOdomBridge : public rclcpp::Node
{
public:
  FastlioOdomBridge() : Node("fastlio_odom_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    max_input_age_ = declare_parameter<double>("max_input_age", 0.5);
    max_twist_dt_ = declare_parameter<double>("max_twist_dt", 0.5);
    linear_variance_ = declare_parameter<double>("linear_velocity_variance", 0.04);
    angular_variance_ = declare_parameter<double>("angular_velocity_variance", 0.09);
    if (!std::isfinite(max_input_age_) || max_input_age_ <= 0.0 ||
        !std::isfinite(max_twist_dt_) || max_twist_dt_ <= 0.0 ||
        !std::isfinite(linear_variance_) || linear_variance_ <= 0.0 ||
        !std::isfinite(angular_variance_) || angular_variance_ <= 0.0)
      throw std::invalid_argument("Odometry age limits and velocity variances must be positive");
    // Koala No. 2 base_link -> MID360S IMU.  This is derived from the
    // mechanical base_link -> lidar_link drawing and FAST-LIO's calibrated
    // IMU -> LiDAR translation in config/mid360.yaml.
    const auto xyz = declare_parameter<std::vector<double>>(
        "base_T_imu_xyz", {0.278256279, 0.023290, 0.112620959});
    const auto rpy_deg = declare_parameter<std::vector<double>>(
        "base_T_imu_rpy_deg", {0.0, 30.0, 0.0});

    if (xyz.size() != 3 || rpy_deg.size() != 3)
      throw std::runtime_error("base_T_imu_xyz and base_T_imu_rpy_deg must contain 3 values");

    constexpr double deg_to_rad = M_PI / 180.0;
    tf2::Quaternion rotation;
    rotation.setRPY(
        rpy_deg[0] * deg_to_rad,
        rpy_deg[1] * deg_to_rad,
        rpy_deg[2] * deg_to_rad);
    rotation.normalize();
    base_T_imu_.setOrigin(tf2::Vector3(xyz[0], xyz[1], xyz[2]));
    base_T_imu_.setRotation(rotation);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 5);
    age_pub_ = create_publisher<std_msgs::msg::Float32>("~/input_age_ms", 1);
    duration_pub_ = create_publisher<std_msgs::msg::Float32>("~/processing_ms", 1);
    if (publish_tf_)
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&FastlioOdomBridge::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(), "Bridging %s to %s with frames %s -> %s",
        input_topic_.c_str(), output_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto started = std::chrono::steady_clock::now();
    const int64_t stamp = rclcpp::Time(message->header.stamp).nanoseconds();
    std_msgs::msg::Float32 age;
    age.data = static_cast<float>((now().nanoseconds() - stamp) * 1e-6);
    age_pub_->publish(age);
    const auto & position = message->pose.pose.position;
    const auto & q = message->pose.pose.orientation;
    const double qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (message->header.frame_id != "camera_init" || message->child_frame_id != "body" ||
        stamp <= previous_stamp_ || age.data < -50.0 || age.data > max_input_age_ * 1000.0 ||
        !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !std::isfinite(qnorm) || std::abs(qnorm - 1.0) > 0.01 ||
        !std::all_of(message->pose.covariance.begin(), message->pose.covariance.end(),
          [](double value) { return std::isfinite(value); })) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Rejecting stale, out-of-order or invalid FAST-LIO odometry (age %.1f ms)", age.data);
      return;
    }
    tf2::Transform odom_T_imu;
    tf2::fromMsg(message->pose.pose, odom_T_imu);
    odom_T_imu.setRotation(odom_T_imu.getRotation().normalized());
    const tf2::Transform odom_T_base = odom_T_imu * base_T_imu_.inverse();

    nav_msgs::msg::Odometry output = *message;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = odom_T_base.getOrigin().x();
    output.pose.pose.position.y = odom_T_base.getOrigin().y();
    output.pose.pose.position.z = odom_T_base.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(odom_T_base.getRotation());
    output.pose.covariance = shiftPoseCovariance(message->pose.covariance,
      odom_T_base.getOrigin() - odom_T_imu.getOrigin());
    output.twist.twist = geometry_msgs::msg::Twist();
    output.twist.covariance.fill(0.0);
    const double dt = (stamp - previous_stamp_) * 1e-9;
    const bool have_velocity = previous_stamp_ > 0 && dt >= 1e-4 && dt <= max_twist_dt_;
    if (have_velocity)
      output.twist.twist = baseTwist(previous_pose_, odom_T_base, dt);
    // These configurable variances are conservative defaults, not a claim
    // of calibrated uncertainty. A first sample or gap has unknown velocity.
    for (int i = 0; i < 6; ++i)
      output.twist.covariance[i * 6 + i] = have_velocity ?
        (i < 3 ? linear_variance_ : angular_variance_) : 1e6;
    previous_pose_ = odom_T_base;
    previous_stamp_ = stamp;
    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = base_frame_;
      transform.transform = tf2::toMsg(odom_T_base);
      tf_broadcaster_->sendTransform(transform);
    }
    odom_pub_->publish(output);
    std_msgs::msg::Float32 duration;
    duration.data = static_cast<float>(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count());
    duration_pub_->publish(duration);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  double max_input_age_, max_twist_dt_, linear_variance_, angular_variance_;
  int64_t previous_stamp_{0};
  tf2::Transform previous_pose_{tf2::Transform::getIdentity()};
  tf2::Transform base_T_imu_{tf2::Transform::getIdentity()};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr age_pub_, duration_pub_;
};
}  // namespace go2w_nav2_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2w_nav2_navigation::FastlioOdomBridge>());
  rclcpp::shutdown();
  return 0;
}
