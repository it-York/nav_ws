#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace go2w_nav2_navigation
{
class CloudToScan : public rclcpp::Node
{
public:
  CloudToScan() : Node("cloud_to_scan")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_2d");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    min_height_ = declare_parameter<double>("min_height", 0.08);
    max_height_ = declare_parameter<double>("max_height", 0.80);
    range_min_ = declare_parameter<double>("range_min", 0.12);
    range_max_ = declare_parameter<double>("range_max", 15.0);
    self_filter_enabled_ = declare_parameter<bool>("self_filter_enabled", true);
    self_min_x_ = declare_parameter<double>("self_min_x", -0.50);
    self_max_x_ = declare_parameter<double>("self_max_x", 0.55);
    self_min_y_ = declare_parameter<double>("self_min_y", -0.32);
    self_max_y_ = declare_parameter<double>("self_max_y", 0.32);
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = declare_parameter<double>("angle_increment", M_PI / 720.0);
    use_inf_ = declare_parameter<bool>("use_inf", true);
    const auto xyz = declare_parameter<std::vector<double>>(
        "base_T_cloud_xyz", {0.278256279, 0.023290, 0.112620959});
    const auto rpy = declare_parameter<std::vector<double>>(
        "base_T_cloud_rpy_deg", {0.0, 30.0, 0.0});

    if (xyz.size() != 3 || rpy.size() != 3 || min_height_ >= max_height_ ||
        range_min_ < 0.0 || range_min_ >= range_max_ ||
        self_min_x_ >= self_max_x_ || self_min_y_ >= self_max_y_ ||
        angle_min_ >= angle_max_ ||
        angle_increment_ <= 0.0)
      throw std::invalid_argument("invalid cloud_to_scan geometry parameters");

    tf2::Quaternion q;
    constexpr double deg = M_PI / 180.0;
    q.setRPY(rpy[0] * deg, rpy[1] * deg, rpy[2] * deg);
    q.normalize();
    base_T_cloud_.setOrigin(tf2::Vector3(xyz[0], xyz[1], xyz[2]));
    base_T_cloud_.setRotation(q);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, rclcpp::SensorDataQoS());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CloudToScan::cloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Projecting %s to %s in %s", input_topic_.c_str(),
        output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header = cloud->header;
    scan.header.frame_id = target_frame_;
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.scan_time = 0.1;
    const auto count = static_cast<std::size_t>(
        std::ceil((angle_max_ - angle_min_) / angle_increment_));
    scan.ranges.assign(count, use_inf_ ? std::numeric_limits<float>::infinity() : range_max_ + 1.0f);

    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z)
      {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
          continue;
        const tf2::Vector3 point = base_T_cloud_ * tf2::Vector3(*x, *y, *z);
        if (point.z() < min_height_ || point.z() > max_height_)
          continue;
        // Remove returns from the Go2W body, legs and wheels using the real
        // base footprint rather than a large circular blind zone.  Obstacles
        // close to the corners remain visible.
        if (self_filter_enabled_ && point.x() >= self_min_x_ && point.x() <= self_max_x_ &&
            point.y() >= self_min_y_ && point.y() <= self_max_y_)
          continue;
        const double range = std::hypot(point.x(), point.y());
        if (range < range_min_ || range > range_max_)
          continue;
        const double angle = std::atan2(point.y(), point.x());
        if (angle < angle_min_ || angle >= angle_max_)
          continue;
        const auto bin = static_cast<std::size_t>((angle - angle_min_) / angle_increment_);
        if (bin < scan.ranges.size())
          scan.ranges[bin] = std::min(scan.ranges[bin], static_cast<float>(range));
      }
    }
    catch (const std::runtime_error &error)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud2 fields invalid: %s", error.what());
      return;
    }
    scan_pub_->publish(scan);
  }

  std::string input_topic_, output_topic_, target_frame_;
  double min_height_, max_height_, range_min_, range_max_;
  double self_min_x_, self_max_x_, self_min_y_, self_max_y_;
  double angle_min_, angle_max_, angle_increment_;
  bool use_inf_;
  bool self_filter_enabled_;
  tf2::Transform base_T_cloud_{tf2::Transform::getIdentity()};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};
}  // namespace go2w_nav2_navigation

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2w_nav2_navigation::CloudToScan>());
  rclcpp::shutdown();
  return 0;
}
