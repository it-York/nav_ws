#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace go2w_nav2_navigation
{
class CloudObstacleFilter : public rclcpp::Node
{
public:
  CloudObstacleFilter() : Node("cloud_obstacle_filter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cloud_obstacles");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    min_height_ = declare_parameter<double>("min_height", 0.08);
    max_height_ = declare_parameter<double>("max_height", 0.80);
    min_range_ = declare_parameter<double>("min_range", 0.12);
    max_range_ = declare_parameter<double>("max_range", 8.0);
    voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.03);
    self_min_x_ = declare_parameter<double>("self_min_x", -0.50);
    self_max_x_ = declare_parameter<double>("self_max_x", 0.55);
    self_min_y_ = declare_parameter<double>("self_min_y", -0.32);
    self_max_y_ = declare_parameter<double>("self_max_y", 0.32);
    const auto xyz = declare_parameter<std::vector<double>>(
        "base_T_cloud_xyz", {0.278256279, 0.023290, 0.112620959});
    const auto rpy = declare_parameter<std::vector<double>>(
        "base_T_cloud_rpy_deg", {0.0, 30.0, 0.0});

    if (xyz.size() != 3 || rpy.size() != 3 || min_height_ >= max_height_ ||
        min_range_ < 0.0 || min_range_ >= max_range_ || voxel_leaf_ <= 0.0 ||
        self_min_x_ >= self_max_x_ || self_min_y_ >= self_max_y_)
      throw std::invalid_argument("invalid cloud obstacle filter parameters");

    constexpr double deg = M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(rpy[0] * deg, rpy[1] * deg, rpy[2] * deg);
    q.normalize();
    base_T_cloud_.setOrigin(tf2::Vector3(xyz[0], xyz[1], xyz[2]));
    base_T_cloud_.setRotation(q);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::SensorDataQoS());
    subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CloudObstacleFilter::callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Filtering %s into %s (%s frame)", input_topic_.c_str(),
        output_topic_.c_str(), target_frame_.c_str());
  }

private:
  struct Point {float x; float y; float z;};

  static std::uint64_t voxelKey(int x, int y, int z)
  {
    constexpr std::int64_t offset = 1 << 20;
    const auto ux = static_cast<std::uint64_t>(static_cast<std::int64_t>(x) + offset) & 0x1fffff;
    const auto uy = static_cast<std::uint64_t>(static_cast<std::int64_t>(y) + offset) & 0x1fffff;
    const auto uz = static_cast<std::uint64_t>(static_cast<std::int64_t>(z) + offset) & 0x1fffff;
    return (ux << 42) | (uy << 21) | uz;
  }

  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    std::vector<Point> points;
    points.reserve(cloud->width * cloud->height / 2);
    std::unordered_set<std::uint64_t> occupied;
    occupied.reserve(cloud->width * cloud->height / 2);
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z)
      {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
          continue;
        const tf2::Vector3 p = base_T_cloud_ * tf2::Vector3(*x, *y, *z);
        if (p.z() < min_height_ || p.z() > max_height_)
          continue;
        if (p.x() >= self_min_x_ && p.x() <= self_max_x_ &&
            p.y() >= self_min_y_ && p.y() <= self_max_y_)
          continue;
        const double range = std::hypot(p.x(), p.y());
        if (range < min_range_ || range > max_range_)
          continue;
        const int vx = static_cast<int>(std::floor(p.x() / voxel_leaf_));
        const int vy = static_cast<int>(std::floor(p.y() / voxel_leaf_));
        const int vz = static_cast<int>(std::floor(p.z() / voxel_leaf_));
        if (!occupied.insert(voxelKey(vx, vy, vz)).second)
          continue;
        points.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()),
            static_cast<float>(p.z())});
      }
    }
    catch (const std::runtime_error &error)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "PointCloud2 fields invalid: %s", error.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud->header;
    output.header.frame_id = target_frame_;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> ox(output, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(output, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(output, "z");
    for (const auto &p : points)
    {
      *ox = p.x; *oy = p.y; *oz = p.z;
      ++ox; ++oy; ++oz;
    }
    publisher_->publish(output);
  }

  std::string input_topic_, output_topic_, target_frame_;
  double min_height_, max_height_, min_range_, max_range_, voxel_leaf_;
  double self_min_x_, self_max_x_, self_min_y_, self_max_y_;
  tf2::Transform base_T_cloud_{tf2::Transform::getIdentity()};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
};
}  // namespace go2w_nav2_navigation

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2w_nav2_navigation::CloudObstacleFilter>());
  rclcpp::shutdown();
  return 0;
}
