#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <unitree_api/msg/request.hpp>

namespace go2w_nav2_bridge
{
class CmdVelToSport : public rclcpp::Node
{
public:
  CmdVelToSport() : Node("cmd_vel_to_sport")
  {
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = declare_parameter<double>("max_vyaw", 1.0);
    min_linear_speed_ = declare_parameter<double>("min_linear_speed", 0.0);
    min_yaw_speed_ = declare_parameter<double>("min_yaw_speed", 0.0);
    linear_deadband_ = declare_parameter<double>("linear_deadband", 0.0);
    yaw_deadband_ = declare_parameter<double>("yaw_deadband", 0.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    localization_timeout_ = declare_parameter<double>("localization_timeout", 5.0);
    min_localization_confidence_ =
        declare_parameter<double>("min_localization_confidence", 0.7);
    max_abs_position_ = declare_parameter<double>("max_abs_position", 100.0);
    max_abs_z_ = declare_parameter<double>("max_abs_z", 5.0);
    max_odom_jump_ = declare_parameter<double>("max_odom_jump", 2.0);
    require_prediction_ = declare_parameter<bool>("require_prediction", false);
    prediction_timeout_ = declare_parameter<double>("prediction_timeout", 0.15);
    if (!std::isfinite(prediction_timeout_) || prediction_timeout_<=0)
      throw std::invalid_argument("prediction timeout must be positive");
    if (require_prediction_) prediction_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/odom_prediction/valid", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        prediction_valid_=msg->data; last_prediction_time_=std::chrono::steady_clock::now();
      });
    require_localization_ = declare_parameter<bool>("require_localization", false);
    require_obstacle_cloud_ =
        declare_parameter<bool>("require_obstacle_cloud", false);
    require_wheeled_mode_ = declare_parameter<bool>("require_wheeled_mode", true);
    wheeled_mode_timeout_ = declare_parameter<double>("wheeled_mode_timeout", 1.0);
    lock_on_goal_reached_ = declare_parameter<bool>("lock_on_goal_reached", false);
    obstacle_cloud_timeout_ =
        declare_parameter<double>("obstacle_cloud_timeout", 1.0);

    if (publish_rate <= 0.0 || cmd_timeout_ <= 0.0 ||
        max_vx_ <= 0.0 || max_vy_ <= 0.0 || max_vyaw_ <= 0.0 ||
        odom_timeout_ <= 0.0 || localization_timeout_ <= 0.0 ||
        obstacle_cloud_timeout_ <= 0.0 || wheeled_mode_timeout_ <= 0.0 ||
        max_abs_position_ <= 0.0 || max_abs_z_ <= 0.0 || max_odom_jump_ <= 0.0)
    {
      throw std::invalid_argument("rate, timeout, position and velocity limits must be positive");
    }
    if (min_linear_speed_ < 0.0 || min_yaw_speed_ < 0.0 ||
        linear_deadband_ < 0.0 || yaw_deadband_ < 0.0 ||
        min_linear_speed_ > std::hypot(max_vx_, max_vy_) ||
        min_yaw_speed_ > max_vyaw_)
    {
      throw std::invalid_argument("minimum speeds and deadbands must be non-negative and within limits");
    }

    sport_request_pub_ =
        create_publisher<unitree_api::msg::Request>("api/sport/request", 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&CmdVelToSport::cmdVelCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&CmdVelToSport::odomCallback, this, std::placeholders::_1));
    if (require_localization_)
    {
      localization_sub_ = create_subscription<std_msgs::msg::Float32>(
          "localization_confidence", 10,
          std::bind(&CmdVelToSport::localizationCallback, this, std::placeholders::_1));
    }
    if (require_obstacle_cloud_)
    {
      obstacle_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "obstacle_cloud", rclcpp::SensorDataQoS(),
          std::bind(&CmdVelToSport::obstacleCloudCallback, this, std::placeholders::_1));
    }
    if (require_wheeled_mode_)
    {
      wheeled_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
          "wheel_mode_ready", 10,
          std::bind(&CmdVelToSport::wheeledModeCallback, this, std::placeholders::_1));
    }
    if (lock_on_goal_reached_)
    {
      goal_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
          "navigate_to_pose_status", 10,
          std::bind(&CmdVelToSport::goalStatusCallback, this, std::placeholders::_1));
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&CmdVelToSport::publishCommand, this));

    RCLCPP_INFO(
        get_logger(),
        "Go2 velocity bridge ready (rate %.1f Hz, cmd timeout %.2f s, "
        "localization guard %s, obstacle-cloud guard %s, min linear %.2f m/s, "
        "min yaw %.2f rad/s)",
        publish_rate, cmd_timeout_, require_localization_ ? "enabled" : "disabled",
        require_obstacle_cloud_ ? "enabled" : "disabled", min_linear_speed_, min_yaw_speed_);
  }

  void stop()
  {
    publishStop("bridge shutting down");
  }

private:
  static constexpr std::int64_t kMoveApiId = 1008;
  static constexpr std::int64_t kStopMoveApiId = 1003;

  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z)) {
      latest_cmd_ = geometry_msgs::msg::Twist{}; have_cmd_=false;
      publishStop("nonfinite cmd_vel"); return;
    }
    double vx = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    double vy = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    const double linear_speed = std::hypot(vx, vy);
    if (linear_speed <= linear_deadband_)
    {
      vx = 0.0;
      vy = 0.0;
    }
    else if (linear_speed < min_linear_speed_)
    {
      const double scale = min_linear_speed_ / linear_speed;
      vx = std::clamp(vx * scale, -max_vx_, max_vx_);
      vy = std::clamp(vy * scale, -max_vy_, max_vy_);
    }

    double vyaw = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    if (std::abs(vyaw) <= yaw_deadband_)
      vyaw = 0.0;
    else if (std::abs(vyaw) < min_yaw_speed_)
      vyaw = std::copysign(min_yaw_speed_, vyaw);

    latest_cmd_.linear.x = vx;
    latest_cmd_.linear.y = vy;
    latest_cmd_.angular.z = vyaw;
    last_cmd_time_ = std::chrono::steady_clock::now();
    have_cmd_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto &position = msg->pose.pose.position;
    last_odom_stamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
    const auto & q=msg->pose.pose.orientation;
    const double qnorm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    bool valid = stampFresh(last_odom_stamp_,odom_timeout_) &&
        msg->header.frame_id=="odom" && msg->child_frame_id=="base_link" &&
        std::isfinite(qnorm) && std::abs(qnorm-1)<0.01 && std::isfinite(position.x) && std::isfinite(position.y) &&
        std::isfinite(position.z) && std::abs(position.x) <= max_abs_position_ &&
        std::abs(position.y) <= max_abs_position_ && std::abs(position.z) <= max_abs_z_;

    if (valid && have_valid_odom_)
    {
      const double dt = std::chrono::duration<double>(now - last_odom_time_).count();
      const double dx = position.x - last_odom_x_;
      const double dy = position.y - last_odom_y_;
      const double dz = position.z - last_odom_z_;
      if (dt > 0.0 && dt < 1.0 && std::sqrt(dx * dx + dy * dy + dz * dz) > max_odom_jump_)
        valid = false;
    }

    odom_received_ = true;
    odom_valid_ = valid;
    last_odom_time_ = now;
    if (valid)
    {
      have_valid_odom_ = true;
      last_odom_x_ = position.x;
      last_odom_y_ = position.y;
      last_odom_z_ = position.z;
    }
  }

  void localizationCallback(const std_msgs::msg::Float32::ConstSharedPtr msg)
  {
    localization_received_ = true;
    localization_valid_ = std::isfinite(msg->data) &&
        msg->data >= min_localization_confidence_;
    last_localization_time_ = std::chrono::steady_clock::now();
  }

  void obstacleCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    obstacle_cloud_received_ = msg->header.frame_id=="base_link";
    last_cloud_stamp_=rclcpp::Time(msg->header.stamp).nanoseconds();
    last_obstacle_cloud_time_ = std::chrono::steady_clock::now();
  }

  void wheeledModeCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    wheeled_mode_received_ = true;
    wheeled_mode_ready_ = msg->data;
    last_wheeled_mode_time_ = std::chrono::steady_clock::now();
  }

  void goalStatusCallback(const action_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
  {
    bool executing = false;
    bool succeeded = false;
    for (const auto &status : msg->status_list)
    {
      executing = executing || status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
      succeeded = succeeded || status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
    }

    if (executing)
    {
      goal_was_active_ = true;
      if (arrival_locked_)
        RCLCPP_INFO(get_logger(), "New Nav2 goal active; Go2W arrival lock released");
      arrival_locked_ = false;
      return;
    }
    // Status arrays can retain completed goals. Only a success following a
    // goal observed in ACCEPTED/EXECUTING state may engage the arrival lock.
    if (succeeded && goal_was_active_ && !arrival_locked_)
    {
      goal_was_active_ = false;
      arrival_locked_ = true;
      latest_cmd_ = geometry_msgs::msg::Twist{};
      have_cmd_ = false;
      publishStop("Nav2 goal reached; Go2W standing lock engaged");
    }
    else if (!succeeded)
    {
      goal_was_active_ = false;
    }
  }

  bool stampFresh(int64_t stamp,double limit) const {
    const double age=(this->now().nanoseconds()-stamp)*1e-9;
    return stamp>0 && age>=-.005 && age<=limit;
  }

  bool safetyReady(const std::chrono::steady_clock::time_point &now, std::string &reason) const
  {
    if (require_prediction_ && (!prediction_valid_ ||
        std::chrono::duration<double>(now-last_prediction_time_).count()>prediction_timeout_)) {
      reason="IMU prediction invalid or stale"; return false;
    }
    if (require_wheeled_mode_ &&
        (!wheeled_mode_received_ || !wheeled_mode_ready_ ||
         std::chrono::duration<double>(now - last_wheeled_mode_time_).count() >
             wheeled_mode_timeout_))
    {
      reason = "Go2W wheeled mode is not verified";
      return false;
    }
    if (!odom_received_ || !odom_valid_ || !stampFresh(last_odom_stamp_,odom_timeout_) ||
        std::chrono::duration<double>(now - last_odom_time_).count() > odom_timeout_)
    {
      reason = "odometry invalid or stale";
      return false;
    }
    if (require_localization_ &&
        (!localization_received_ || !localization_valid_ ||
         std::chrono::duration<double>(now - last_localization_time_).count() >
             localization_timeout_))
    {
      reason = "localization confidence invalid or stale";
      return false;
    }
    if (require_obstacle_cloud_ &&
        (!obstacle_cloud_received_ || !stampFresh(last_cloud_stamp_,obstacle_cloud_timeout_) ||
         std::chrono::duration<double>(now - last_obstacle_cloud_time_).count() >
             obstacle_cloud_timeout_))
    {
      reason = "local obstacle map stale";
      return false;
    }
    return true;
  }

  void publishCommand()
  {
    if (arrival_locked_)
    {
      if (!stop_sent_)
        publishStop("Go2W arrival lock active");
      return;
    }
    if (!have_cmd_)
      return;

    const auto now = std::chrono::steady_clock::now();
    std::string safety_reason;
    if (!safetyReady(now, safety_reason))
    {
      if (!stop_sent_)
        publishStop(safety_reason.c_str());
      return;
    }

    const double command_age = std::chrono::duration<double>(now - last_cmd_time_).count();
    if (command_age > cmd_timeout_)
    {
      if (!stop_sent_)
        publishStop("cmd_vel timed out");
      return;
    }

    unitree_api::msg::Request request;
    request.header.identity.api_id = kMoveApiId;
    request.parameter = moveParameter(
        latest_cmd_.linear.x, latest_cmd_.linear.y, latest_cmd_.angular.z);
    sport_request_pub_->publish(request);
    stop_sent_ = false;
  }

  void publishStop(const char *reason)
  {
    unitree_api::msg::Request request;
    request.header.identity.api_id = kStopMoveApiId;
    sport_request_pub_->publish(request);
    stop_sent_ = true;
    RCLCPP_WARN(get_logger(), "%s; sent StopMove to Go2", reason);
  }

  static std::string moveParameter(double vx, double vy, double vyaw)
  {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(8)
           << "{\"x\":" << vx
           << ",\"y\":" << vy
           << ",\"z\":" << vyaw << "}";
    return stream.str();
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr localization_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wheeled_mode_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  geometry_msgs::msg::Twist latest_cmd_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  bool have_cmd_{false};
  bool stop_sent_{true};
  bool odom_received_{false};
  bool odom_valid_{false};
  bool have_valid_odom_{false};
  bool localization_received_{false};
  bool localization_valid_{false};
  bool require_localization_{false};
  bool obstacle_cloud_received_{false};
  bool require_obstacle_cloud_{false};
  bool require_wheeled_mode_{true};
  bool wheeled_mode_received_{false};
  bool wheeled_mode_ready_{false};
  bool lock_on_goal_reached_{false};
  bool goal_was_active_{false};
  bool arrival_locked_{false};
  double last_odom_x_{0.0}, last_odom_y_{0.0}, last_odom_z_{0.0};
  std::chrono::steady_clock::time_point last_odom_time_;
  std::chrono::steady_clock::time_point last_localization_time_;
  std::chrono::steady_clock::time_point last_obstacle_cloud_time_;
  std::chrono::steady_clock::time_point last_wheeled_mode_time_;
  bool require_prediction_{false}, prediction_valid_{false};
  double prediction_timeout_{0.15};
  int64_t last_odom_stamp_{0},last_cloud_stamp_{0};
  std::chrono::steady_clock::time_point last_prediction_time_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr prediction_sub_;
  double cmd_timeout_, odom_timeout_, localization_timeout_, obstacle_cloud_timeout_;
  double wheeled_mode_timeout_;
  double min_localization_confidence_, max_abs_position_, max_abs_z_, max_odom_jump_;
  double max_vx_, max_vy_, max_vyaw_;
  double min_linear_speed_, min_yaw_speed_, linear_deadband_, yaw_deadband_;
};
}  // namespace go2w_nav2_bridge

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2w_nav2_bridge::CmdVelToSport>();
  std::weak_ptr<go2w_nav2_bridge::CmdVelToSport> weak_node = node;
  node->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [weak_node]() {
        if (const auto locked_node = weak_node.lock())
        {
          locked_node->stop();
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
