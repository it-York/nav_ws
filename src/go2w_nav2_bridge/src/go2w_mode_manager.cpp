#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>

namespace go2w_nav2_bridge
{
class Go2wModeManager : public rclcpp::Node
{
public:
  Go2wModeManager() : Node("go2w_mode_manager")
  {
    target_mode_ = declare_parameter<std::string>("target_mode", "ai-w");
    check_period_ = declare_parameter<double>("check_period", 1.0);
    response_timeout_ = declare_parameter<double>("response_timeout", 2.0);
    auto_select_ = declare_parameter<bool>("auto_select", true);

    request_pub_ = create_publisher<unitree_api::msg::Request>(
        "/api/motion_switcher/request", 10);
    response_sub_ = create_subscription<unitree_api::msg::Response>(
        "/api/motion_switcher/response", 10,
        std::bind(&Go2wModeManager::responseCallback, this, std::placeholders::_1));
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/go2w/wheel_mode_ready", 10);

    ready_timer_ = create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&Go2wModeManager::publishReady, this));
    check_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(check_period_)),
        std::bind(&Go2wModeManager::checkMode, this));

    publishReady();
    RCLCPP_INFO(
        get_logger(), "Go2W mode guard started; required motion mode is '%s'",
        target_mode_.c_str());
  }

private:
  static constexpr std::int64_t kCheckModeApiId = 1001;
  static constexpr std::int64_t kSelectModeApiId = 1002;

  static std::int64_t requestId()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void checkMode()
  {
    const auto now = std::chrono::steady_clock::now();
    if (request_pending_ &&
        std::chrono::duration<double>(now - request_time_).count() < response_timeout_)
      return;

    if (request_pending_)
    {
      request_pending_ = false;
      wheel_mode_ready_ = false;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No response from Go2W motion_switcher; motion remains inhibited");
    }

    sendRequest(kCheckModeApiId, "");
  }

  void sendRequest(std::int64_t api_id, const std::string &parameter)
  {
    unitree_api::msg::Request request;
    pending_id_ = requestId();
    pending_api_id_ = api_id;
    request.header.identity.id = pending_id_;
    request.header.identity.api_id = api_id;
    request.parameter = parameter;
    request_pub_->publish(request);
    request_time_ = std::chrono::steady_clock::now();
    request_pending_ = true;
  }

  void responseCallback(const unitree_api::msg::Response::ConstSharedPtr response)
  {
    if (!request_pending_ || response->header.identity.id != pending_id_ ||
        response->header.identity.api_id != pending_api_id_)
      return;

    request_pending_ = false;
    if (response->header.status.code != 0)
    {
      wheel_mode_ready_ = false;
      RCLCPP_ERROR(
          get_logger(), "motion_switcher API %ld failed with code %d",
          pending_api_id_, response->header.status.code);
      return;
    }

    if (pending_api_id_ == kSelectModeApiId)
    {
      wheel_mode_ready_ = false;
      RCLCPP_INFO(get_logger(), "Requested Go2W motion mode '%s'; verifying", target_mode_.c_str());
      return;
    }

    try
    {
      const auto json = nlohmann::json::parse(response->data.data());
      const std::string mode = json.value("name", "");
      const std::string form = json.value("form", "");
      if (mode == target_mode_)
      {
        if (!wheel_mode_ready_)
          RCLCPP_INFO(get_logger(), "Go2W wheeled mode verified (form=%s, mode=%s)",
              form.c_str(), mode.c_str());
        wheel_mode_ready_ = true;
        last_verified_time_ = std::chrono::steady_clock::now();
      }
      else
      {
        wheel_mode_ready_ = false;
        RCLCPP_WARN(get_logger(), "Active motion mode is '%s' (form=%s), expected '%s'",
            mode.c_str(), form.c_str(), target_mode_.c_str());
        if (auto_select_)
        {
          nlohmann::json parameter;
          parameter["name"] = target_mode_;
          sendRequest(kSelectModeApiId, parameter.dump());
        }
      }
    }
    catch (const nlohmann::json::exception &error)
    {
      wheel_mode_ready_ = false;
      RCLCPP_ERROR(get_logger(), "Invalid motion_switcher response: %s", error.what());
    }
  }

  void publishReady()
  {
    const auto now = std::chrono::steady_clock::now();
    const bool fresh = wheel_mode_ready_ &&
        std::chrono::duration<double>(now - last_verified_time_).count() <=
            std::max(3.0 * check_period_, response_timeout_ + check_period_);
    if (!fresh)
      wheel_mode_ready_ = false;
    std_msgs::msg::Bool message;
    message.data = fresh;
    ready_pub_->publish(message);
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  std::string target_mode_;
  double check_period_{1.0};
  double response_timeout_{2.0};
  bool auto_select_{true};
  bool request_pending_{false};
  bool wheel_mode_ready_{false};
  std::int64_t pending_id_{0};
  std::int64_t pending_api_id_{0};
  std::chrono::steady_clock::time_point request_time_{};
  std::chrono::steady_clock::time_point last_verified_time_{};
};
}  // namespace go2w_nav2_bridge

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2w_nav2_bridge::Go2wModeManager>());
  rclcpp::shutdown();
  return 0;
}
