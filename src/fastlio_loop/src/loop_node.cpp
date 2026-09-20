#include "fastlio_loop/backend.hpp"
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <atomic>
#include <algorithm>
#include <iostream>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <thread>

namespace fastlio_loop {
class LoopNode:public rclcpp::Node {
  using Odom=nav_msgs::msg::Odometry;
  using PointCloud=sensor_msgs::msg::PointCloud2;
  using Policy=message_filters::sync_policies::ExactTime<Odom,PointCloud>;
  struct Task {
    Odom::ConstSharedPtr odom;
    PointCloud::ConstSharedPtr cloud;
    std::shared_ptr<std::promise<std::string>> save;
  };
 public:
  LoopNode():Node("fastlio_loop_backend") {
    Options o;
    o.keyframe_distance=declare_parameter("keyframe_distance",o.keyframe_distance);
    o.keyframe_angle=declare_parameter("keyframe_angle",o.keyframe_angle);
    o.min_loop_age=declare_parameter("min_loop_age",o.min_loop_age);
    o.search_radius=declare_parameter("search_radius",o.search_radius);
    o.min_loop_separation=declare_parameter("min_loop_separation",o.min_loop_separation);
    o.submap_neighbors=declare_parameter("submap_neighbors",o.submap_neighbors);
    o.max_candidates=declare_parameter("max_candidates",o.max_candidates);
    o.confirmation_count=declare_parameter("confirmation_count",o.confirmation_count);
    o.confirmation_distance=declare_parameter("confirmation_distance",o.confirmation_distance);
    o.confirmation_angle=declare_parameter("confirmation_angle",o.confirmation_angle);
    o.loop_cooldown=declare_parameter("loop_cooldown",o.loop_cooldown);
    o.max_keyframes=declare_parameter("max_keyframes",o.max_keyframes);
    o.cloud_max_range=declare_parameter("cloud_max_range",o.cloud_max_range);
    o.map_leaf=declare_parameter("map_leaf",o.map_leaf);
    auto &r=o.registration;
    r.leaf=declare_parameter("registration.leaf",r.leaf);
    r.coarse_distance=declare_parameter("registration.coarse_distance",r.coarse_distance);
    r.inlier_distance=declare_parameter("registration.inlier_distance",r.inlier_distance);
    r.min_overlap=declare_parameter("registration.min_overlap",r.min_overlap);
    r.min_reverse_overlap=declare_parameter("registration.min_reverse_overlap",r.min_reverse_overlap);
    r.max_rmse=declare_parameter("registration.max_rmse",r.max_rmse);
    r.min_observability=declare_parameter("registration.min_observability",r.min_observability);
    r.max_correction=declare_parameter("registration.max_correction",r.max_correction);
    r.max_rotation=declare_parameter("registration.max_rotation",r.max_rotation);
    r.min_inliers=declare_parameter("registration.min_inliers",r.min_inliers);
    backend_=std::make_unique<Backend>(o);
    map_frame_=declare_parameter<std::string>("map_frame","map");
    odom_frame_=declare_parameter<std::string>("odom_frame","camera_init");
    body_frame_=declare_parameter<std::string>("body_frame","body");
    map_path_=declare_parameter<std::string>("map_path","magic2_loop.pcd");
    overwrite_=declare_parameter("overwrite_map",false);
    map_period_=declare_parameter("map_publish_period",5.0);
    queue_limit_=declare_parameter("queue_limit",20);
    if(map_frame_==odom_frame_ || map_period_<=0 || queue_limit_<1)
      throw std::invalid_argument("invalid frames, map period or queue size");
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_=create_publisher<Odom>("/slam/odometry",10);
    map_pub_=create_publisher<PointCloud>("/slam/map",rclcpp::QoS(1).transient_local());
    path_pub_=create_publisher<nav_msgs::msg::Path>("/slam/path",rclcpp::QoS(1).transient_local());
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/slam/diagnostics",10);
    odom_sub_.subscribe(this,"/Odometry",rmw_qos_profile_sensor_data);
    cloud_sub_.subscribe(this,"/cloud_registered_body",rmw_qos_profile_sensor_data);
    sync_=std::make_shared<message_filters::Synchronizer<Policy>>(Policy(30),odom_sub_,cloud_sub_);
    sync_->registerCallback(std::bind(&LoopNode::pair,this,std::placeholders::_1,std::placeholders::_2));
    service_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    save_service_=create_service<std_srvs::srv::Trigger>("/map_save",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        if(save_pending_.exchange(true)) {response->message="A map save is already in progress";return;}
        auto promise=std::make_shared<std::promise<std::string>>();auto future=promise->get_future();
        {std::lock_guard<std::mutex> lock(queue_mutex_);queue_.push_back({nullptr,nullptr,promise});}
        condition_.notify_one();
        if(future.wait_for(std::chrono::seconds(60))!=std::future_status::ready) {
          response->message="Save is still running; check diagnostics and destination before retrying";return;
        }
        try {response->message=future.get();response->success=true;}
        catch(const std::exception &e) {response->message=e.what();response->success=false;}
      },rmw_qos_profile_services_default,service_group_);
    last_input_=std::chrono::steady_clock::now();
    status_timer_=create_wall_timer(std::chrono::seconds(1),std::bind(&LoopNode::publishStatus,this));
    worker_=std::thread(&LoopNode::work,this);
    RCLCPP_INFO(get_logger(),"Loop backend ready; map_save writes %s; overwrite=%s",
      map_path_.c_str(),overwrite_?"true":"false");
  }
  ~LoopNode() override {
    stop_=true;condition_.notify_all();if(worker_.joinable()) worker_.join();
    for(auto &task:queue_) if(task.save) task.save->set_exception(
      std::make_exception_ptr(std::runtime_error("Backend shut down before save")));
  }
 private:
  static Pose fromMessage(const geometry_msgs::msg::Pose &p) {
    Pose out;out.t={p.position.x,p.position.y,p.position.z};
    out.q=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
    return out;
  }
  static geometry_msgs::msg::Pose toMessage(const Pose &p) {
    geometry_msgs::msg::Pose out;
    out.position.x=p.t.x();out.position.y=p.t.y();out.position.z=p.t.z();
    out.orientation.x=p.q.x();out.orientation.y=p.q.y();out.orientation.z=p.q.z();out.orientation.w=p.q.w();return out;
  }
  void pair(const Odom::ConstSharedPtr &odom,const PointCloud::ConstSharedPtr &cloud) {
    if(odom->header.frame_id!=odom_frame_ || odom->child_frame_id!=body_frame_ || cloud->header.frame_id!=body_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Input frame mismatch; expected %s -> %s and cloud in %s",
        odom_frame_.c_str(),body_frame_.c_str(),body_frame_.c_str());return;
    }
    const Pose pose=fromMessage(odom->pose.pose);
    if(!pose.finite()) {RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Invalid frontend pose rejected");return;}
    const auto stamp=rclcpp::Time(odom->header.stamp).nanoseconds();
    if(stamp<=last_stamp_) {RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Time moved backwards; restart both SLAM nodes for a new bag");return;}
    last_stamp_=stamp;
    Pose correction;
    {std::lock_guard<std::mutex> lock(state_mutex_);correction=correction_;last_input_=std::chrono::steady_clock::now();}
    Odom output=*odom;output.header.frame_id=map_frame_;output.pose.pose=toMessage(correction*pose);
    // Rotate the frontend pose covariance into the corrected map frame.
    Eigen::Matrix<double,6,6> covariance,rotation=Eigen::Matrix<double,6,6>::Zero();
    for(int i=0;i<6;++i) for(int j=0;j<6;++j) covariance(i,j)=odom->pose.covariance[i*6+j];
    rotation.block<3,3>(0,0)=correction.q.toRotationMatrix();rotation.block<3,3>(3,3)=correction.q.toRotationMatrix();
    covariance=rotation*covariance*rotation.transpose();
    for(int i=0;i<6;++i) for(int j=0;j<6;++j) output.pose.covariance[i*6+j]=covariance(i,j);
    odom_pub_->publish(output);
    geometry_msgs::msg::TransformStamped tf;
    tf.header=output.header;tf.child_frame_id=odom_frame_;
    tf.transform.translation.x=correction.t.x();tf.transform.translation.y=correction.t.y();tf.transform.translation.z=correction.t.z();
    tf.transform.rotation=toMessage(correction).orientation;tf_->sendTransform(tf);
    {std::lock_guard<std::mutex> lock(queue_mutex_);
      if(queue_.size()>=static_cast<size_t>(queue_limit_)) {
        auto it=std::find_if(queue_.begin(),queue_.end(),[](const Task &t){return !t.save;});
        if(it!=queue_.end()) {queue_.erase(it);++dropped_;}
      }
      queue_.push_back({odom,cloud,nullptr});}
    condition_.notify_one();
  }
  void work() {
    auto last_map=std::chrono::steady_clock::now()-std::chrono::seconds(60);
    bool map_dirty=false;
    while(!stop_) {
      Task task;
      {std::unique_lock<std::mutex> lock(queue_mutex_);
        condition_.wait(lock,[this]{return stop_ || !queue_.empty();});
        if(stop_) break;
        task=queue_.front();queue_.pop_front();}
      try {
        if(task.save) {
          const auto message=backend_->save(map_path_,overwrite_);
          RCLCPP_INFO(get_logger(),"%s",message.c_str());task.save->set_value(message);save_pending_=false;continue;
        }
        Cloud::Ptr cloud(new Cloud);pcl::fromROSMsg(*task.cloud,*cloud);
        const double stamp=rclcpp::Time(task.odom->header.stamp).seconds();
        const auto start=std::chrono::steady_clock::now();
        auto update=backend_->addFrame(stamp,fromMessage(task.odom->pose.pose),cloud);
        {std::lock_guard<std::mutex> lock(state_mutex_);
          correction_=backend_->correction();keyframes_=backend_->keyframes().size();loops_=backend_->loopCount();
          if(update.reason!="below keyframe motion threshold") last_reason_=update.reason;
          if(update.keyframe) {
            registration_=update.registration;
            processing_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
          }}
        if(update.loop) RCLCPP_INFO(get_logger(),"LOOP ACCEPTED: keyframes=%zu loops=%zu RMSE=%.3f overlap=%.2f",
          backend_->keyframes().size(),backend_->loopCount(),update.registration.rmse,update.registration.overlap);
        if(update.keyframe) {
          map_dirty=true;
          nav_msgs::msg::Path path;path.header=task.odom->header;path.header.frame_id=map_frame_;
          for(size_t i=0;i<backend_->poses().size();++i) {
            geometry_msgs::msg::PoseStamped pose;pose.header.frame_id=map_frame_;
            pose.header.stamp=rclcpp::Time(static_cast<int64_t>(backend_->keyframes()[i].stamp*1e9));
            pose.pose=toMessage(backend_->poses()[i]);path.poses.push_back(pose);
          }
          path_pub_->publish(path);
        }
        const auto now=std::chrono::steady_clock::now();
        if(map_dirty && map_pub_->get_subscription_count()>0 &&
            (update.loop || std::chrono::duration<double>(now-last_map).count()>=map_period_)) {
          PointCloud message;pcl::toROSMsg(*backend_->buildMap(),message);
          message.header=task.odom->header;message.header.frame_id=map_frame_;
          map_pub_->publish(message);last_map=now;map_dirty=false;
        }
      } catch(const std::exception &e) {
        RCLCPP_ERROR(get_logger(),"Backend operation failed: %s",e.what());
        if(task.save) {task.save->set_exception(std::current_exception());save_pending_=false;}
        std::lock_guard<std::mutex> lock(state_mutex_);last_reason_=std::string("ERROR: ")+e.what();
      }
    }
  }
  void publishStatus() {
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name="fastlio_loop";status.hardware_id="MID360s";
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-last_input_).count();
    status.level=(age>2)?diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      (loops_==0 || dropped_>0)?diagnostic_msgs::msg::DiagnosticStatus::WARN:diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message=age>2?"No synchronized odometry/body cloud received":last_reason_;
    auto add=[&status](const std::string &key,const std::string &value) {
      diagnostic_msgs::msg::KeyValue item;item.key=key;item.value=value;status.values.push_back(item);
    };
    add("keyframes",std::to_string(keyframes_));add("accepted_loops",std::to_string(loops_));
    add("dropped_pairs",std::to_string(dropped_.load()));add("input_age_seconds",std::to_string(age));
    add("keyframe_processing_ms",std::to_string(processing_ms_));
    add("last_candidate_rmse",std::to_string(registration_.rmse));
    add("last_candidate_overlap",std::to_string(registration_.overlap));
    add("last_candidate_observability",std::to_string(registration_.observability));
    array.status.push_back(status);diagnostics_->publish(array);
  }
  std::unique_ptr<Backend> backend_;
  message_filters::Subscriber<Odom> odom_sub_;
  message_filters::Subscriber<PointCloud> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<PointCloud>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::thread worker_;std::atomic<bool> stop_{false},save_pending_{false};
  std::atomic<size_t> dropped_{0};
  std::mutex queue_mutex_,state_mutex_;std::condition_variable condition_;
  std::deque<Task> queue_;Pose correction_;
  std::chrono::steady_clock::time_point last_input_;
  int64_t last_stamp_=-1;int queue_limit_=20;
  size_t keyframes_=0,loops_=0;double processing_ms_=0,map_period_=5;
  std::string map_frame_,odom_frame_,body_frame_,map_path_,last_reason_="Waiting for first keyframe";
  bool overwrite_=false;RegistrationResult registration_;
};
}  // namespace fastlio_loop
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<fastlio_loop::LoopNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);executor.spin();
  } catch(const std::exception &e) {std::cerr<<"Loop backend: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
