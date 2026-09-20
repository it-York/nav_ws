// Uses the public/protected extension API of Nav2 Humble CollisionMonitor.
// Static emergency-stop polygons use upstream collision_monitor_core.
// Optional dynamic slowdown evaluates measured/commanded motion and braking.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <nav2_collision_monitor/collision_monitor_node.hpp>
#include <nav2_collision_monitor/pointcloud.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "go2w_nav2_navigation/dynamic_slowdown.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace go2w_nav2_navigation {
using namespace nav2_collision_monitor;
using Clock=std::chrono::steady_clock;
class CheckedCloud : public PointCloud {
public:
  using PointCloud::PointCloud;
  mutable std::string failure;
  mutable int64_t stamp_ns{0};
  void configure() {
    PointCloud::configure();
    auto node=node_.lock();
    const auto topic=node->get_parameter(source_name_+".topic").as_string();
    data_sub_=node->create_subscription<sensor_msgs::msg::PointCloud2>(topic,rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){data_=m; received_=Clock::now();});
  }
  void getData(const rclcpp::Time & time,std::vector<Point> & points) const override {
    failure="missing_cloud";
    if (!data_) return;
    const auto & m=*data_;
    if (m.header.stamp.sec<0 || m.header.stamp.nanosec>=1000000000u) {failure="invalid_cloud_stamp";return;}
    const rclcpp::Time stamp(m.header.stamp);
    stamp_ns=stamp.nanoseconds();
    const double age=(time-stamp).seconds();
    failure="stale_or_future_cloud";
    if (stamp.nanoseconds()<=0 || age<-.005 || age>source_timeout_.seconds() ||
        std::chrono::duration<double>(Clock::now()-received_).count()>source_timeout_.seconds()) return;
    failure="invalid_cloud_layout";
    if (m.header.frame_id.empty() || m.is_bigendian || m.height!=1 || m.point_step<12 ||
        static_cast<uint64_t>(m.width)*m.point_step>m.row_step || m.data.size()<m.row_step) return;
    uint32_t offsets[3]; const char * names[]={"x","y","z"};
    for(int i=0;i<3;++i) {
      const auto it=std::find_if(m.fields.begin(),m.fields.end(),[&](const auto & f){return f.name==names[i];});
      if(it==m.fields.end() || it->datatype!=sensor_msgs::msg::PointField::FLOAT32 ||
         it->count!=1 || it->offset>m.point_step-sizeof(float)) return;
      offsets[i]=it->offset;
    }
    failure="pointcloud_tf_unavailable";
    tf2::Transform transform;
    try {
      // Keep true motion compensation: source at measurement time -> base NOW.
      const auto t=tf_buffer_->lookupTransform(base_frame_id_,time,m.header.frame_id,stamp,
        global_frame_id_,rclcpp::Duration::from_seconds(tf2::durationToSec(transform_tolerance_)));
      tf2::fromMsg(t.transform,transform);
    } catch(const tf2::TransformException &) {return;}
    const size_t original=points.size();
    for(uint32_t i=0;i<m.width;++i) {
      float xyz[3];
      for(int j=0;j<3;++j) std::memcpy(&xyz[j],m.data.data()+static_cast<size_t>(i)*m.point_step+offsets[j],sizeof(float));
      // The input is already filtered; a malformed value is not free space.
      if(!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
        points.resize(original);failure="nonfinite_cloud";return;
      }
      const auto p=transform*tf2::Vector3(xyz[0],xyz[1],xyz[2]);
      if (p.z()>=min_height_ && p.z()<=max_height_) points.push_back({p.x(),p.y()});
    }
    failure.clear();
  }
private:
  Clock::time_point received_;
};
class SafeCollisionMonitor : public CollisionMonitor {
public:
  SafeCollisionMonitor() : CollisionMonitor() {
    rcl_interfaces::msg::ParameterDescriptor model_parameter;
    model_parameter.read_only=true;
    model_parameter.description="Startup-only motion model; edit configuration and restart to apply.";
    dynamic_enabled_=declare_parameter("dynamic_slowdown.enabled",false,model_parameter);
    if(dynamic_enabled_) {
      dynamic_slowdown::Config c;
      c.length=declare_parameter("dynamic_slowdown.body_length",.75,model_parameter);
      c.width=declare_parameter("dynamic_slowdown.body_width",.45,model_parameter);
      c.margin=declare_parameter("dynamic_slowdown.margin",.04,model_parameter);
      c.reaction_time=declare_parameter("dynamic_slowdown.reaction_time",.20,model_parameter);
      c.preview_time=declare_parameter("dynamic_slowdown.preview_time",.60,model_parameter);
      c.linear_braking=declare_parameter("dynamic_slowdown.linear_braking",.50,model_parameter);
      c.angular_braking=declare_parameter("dynamic_slowdown.angular_braking",.80,model_parameter);
      c.linear_acceleration=declare_parameter("dynamic_slowdown.linear_acceleration",.40,model_parameter);
      c.angular_acceleration=declare_parameter("dynamic_slowdown.angular_acceleration",1.0,model_parameter);
      c.recovery_acceleration=declare_parameter("dynamic_slowdown.recovery_acceleration",.40,model_parameter);
      c.recovery_angular_acceleration=declare_parameter("dynamic_slowdown.recovery_angular_acceleration",1.0,model_parameter);
      c.release_hold=declare_parameter("dynamic_slowdown.release_hold",.15,model_parameter);
      c.step_time=declare_parameter("dynamic_slowdown.step_time",.04,model_parameter);
      c.step_distance=declare_parameter("dynamic_slowdown.step_distance",.01,model_parameter);
      c.max_linear_speed=declare_parameter("dynamic_slowdown.max_linear_speed",1.5,model_parameter);
      c.max_angular_speed=declare_parameter("dynamic_slowdown.max_angular_speed",2.0,model_parameter);
      c.scale_samples=declare_parameter("dynamic_slowdown.scale_samples",16,model_parameter);
      c.refinement_steps=declare_parameter("dynamic_slowdown.refinement_steps",5,model_parameter);
      governor_=std::make_unique<dynamic_slowdown::Governor>(c);
      processing_budget_=declare_parameter("dynamic_slowdown.max_processing_time",.08,model_parameter);
      if(!std::isfinite(processing_budget_)||processing_budget_<=0||processing_budget_>.1)
        throw std::invalid_argument("invalid collision processing budget");
      odom_timeout_=declare_parameter("dynamic_slowdown.odom_timeout",.15,model_parameter);
      odom_topic_=declare_parameter<std::string>("dynamic_slowdown.odom_topic","/odom",model_parameter);
      if(!std::isfinite(odom_timeout_)||odom_timeout_<=0||odom_timeout_>.3)
        throw std::invalid_argument("dynamic slowdown odometry timeout invalid");
    }
    prediction_timeout_=declare_parameter("prediction_timeout",0.15);
    command_timeout_=declare_parameter("command_timeout",0.3);
    if (!std::isfinite(prediction_timeout_) || prediction_timeout_<=0 ||
        !std::isfinite(command_timeout_) || command_timeout_<=0)
      throw std::invalid_argument("invalid monitor safety timeout");
  }
protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override {
    const auto result=CollisionMonitor::on_configure(state);
    if(result!=nav2_util::CallbackReturn::SUCCESS) return result;
    sources_.clear(); checked_.clear();
    const auto names=get_parameter("observation_sources").as_string_array();
    const auto base=get_parameter("base_frame_id").as_string();
    const auto odom=get_parameter("odom_frame_id").as_string();
    const auto tolerance=tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
    const auto timeout=rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());
    try {
      if(names.empty() || !get_parameter("base_shift_correction").as_bool() ||
         timeout.seconds()<=0 || timeout.seconds()>.5 || tf2::durationToSec(tolerance)>.05)
        throw std::runtime_error("required sources / motion compensation / timing budgets invalid");
      for(const auto & name:names) {
        if(get_parameter(name+".type").as_string()!="pointcloud")
          throw std::runtime_error("safe monitor currently supports PointCloud2 sources only");
        auto source=std::make_shared<CheckedCloud>(shared_from_this(),name,tf_buffer_,base,odom,tolerance,timeout,true);
        source->configure(); checked_.push_back(source);sources_.push_back(source);
      }
      for(const auto & polygon:polygons_)
        if((dynamic_enabled_ && polygon->getActionType()!=STOP) ||
           (polygon->getActionType()!=STOP && polygon->getActionType()!=SLOWDOWN))
          throw std::runtime_error("dynamic slowdown requires static STOP polygons only; legacy mode supports STOP/SLOWDOWN");
    } catch(const std::exception & e) {
      RCLCPP_ERROR(get_logger(),"Safe monitor configuration: %s",e.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
    ready_pub_=create_publisher<std_msgs::msg::Bool>("~/ready",1);
    reason_pub_=create_publisher<std_msgs::msg::String>("~/reason",1);
    duration_pub_=create_publisher<std_msgs::msg::Float32>("~/processing_ms",1);
    if(dynamic_enabled_) {
      markers_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("~/slow_zone",1);
      scale_pub_=create_publisher<std_msgs::msg::Float32>("~/speed_scale",1);
      reaction_pub_=create_publisher<std_msgs::msg::Float32>("~/effective_reaction_s",1);
      odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,rclcpp::SensorDataQoS().keep_last(1),
        [this,base,odom](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          if(m->header.stamp.sec<0 || m->header.stamp.nanosec>=1000000000u) {odom_valid_=false;return;}
          const int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
          const double age=(now().nanoseconds()-stamp)*1e-9;
          const auto & v=m->twist.twist;
          if(m->header.frame_id!=odom || m->child_frame_id!=base || stamp<=0 || age<-.005 || age>odom_timeout_ ||
             !std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z)) {
            odom_valid_=false;return;
          }
          if(stamp<=odom_stamp_) return;
          measured_={v.linear.x,v.linear.y,v.angular.z};odom_stamp_=stamp;odom_received_=Clock::now();odom_valid_=true;
        });
    }
    prediction_sub_=create_subscription<std_msgs::msg::Bool>("/odom_prediction/valid",1,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){prediction_valid_=m->data;prediction_received_=Clock::now();});
    cmd_vel_in_sub_=create_subscription<geometry_msgs::msg::Twist>(get_parameter("cmd_vel_in_topic").as_string(),1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        command_received_=Clock::now();have_command_=true;
        evaluate({m->linear.x,m->linear.y,m->angular.z});
      });
    watchdog_=create_wall_timer(std::chrono::milliseconds(50),[this] {
      if (process_active_ && (!have_command_ ||
          std::chrono::duration<double>(Clock::now()-command_received_).count()>command_timeout_))
        stop("command_timeout");
      else if (process_active_ && (!prediction_valid_ ||
          std::chrono::duration<double>(Clock::now()-prediction_received_).count()>prediction_timeout_))
        stop("prediction_invalid_or_stale");
      else if(process_active_ && dynamic_enabled_ && !odomFresh()) stop("dynamic_odometry_invalid_or_stale");
    });
    return result;
  }
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
    prediction_valid_=false;have_command_=false;
    if(dynamic_enabled_) {
      markers_pub_->on_activate();scale_pub_->on_activate();reaction_pub_->on_activate();
      governor_->reset(steadySeconds());last_evaluation_=Clock::now();
    }
    ready_pub_->on_activate();reason_pub_->on_activate();duration_pub_->on_activate();
    return CollisionMonitor::on_activate(state);
  }
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
    if (process_active_) stop("monitor_deactivated");
    const auto result=CollisionMonitor::on_deactivate(state);
    if(dynamic_enabled_) {markers_pub_->on_deactivate();scale_pub_->on_deactivate();reaction_pub_->on_deactivate();}
    ready_pub_->on_deactivate();reason_pub_->on_deactivate();duration_pub_->on_deactivate();return result;
  }
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override {
    watchdog_.reset();prediction_sub_.reset();checked_.clear();odom_sub_.reset();
    markers_pub_.reset();scale_pub_.reset();reaction_pub_.reset();odom_valid_=false;odom_stamp_=0;
    ready_pub_.reset();reason_pub_.reset();duration_pub_.reset();
    return CollisionMonitor::on_cleanup(state);
  }
private:
  static double steadySeconds() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
  bool odomFresh() const {
    const double age=(now().nanoseconds()-odom_stamp_)*1e-9;
    return odom_valid_ && age>=-.005 && age<=odom_timeout_ &&
      std::chrono::duration<double>(Clock::now()-odom_received_).count()<=odom_timeout_;
  }
  void showPaths(const dynamic_slowdown::Result & result,const rclcpp::Time & stamp) {
    visualization_msgs::msg::MarkerArray array;
    const auto & c=governor_->config();
    const double hx=c.length*.5+c.margin+c.step_distance,hy=c.width*.5+c.margin+c.step_distance;
    int id=0;
    for(const auto & path:result.paths) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id=get_parameter("base_frame_id").as_string();marker.header.stamp=stamp;
      marker.ns="motion_envelope";marker.id=id++;marker.type=visualization_msgs::msg::Marker::LINE_LIST;
      marker.action=visualization_msgs::msg::Marker::ADD;marker.pose.orientation.w=1.;marker.scale.x=.008;
      marker.color.r=1.;marker.color.g=.55;marker.color.b=0.;marker.color.a=marker.id==0?.9:.5;
      marker.lifetime=rclcpp::Duration::from_seconds(.3);
      // Decimate visualization only. Collision checking uses every sample.
      const size_t stride=std::max<size_t>(1,path.size()/18);
      for(size_t n=0;n<path.size();++n) {
        if(n%stride!=0 && n+1!=path.size()) continue;
        const auto & p=path[n];const double cs=std::cos(p.yaw),sn=std::sin(p.yaw);
        const double xs[4]={hx,hx,-hx,-hx},ys[4]={hy,-hy,-hy,hy};
        geometry_msgs::msg::Point corners[4];
        for(int i=0;i<4;++i) {corners[i].x=p.x+cs*xs[i]-sn*ys[i];corners[i].y=p.y+sn*xs[i]+cs*ys[i];corners[i].z=.04;}
        for(int i=0;i<4;++i) {marker.points.push_back(corners[i]);marker.points.push_back(corners[(i+1)%4]);}
      }
      array.markers.push_back(std::move(marker));
    }
    markers_pub_->publish(array);
  }
  void status(bool ok,const std::string & reason) {
    std_msgs::msg::Bool b;b.data=ok;ready_pub_->publish(b);
    std_msgs::msg::String s;s.data=reason;reason_pub_->publish(s);
  }
  void stop(const std::string & reason) {
    // Publish every failed evaluation, including startup. Never treat a failed
    // source transform as an obstacle-free observation.
    geometry_msgs::msg::Twist zero;cmd_vel_out_pub_->publish(zero);
    robot_action_prev_={STOP,{0.,0.,0.}};
    if(dynamic_enabled_) {governor_->reset(steadySeconds());std_msgs::msg::Float32 scale;scale.data=0;scale_pub_->publish(scale);}
    status(false,reason);
  }
  void evaluate(const Velocity & velocity) {
    if(!process_active_) return;
    const auto started=Clock::now();
    std::string failure;
    if (!prediction_valid_ || std::chrono::duration<double>(started-prediction_received_).count()>prediction_timeout_)
      failure="prediction_invalid_or_stale";
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.tw)) failure="invalid_command";
    if(dynamic_enabled_ && !odomFresh()) failure="dynamic_odometry_invalid_or_stale";
    std::vector<Point> points;
    const auto current=now();
    if (failure.empty()) for(const auto & source:checked_) {
      if(!source->getEnabled()) {failure="required_source_disabled";break;}
      source->getData(current,points);
      if(!source->failure.empty()) {failure=source->failure;break;}
    }
    bool have_stop=false;
    for(const auto & polygon:polygons_) have_stop|=polygon->getEnabled() && polygon->getActionType()==STOP;
    if (!have_stop) failure="no_enabled_stop_polygon";
    if (!failure.empty()) stop(failure);
    else {
      Action action{DO_NOTHING,velocity};
      for(const auto & polygon:polygons_) {
        if (!polygon->getEnabled()) continue;
        processStopSlowdown(polygon,points,velocity,action);
        if (action.action_type==STOP) break;
      }
      std::string reason=action.action_type==STOP?"obstacle_stop":action.action_type==SLOWDOWN?"obstacle_slowdown":"ok";
      bool model_ok=true;
      if(dynamic_enabled_) {
        if(action.action_type==STOP) {
          governor_->reset(steadySeconds());std_msgs::msg::Float32 scale;scale.data=0;scale_pub_->publish(scale);
        } else {
          try {
            if(!odomFresh()) throw std::runtime_error("dynamic_odometry_invalid_or_stale");
            double data_age=std::max(0.,(now().nanoseconds()-odom_stamp_)*1e-9);
            for(const auto & source:checked_) data_age=std::max(data_age,(now().nanoseconds()-source->stamp_ns)*1e-9);
            const double reaction=governor_->config().reaction_time+data_age;
            std::vector<dynamic_slowdown::Point> obstacle_points;obstacle_points.reserve(points.size());
            for(const auto & p:points) obstacle_points.push_back({p.x,p.y});
            const auto now_steady=Clock::now();
            const double dt=std::chrono::duration<double>(now_steady-last_evaluation_).count();
            const auto result=governor_->evaluate(obstacle_points,measured_,{velocity.x,velocity.y,velocity.tw},
              reaction,std::max(dt,1e-4),steadySeconds());
            action.req_vel={result.output.x,result.output.y,result.output.w};
            action.action_type=result.scale<1?SLOWDOWN:DO_NOTHING;
            if(result.scale<=1e-6) action.action_type=STOP;
            reason=result.measured_stop_unsafe?"measured_braking_path_blocked":
              result.limited?"dynamic_slowdown":result.scale<.999?"speed_recovery":"ok";
            std_msgs::msg::Float32 scale;scale.data=result.scale;scale_pub_->publish(scale);
            std_msgs::msg::Float32 reaction_msg;reaction_msg.data=reaction;reaction_pub_->publish(reaction_msg);
            showPaths(result,current);
          } catch(const std::exception & e) {stop(e.what());model_ok=false;}
        }
      }
      if(model_ok && dynamic_enabled_) {
        std::string overdue;
        if(std::chrono::duration<double>(Clock::now()-started).count()>processing_budget_) overdue="collision_processing_deadline";
        else if(!odomFresh()) overdue="dynamic_odometry_invalid_or_stale";
        else if(std::chrono::duration<double>(Clock::now()-prediction_received_).count()>prediction_timeout_) overdue="prediction_invalid_or_stale";
        else for(const auto & source:checked_)
          if((now().nanoseconds()-source->stamp_ns)*1e-9>get_parameter("source_timeout").as_double()) overdue="stale_or_future_cloud";
        if(!overdue.empty()) {stop(overdue);model_ok=false;}
      }
      if(model_ok) {publishVelocity(action);publishPolygons();robot_action_prev_=action;status(true,reason);}
    }
    last_evaluation_=Clock::now();
    std_msgs::msg::Float32 duration;
    duration.data=std::chrono::duration<double,std::milli>(Clock::now()-started).count();duration_pub_->publish(duration);
  }
  bool dynamic_enabled_{false},odom_valid_{false};
  double odom_timeout_{.15},processing_budget_{.08};int64_t odom_stamp_{0};std::string odom_topic_;
  Clock::time_point odom_received_,last_evaluation_;
  dynamic_slowdown::Twist measured_;
  std::unique_ptr<dynamic_slowdown::Governor> governor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr scale_pub_,reaction_pub_;
  double prediction_timeout_,command_timeout_;bool prediction_valid_{false},have_command_{false};
  Clock::time_point prediction_received_,command_received_;
  std::vector<std::shared_ptr<CheckedCloud>> checked_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr prediction_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr duration_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<go2w_nav2_navigation::SafeCollisionMonitor>();
  rclcpp::spin(node->get_node_base_interface());rclcpp::shutdown();return 0;
}
