#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <fast_lio/msg/estimator_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "go2w_nav2_navigation/imu_prediction.hpp"
#include "go2w_nav2_navigation/odom_kinematics.hpp"

namespace go2w_nav2_navigation {
class ImuOdometry : public rclcpp::Node {
public:
  ImuOdometry() : Node("odom_prediction") {
    rate_=declare_parameter("publish_rate",50.0);
    horizon_=declare_parameter("max_prediction_horizon",0.30);
    tail_=declare_parameter("max_imu_age",0.04);
    gap_=declare_parameter("max_imu_gap",0.025);
    jump_=declare_parameter("max_correction_translation",0.15);
    angle_=declare_parameter("max_correction_rotation",0.15);
    const auto xyz=declare_parameter<std::vector<double>>("base_T_imu_xyz",{0.278256279,0.023290,0.112620959});
    const auto rpy=declare_parameter<std::vector<double>>("base_T_imu_rpy_deg",{0.,30.,0.});
    imu_frame_=declare_parameter<std::string>("imu_frame","livox_frame");
    for (double v : {rate_,horizon_,tail_,gap_,jump_,angle_})
      if (!std::isfinite(v)||v<=0) throw std::invalid_argument("invalid prediction limit");
    if (rate_>200 || horizon_>0.5 || tail_>0.1 || xyz.size()!=3 || rpy.size()!=3)
      throw std::invalid_argument("invalid prediction budget or extrinsics");
    for (double v:xyz) if (!std::isfinite(v)) throw std::invalid_argument("extrinsics");
    for (double v:rpy) if (!std::isfinite(v)) throw std::invalid_argument("extrinsics");
    tf2::Quaternion q; q.setRPY(rpy[0]*M_PI/180,rpy[1]*M_PI/180,rpy[2]*M_PI/180);
    base_T_imu_=tf2::Transform(q,tf2::Vector3(xyz[0],xyz[1],xyz[2]));
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/odom",5);
    valid_pub_=create_publisher<std_msgs::msg::Bool>("~/valid",1);
    reason_pub_=create_publisher<std_msgs::msg::String>("~/reason",1);
    anchor_age_pub_=create_publisher<std_msgs::msg::Float32>("~/anchor_age_ms",1);
    imu_age_pub_=create_publisher<std_msgs::msg::Float32>("~/imu_age_ms",1);
    correction_position_pub_=create_publisher<std_msgs::msg::Float32>("~/correction_position_m",1);
    correction_angle_pub_=create_publisher<std_msgs::msg::Float32>("~/correction_rotation_rad",1);
    duration_pub_=create_publisher<std_msgs::msg::Float32>("~/processing_ms",1);
    state_sub_=create_subscription<fast_lio::msg::EstimatorState>("/fastlio/estimator_state",5,
      [this](fast_lio::msg::EstimatorState::ConstSharedPtr m){anchorCallback(*m);});
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("/livox/imu",rclcpp::SensorDataQoS().keep_last(400),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){imuCallback(*m);});
    timer_=create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9/rate_)),[this]{tick();});
  }
private:
  using Clock=std::chrono::steady_clock;
  static Eigen::Vector3d v(const geometry_msgs::msg::Vector3 & m) {return {m.x,m.y,m.z};}
  void latch(const std::string & reason) {latched_=true; latch_reason_=reason; health(false,reason);}
  void health(bool ok,const std::string & reason) {
    std_msgs::msg::Bool b; b.data=ok; valid_pub_->publish(b);
    std_msgs::msg::String s; s.data=reason; reason_pub_->publish(s);
    if (!ok) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Prediction unavailable: %s",reason.c_str());
  }
  void metric(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & p,double value) {
    std_msgs::msg::Float32 m; m.data=value; p->publish(m);
  }
  void imuCallback(const sensor_msgs::msg::Imu & m) {
    const double t=rclcpp::Time(m.header.stamp).seconds();
    const double age=now().seconds()-t;
    if (m.header.frame_id!=imu_frame_ || !v(m.angular_velocity).allFinite() ||
        !v(m.linear_acceleration).allFinite() || t<=0 || age>2 || age<-.05) return;
    if (!raw_.empty() && t<=raw_.back().time) {
      if (t<raw_.back().time-.1) latch("imu_clock_rollback_restart_required");
      return;
    }
    if (v(m.angular_velocity).norm()>20 || v(m.linear_acceleration).norm()>200) {
      latch("imu_out_of_range_restart_required"); return;
    }
    raw_.push_back({t,v(m.angular_velocity),v(m.linear_acceleration)});
    while (!raw_.empty() && (t-raw_.front().time>2 || raw_.size()>4000)) raw_.pop_front();
    imu_received_=Clock::now();
  }
  std::deque<ImuSample> corrected(double offset) const {
    auto out=raw_; for (auto & s:out) s.time-=offset; return out;
  }
  void anchorCallback(const fast_lio::msg::EstimatorState & m) {
    if (latched_) return;
    InertialState a; a.time=rclcpp::Time(m.header.stamp).seconds();
    a.position={m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z};
    const auto & q=m.pose.pose.orientation;
    a.rotation=Eigen::Quaterniond(q.w,q.x,q.y,q.z);
    a.velocity=v(m.velocity_world); a.gyro_bias=v(m.gyro_bias);
    a.accel_bias=v(m.accel_bias); a.gravity=v(m.gravity_world); a.scale=m.accel_scale;
    if (have_anchor_ && a.time<=anchor_.time) {
      if (a.time<anchor_.time-.1) latch("estimator_clock_rollback_restart_required");
      return;
    }
    const double age=now().seconds()-a.time;
    if (m.header.frame_id!="camera_init" || a.time<=0 || age<0 || age>horizon_ ||
        !a.position.allFinite() || !a.velocity.allFinite() || !a.gyro_bias.allFinite() ||
        !a.accel_bias.allFinite() || !a.gravity.allFinite() ||
        !a.rotation.coeffs().allFinite() || std::abs(a.rotation.squaredNorm()-1)>0.01 ||
        !std::isfinite(a.scale) || a.scale<=0 || a.scale>100 ||
        a.gravity.norm()<8 || a.gravity.norm()>11 ||
        !std::isfinite(m.imu_time_offset) || std::abs(m.imu_time_offset)>.1 ||
        !std::all_of(m.pose.covariance.begin(),m.pose.covariance.end(),[](double x){return std::isfinite(x);}) ||
        !std::all_of(m.velocity_covariance.begin(),m.velocity_covariance.end(),[](double x){return std::isfinite(x);})) {
      health(false,"invalid_or_stale_anchor"); rejected_anchor_=true; return;
    }
    a.rotation.normalize();
    if (have_anchor_) {
      if (std::abs(m.imu_time_offset-offset_)>1e-6 || std::abs(a.scale-anchor_.scale)>1e-6) {
        latch("imu_calibration_changed_restart_required"); return;
      }
      try {
        const auto expected=predict(anchor_,corrected(offset_),a.time,horizon_,gap_,tail_);
        const double position_error=(expected.position-a.position).norm();
        const double angle_error=expected.rotation.angularDistance(a.rotation);
        metric(correction_position_pub_,position_error);metric(correction_angle_pub_,angle_error);
        if (position_error>jump_ || angle_error>angle_) {
          latch("estimator_correction_jump_restart_required"); return;
        }
      } catch (const std::exception &) {
        // A new correction may re-establish covered history after a short IMU gap.
        // For long estimator gaps require explicit restart to avoid hidden jumps.
        const double dt=a.time-anchor_.time;
        if (dt>horizon_) {latch("estimator_gap_restart_required"); return;}
        // During a short IMU gap retain a conservative continuity check against
        // the previous anchor (10 m/s^2 acceleration / 3 rad/s angular bound).
        if ((a.position-anchor_.position-anchor_.velocity*dt).norm()>jump_+5*dt*dt ||
            anchor_.rotation.angularDistance(a.rotation)>angle_+3*dt) {
          latch("uncovered_estimator_jump_restart_required");return;
        }
      }
    }
    anchor_=a; covariance_=m.pose.covariance; velocity_cov_=m.velocity_covariance;
    offset_=m.imu_time_offset; have_anchor_=true; rejected_anchor_=false; anchor_received_=Clock::now();
  }
  void tick() {
    const auto started=Clock::now(); const auto time=now(); const double target=time.seconds();
    if (previous_now_>0 && target<previous_now_) latch("ros_clock_rollback_restart_required");
    previous_now_=target;
    metric(anchor_age_pub_,have_anchor_?(target-anchor_.time)*1000:-1);
    metric(imu_age_pub_,raw_.empty()?-1:(target-raw_.back().time+offset_)*1000);
    if (latched_) {health(false,latch_reason_);return;}
    if (rejected_anchor_) {health(false,"invalid_or_stale_anchor");return;}
    if (!have_anchor_ || raw_.empty()) {health(false,"waiting_for_anchor_and_imu");return;}
    if (std::chrono::duration<double>(started-anchor_received_).count()>horizon_ ||
        std::chrono::duration<double>(started-imu_received_).count()>tail_) {
      health(false,"input_receipt_timeout");return;
    }
    try {
      auto samples=corrected(offset_);
      const auto s=predict(anchor_,samples,target,horizon_,gap_,tail_);
      tf2::Transform odom_T_imu(tf2::Quaternion(s.rotation.x(),s.rotation.y(),s.rotation.z(),s.rotation.w()),
        tf2::Vector3(s.position.x(),s.position.y(),s.position.z()));
      const auto odom_T_base=odom_T_imu*base_T_imu_.inverse();
      const auto lever=odom_T_base.getOrigin()-odom_T_imu.getOrigin();
      const auto w=s.rotation*(samples.back().gyro-s.gyro_bias);
      const tf2::Vector3 omega(w.x(),w.y(),w.z());
      const auto linear=odom_T_base.getBasis().transpose()*(tf2::Vector3(s.velocity.x(),s.velocity.y(),s.velocity.z())+omega.cross(lever));
      const auto angular=odom_T_base.getBasis().transpose()*omega;
      nav_msgs::msg::Odometry out;
      out.header.stamp=time; out.header.frame_id="odom"; out.child_frame_id="base_link";
      tf2::toMsg(odom_T_base,out.pose.pose);
      auto cov=covariance_; const double dt=target-anchor_.time;
      // Conservative diagonal growth, not a fully propagated EKF covariance.
      for(int i=0;i<6;++i) cov[i*6+i]+= i<3 ? (std::max(0.0,velocity_cov_[i*3+i])+0.04)*dt*dt : 0.01*dt;
      out.pose.covariance=shiftPoseCovariance(cov,lever);
      out.twist.twist.linear.x=linear.x();out.twist.twist.linear.y=linear.y();out.twist.twist.linear.z=linear.z();
      out.twist.twist.angular.x=angular.x();out.twist.twist.angular.y=angular.y();out.twist.twist.angular.z=angular.z();
      for(int i=0;i<6;++i) out.twist.covariance[i*6+i]=i<3?0.04:0.01;
      geometry_msgs::msg::TransformStamped transform;
      transform.header=out.header; transform.child_frame_id=out.child_frame_id;
      transform.transform=tf2::toMsg(odom_T_base);
      tf_->sendTransform(transform); odom_pub_->publish(out); health(true,"ok");
    } catch (const std::exception & e) {health(false,e.what());}
    metric(duration_pub_,std::chrono::duration<double,std::milli>(Clock::now()-started).count());
  }
  double rate_,horizon_,tail_,gap_,jump_,angle_,offset_{0},previous_now_{0};
  bool have_anchor_{false},latched_{false},rejected_anchor_{false}; std::string latch_reason_,imu_frame_;
  InertialState anchor_; std::deque<ImuSample> raw_;
  std::array<double,36> covariance_{}; std::array<double,9> velocity_cov_{};
  Clock::time_point anchor_received_,imu_received_;
  tf2::Transform base_T_imu_;
  rclcpp::Subscription<fast_lio::msg::EstimatorState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr anchor_age_pub_,imu_age_pub_,duration_pub_,correction_position_pub_,correction_angle_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_; rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<go2w_nav2_navigation::ImuOdometry>());
  rclcpp::shutdown();return 0;
}
