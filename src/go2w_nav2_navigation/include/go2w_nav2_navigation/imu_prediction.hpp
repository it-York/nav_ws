#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <stdexcept>
#include <cmath>

namespace go2w_nav2_navigation {
struct ImuSample {
  double time;
  Eigen::Vector3d gyro, accel;
};
struct InertialState {
  double time{0}, scale{1};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}, velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()}, accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity{0,0,-9.809};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};
inline Eigen::Quaterniond rotationIncrement(const Eigen::Vector3d & angle) {
  const double n = angle.norm();
  return n < 1e-12 ? Eigen::Quaterniond::Identity() :
    Eigen::Quaterniond(Eigen::AngleAxisd(n, angle/n));
}
// Midpoint integration. Biases, gravity and units come from the SAME FAST-LIO
// correction as the pose and velocity; never estimate them independently here.
inline void integrate(InertialState & s, const ImuSample & a, const ImuSample & b) {
  const double dt = b.time-a.time;
  const Eigen::Vector3d w = (a.gyro+b.gyro)*0.5-s.gyro_bias;
  const Eigen::Vector3d force = (a.accel+b.accel)*(0.5*s.scale)-s.accel_bias;
  const Eigen::Quaterniond middle = s.rotation * rotationIncrement(w*(0.5*dt));
  const Eigen::Vector3d acc = middle*force+s.gravity;
  s.position += s.velocity*dt+acc*(0.5*dt*dt);
  s.velocity += acc*dt;
  s.rotation = (s.rotation*rotationIncrement(w*dt)).normalized();
  s.time = b.time;
}
inline ImuSample interpolate(const ImuSample & a, const ImuSample & b, double t) {
  const double f = (t-a.time)/(b.time-a.time);
  return {t, a.gyro+(b.gyro-a.gyro)*f, a.accel+(b.accel-a.accel)*f};
}
inline InertialState predict(const InertialState & anchor,
  const std::deque<ImuSample> & samples, double target,
  double horizon=0.30, double max_gap=0.025, double max_tail=0.04)
{
  if (!std::isfinite(target) || target < anchor.time || target-anchor.time > horizon)
    throw std::runtime_error("anchor_expired_or_future");
  if (samples.empty() || samples.front().time > anchor.time ||
      target-samples.back().time > max_tail)
    throw std::runtime_error("imu_history_missing_or_stale");
  for (size_t i=1; i<samples.size(); ++i)
    if (samples[i].time <= samples[i-1].time)
      throw std::runtime_error("imu_time_not_monotonic");
  size_t i=0;
  while (i+1<samples.size() && samples[i+1].time <= anchor.time) ++i;
  ImuSample prev=samples[i];
  if (i+1<samples.size()) {
    if (samples[i+1].time-prev.time > max_gap)
      throw std::runtime_error("imu_gap_at_anchor");
    prev=interpolate(prev,samples[i+1],anchor.time);
  } else {
    if (anchor.time-prev.time > max_tail) throw std::runtime_error("imu_anchor_uncovered");
    prev.time=anchor.time;
  }
  InertialState result=anchor;
  while (i+1<samples.size() && result.time<target) {
    const auto & next=samples[++i];
    if (next.time-samples[i-1].time > max_gap) throw std::runtime_error("imu_gap");
    const auto end = next.time>target ? interpolate(prev,next,target) : next;
    integrate(result,prev,end); prev=end;
  }
  if (result.time<target) {
    if (target-prev.time > max_tail) throw std::runtime_error("imu_tail_exceeded");
    auto end=prev; end.time=target; integrate(result,prev,end);
  }
  if (!result.position.allFinite() || !result.velocity.allFinite() ||
      !result.rotation.coeffs().allFinite()) throw std::runtime_error("nonfinite_prediction");
  return result;
}
} // namespace go2w_nav2_navigation
