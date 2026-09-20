#pragma once
#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

// Initialization-only stationary window. Acceleration may be in g or m/s^2;
// dispersion is converted to m/s^2 using the measured gravity magnitude.
class StaticImuInit {
 public:
  void configure(double duration, double gyro_limit, double acc_std_limit) {
    if(!std::isfinite(duration) || duration<0 || !std::isfinite(gyro_limit) ||
       gyro_limit<=0 || !std::isfinite(acc_std_limit) || acc_std_limit<=0)
      throw std::invalid_argument("invalid imu_init parameters");
    duration_=duration;gyro_limit_=gyro_limit;acc_std_limit_=acc_std_limit;reset();
  }
  bool enabled() const {return duration_>0;}
  void reset() {count_=0;mean_.setZero();m2_.setZero();first_=last_=-1;}
  bool add(double stamp,const Eigen::Vector3d &acc,const Eigen::Vector3d &gyro) {
    if(!std::isfinite(stamp) || !acc.allFinite() || !gyro.allFinite() ||
       acc.norm()<0.1 || gyro.norm()>gyro_limit_ || (count_ && stamp<=last_)) {
      reset();return false;
    }
    if(!count_) first_=stamp;
    last_=stamp;++count_;
    const Eigen::Vector3d delta=acc-mean_;mean_+=delta/count_;
    m2_+=delta.cwiseProduct(acc-mean_);
    if(count_>=20 && (mean_.norm()<0.1 ||
       std::sqrt((m2_/(count_-1)).maxCoeff())*9.81/mean_.norm()>acc_std_limit_)) {
      reset();return false;
    }
    return count_>=100 && last_-first_>=duration_;
  }
  size_t count() const {return count_;}
 private:
  double duration_=0,gyro_limit_=0.1,acc_std_limit_=0.25,first_=-1,last_=-1;
  size_t count_=0;
  Eigen::Vector3d mean_=Eigen::Vector3d::Zero(),m2_=Eigen::Vector3d::Zero();
};
