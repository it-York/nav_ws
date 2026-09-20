#include <gtest/gtest.h>
#include "go2w_nav2_navigation/imu_prediction.hpp"
using namespace go2w_nav2_navigation;
std::deque<ImuSample> samples(const Eigen::Vector3d & w={0,0,0},const Eigen::Vector3d & a={0,0,1}) {
  std::deque<ImuSample> out;
  for(int i=0;i<=60;++i) out.push_back({i*.005,w,a});
  return out;
}
TEST(ImuPrediction, NormalizationGravityAndBias) {
  InertialState a;a.scale=9.809;a.time=.003;
  a.gyro_bias={.01,-.02,.03};a.accel_bias={.1,.2,-.1};
  const auto s=predict(a,samples(a.gyro_bias,Eigen::Vector3d(0,0,1)+a.accel_bias/a.scale),.203);
  EXPECT_LT(s.position.norm(),1e-10);EXPECT_LT(s.velocity.norm(),1e-10);
  EXPECT_LT(s.rotation.angularDistance(a.rotation),1e-10);
}
TEST(ImuPrediction, ConstantAccelerationAndDelayedAnchor) {
  InertialState a;a.scale=9.809;a.time=.04;a.position={.0008,0,0};a.velocity={.04,0,0};
  const auto s=predict(a,samples({0,0,0},{1/9.809,0,1}),.24);
  EXPECT_NEAR(s.position.x(),.5*.24*.24,1e-10);EXPECT_NEAR(s.velocity.x(),.24,1e-10);
}
TEST(ImuPrediction, ConstantYawAndLeverArm) {
  InertialState a;a.scale=9.809;
  const auto s=predict(a,samples({0,0,.7}),.20);
  EXPECT_NEAR(s.rotation.angularDistance(a.rotation),.14,1e-10);
  const Eigen::Vector3d lever(-.278256279,-.02329,-.112620959);
  const Eigen::Vector3d base=s.position+s.rotation*lever;
  EXPECT_NEAR(base.x(),std::cos(.14)*lever.x()-std::sin(.14)*lever.y(),1e-10);
}
TEST(ImuPrediction, TiltedStationarySensor) {
  InertialState a;a.scale=9.809;
  a.rotation=Eigen::AngleAxisd(M_PI/6,Eigen::Vector3d::UnitY());
  const auto s=predict(a,samples({0,0,0},a.rotation.conjugate()*Eigen::Vector3d::UnitZ()),.25);
  EXPECT_LT(s.velocity.norm(),1e-10);EXPECT_LT(s.position.norm(),1e-10);
}
TEST(ImuPrediction, BoundedTail) {
  InertialState a;a.scale=9.809;a.time=.1;
  auto imu=samples();
  const auto s=predict(a,imu,.32);
  EXPECT_NEAR(s.time,.32,1e-10);EXPECT_LT(s.velocity.norm(),1e-10);
  EXPECT_THROW(predict(a,imu,.35),std::runtime_error);
}
TEST(ImuPrediction, RejectOldAnchorAndFutureTarget) {
  InertialState a;a.time=.1;
  EXPECT_THROW(predict(a,samples(),.05),std::runtime_error);
  EXPECT_THROW(predict(a,samples(),.41),std::runtime_error);
}
TEST(ImuPrediction, MissingHistoryAndGap) {
  InertialState a;a.time=.1;
  auto imu=samples();imu.erase(imu.begin()+30,imu.begin()+40);
  EXPECT_THROW(predict(a,imu,.25),std::runtime_error);
  a.time=-.01;EXPECT_THROW(predict(a,samples(),.1),std::runtime_error);
}
TEST(ImuPrediction, TimestampRollback) {
  InertialState a;a.time=.1;
  auto imu=samples();imu[25].time=.1;
  EXPECT_THROW(predict(a,imu,.25),std::runtime_error);
}
