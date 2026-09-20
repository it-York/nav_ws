#include <gtest/gtest.h>
#include <chrono>
#include "go2w_nav2_navigation/dynamic_slowdown.hpp"
using namespace go2w_nav2_navigation::dynamic_slowdown;
Result settle(Governor & g,const std::vector<Point> & points,Twist measured,Twist command,double start=1) {
  Result out;
  for(int i=0;i<100;++i) out=g.evaluate(points,measured,command,.2,.05,start+i*.05);
  return out;
}
TEST(DynamicSlowdown, ParallelSideWallsDoNotLimitStraightMotion) {
  std::vector<Point> points;
  for(int i=-20;i<=60;++i) {points.push_back({i*.05,.4});points.push_back({i*.05,-.4});}
  Governor g;const auto r=settle(g,points,{.6,0,0},{.6,0,0});
  EXPECT_NEAR(r.output.x,.6,1e-9);EXPECT_FALSE(r.measured_stop_unsafe);
}
TEST(DynamicSlowdown, ForwardObstacleLimitsContinuously) {
  Governor far,near;
  const auto f=settle(far,{{1.1,0}},{.25,0,0},{.6,0,0});
  const auto n=settle(near,{{.8,0}},{.25,0,0},{.6,0,0});
  EXPECT_GT(n.output.x,0);EXPECT_LT(n.output.x,f.output.x);EXPECT_LT(f.output.x,.6);
  EXPECT_NE(n.scale,.55); // No fixed 55% tier.
}
TEST(DynamicSlowdown, ReverseProtectsRearAndIgnoresDistantFront) {
  Governor front,rear;
  const auto f=settle(front,{{1.,0}},{-.1,0,0},{-.18,0,0});
  const auto r=settle(rear,{{-.55,0}},{-.1,0,0},{-.18,0,0});
  EXPECT_NEAR(f.output.x,-.18,1e-9);EXPECT_GT(r.output.x,-.18);EXPECT_LT(r.output.x,0);
}
TEST(DynamicSlowdown, RotationCoversFrontAndRearCorners) {
  Governor left,right,rear;
  const auto l=settle(left,{{.2,.38}},{},{0,0,.7});
  const auto r=settle(right,{{.2,.38}},{},{0,0,-.7});
  const auto b=settle(rear,{{-.2,-.38}},{},{0,0,.7});
  EXPECT_GT(l.output.w,0);EXPECT_LT(l.output.w,.7);
  EXPECT_NEAR(r.output.w,-.7,1e-9);EXPECT_LT(b.output.w,.7);
}
TEST(DynamicSlowdown, MeasuredInertiaSurvivesZeroOrReverseCommand) {
  Governor g;
  for(const Twist command:std::vector<Twist>{{},{-.18,0,0}}) {
    const auto r=g.evaluate({{.8,0}},{.6,0,0},command,.2,.05,1);
    EXPECT_TRUE(r.measured_stop_unsafe);EXPECT_DOUBLE_EQ(r.output.x,0);
    EXPECT_GT(r.paths[0].back().x,.4);
  }
}
TEST(DynamicSlowdown, BrakingDistanceMatchesConstantDeceleration) {
  Governor g;const auto path=g.trace({.6,0,0},{},.2,0);
  EXPECT_NEAR(path.back().x,.6*.2+.6*.6/(2*.5),1e-9);
  Config weak;weak.linear_braking=.25;Governor slower(weak);
  EXPECT_GT(slower.trace({.6,0,0},{},.2,0).back().x,path.back().x);
}
TEST(DynamicSlowdown, LatencyExpandsBrakingEnvelope) {
  Governor g;
  EXPECT_NEAR(g.trace({.6,0,0},{},.4,0).back().x-g.trace({.6,0,0},{},.2,0).back().x,.12,1e-9);
}
TEST(DynamicSlowdown, RecoveryIsBoundedAndKeepsRequestedCurvature) {
  Governor g;
  double last_v=0,last_w=0;
  for(int i=0;i<50;++i) {
    const auto r=g.evaluate({}, {}, {.6,0,.7},.2,.05,1+i*.05);
    EXPECT_LE(r.output.x-last_v,.4*.05+1e-9);EXPECT_LE(r.output.w-last_w,1.*.05+1e-9);
    EXPECT_NEAR(r.output.w*.6,r.output.x*.7,1e-9);
    last_v=r.output.x;last_w=r.output.w;
  }
  EXPECT_NEAR(last_v,.6,1e-9);
}
TEST(DynamicSlowdown, HazardReductionIsImmediateAndReleaseHeld) {
  Governor g;settle(g,{}, {.6,0,0},{.6,0,0});
  const auto blocked=g.evaluate({{.8,0}},{.6,0,0},{.6,0,0},.2,.05,7);
  EXPECT_DOUBLE_EQ(blocked.output.x,0);
  EXPECT_DOUBLE_EQ(g.evaluate({}, {}, {.6,0,0},.2,.05,7.1).output.x,0);
  const auto released=g.evaluate({}, {}, {.6,0,0},.2,.05,7.2);
  EXPECT_GT(released.output.x,0);EXPECT_LE(released.output.x,.020001);
}
TEST(DynamicSlowdown, NarrowObstacleBetweenSamplesIsDetected) {
  Config c;c.step_time=.05;c.step_distance=.02;Governor g(c);
  const auto r=g.evaluate({{.897,0}},{.6,0,0},{.6,0,0},.2,.05,1);
  EXPECT_TRUE(r.measured_stop_unsafe);
}
TEST(DynamicSlowdown, InvalidMotionAndConfigurationFailClosed) {
  Governor g;
  EXPECT_THROW(g.evaluate({}, {NAN,0,0},{},.2,.05,1),std::invalid_argument);
  EXPECT_THROW(g.evaluate({}, {},{3,0,0},.2,.05,1),std::invalid_argument);
  EXPECT_THROW(g.evaluate({{NAN,0}}, {},{},.2,.05,1),std::invalid_argument);
  Config c;c.linear_braking=0;EXPECT_THROW(Governor invalid(c),std::invalid_argument);
}
TEST(DynamicSlowdown, ObstacleIndexIncludesRectangleInterior) {
  Obstacles obstacles({{.1,.1}});
  EXPECT_TRUE(obstacles.collision({},.375,.225));
  EXPECT_FALSE(obstacles.collision({1,0,0},.375,.225));
}
TEST(DynamicSlowdown, DenseWallCloudRuntime) {
  std::vector<Point> points;
  for(int i=0;i<20000;++i) points.push_back({(i%1000)*.005,(i%2?1.:-1.)*(.4+(i%10)*.005)});
  Governor g;const auto started=std::chrono::steady_clock::now();
  const auto r=settle(g,points,{.6,0,0},{.6,0,0});
  const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()/100;
  RecordProperty("mean_processing_ms",std::to_string(ms));EXPECT_NEAR(r.output.x,.6,1e-9);
}
