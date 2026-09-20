#include "fastlio_loop/backend.hpp"
#include "static_imu_init.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <unistd.h>
using namespace fastlio_loop;

static Cloud::Ptr room() {
  Cloud::Ptr cloud(new Cloud);
  for(float x=-5;x<=5;x+=0.18f) for(float y=-4;y<=4;y+=0.18f)
    cloud->push_back(pcl::PointXYZ(x,y,-1.2f));
  for(float x=-5;x<=5;x+=0.18f) for(float z=-1.2;z<=3;z+=0.18f) {
    cloud->push_back(pcl::PointXYZ(x,-4,z));cloud->push_back(pcl::PointXYZ(x,4,z));
  }
  for(float y=-4;y<=4;y+=0.18f) for(float z=-1.2;z<=3;z+=0.18f)
    cloud->push_back(pcl::PointXYZ(-5,y,z));
  for(float y=-1;y<1;y+=0.15f) for(float z=-1.2;z<1;z+=0.15f)
    cloud->push_back(pcl::PointXYZ(1,y,z));
  return cloud;
}
TEST(Pose, CompositionAndInverse) {
  Pose a,b;a.t={1,2,3};a.q=Eigen::AngleAxisd(0.7,Eigen::Vector3d::UnitZ());b.t={2,-1,4};
  EXPECT_LT(((a*b).matrix()-a.matrix()*b.matrix()).norm(),1e-12);
  EXPECT_LT((a.inverse()*a).matrix().operator-(Eigen::Matrix4d::Identity()).norm(),1e-12);
}
TEST(Graph, ClosesDriftAndMovesHistory) {
  std::vector<Pose> poses(41);std::vector<Edge> edges;
  for(size_t i=0;i<poses.size();++i) {
    double angle=2*M_PI*i/40;
    poses[i].t={5*(1-std::cos(angle))+0.025*i,5*std::sin(angle),-0.015*i};
    poses[i].q=Eigen::AngleAxisd(0.002*i,Eigen::Vector3d::UnitZ());
    if(i) {Edge e;e.from=i-1;e.to=i;e.measurement=poses[i-1].inverse()*poses[i];edges.push_back(e);}
  }
  auto original=poses;Edge loop;loop.from=0;loop.to=40;loop.loop=true;loop.translation_sigma=0.02;edges.push_back(loop);
  std::string reason;ASSERT_TRUE(optimizeGraph(poses,edges,reason))<<reason;
  EXPECT_LT(poses.back().t.norm(),0.06);
  EXPECT_LT(rotationDistance(poses.back(),Pose{}),0.01);
  EXPECT_GT((poses[20].t-original[20].t).norm(),0.1);
  EXPECT_LT((poses[0].matrix()-original[0].matrix()).norm(),1e-12);
}
TEST(Graph, RejectsInvalidEdge) {
  std::vector<Pose> poses(2);Edge edge;edge.to=5;std::string reason;
  EXPECT_FALSE(optimizeGraph(poses,{edge},reason));EXPECT_TRUE(poses[0].finite());
}
TEST(Registration, RecoversSpatialTransform) {
  auto target=room();Pose truth;truth.t={0.4,-0.25,0.12};
  truth.q=Eigen::AngleAxisd(0.08,Eigen::Vector3d(0.3,0.4,1).normalized());
  auto source=transform(target,truth.inverse());Pose initial=truth;initial.t+=Eigen::Vector3d(0.2,-0.1,0.15);
  auto result=registerClouds(source,target,initial,{});
  ASSERT_TRUE(result.accepted)<<result.reason<<" observability="<<result.observability;
  EXPECT_LT((result.relative.t-truth.t).norm(),0.07);EXPECT_LT(rotationDistance(result.relative,truth),0.02);
}
TEST(Registration, RejectsPlanarAmbiguity) {
  Cloud::Ptr plane(new Cloud);
  for(float x=-5;x<5;x+=0.15f) for(float y=-5;y<5;y+=0.15f) plane->push_back(pcl::PointXYZ(x,y,0));
  auto r=registerClouds(plane,plane,Pose{},{});
  EXPECT_FALSE(r.accepted);EXPECT_EQ(r.reason,"geometrically degenerate closure");
}
TEST(Registration, RejectsExcessiveCorrection) {
  auto cloud=room();Pose initial;initial.t.x()=0.5;RegistrationOptions options;options.max_correction=0.05;
  EXPECT_FALSE(registerClouds(cloud,cloud,initial,options).accepted);
}
TEST(Backend, ConfirmsLoopsRebuildsAndSaves) {
  Options o;o.keyframe_distance=0.15;o.min_loop_age=5;o.min_loop_separation=10;
  o.search_radius=1.6;o.submap_neighbors=0;o.loop_cooldown=5;
  Backend backend(o);auto world=room();bool pending_seen=false;
  for(int i=0;i<=42;++i) {
    double angle=2*M_PI*i/40;Pose truth;truth.t={2*(1-std::cos(angle)),2*std::sin(angle),0};
    Pose odom=truth;odom.t+=Eigen::Vector3d(0.012*i,0,-0.007*i);
    auto u=backend.addFrame(10+i,odom,transform(world,truth.inverse()));
    ASSERT_TRUE(u.keyframe)<<i<<": "<<u.reason;
    if(u.reason.find("waiting for independent")!=std::string::npos) pending_seen=true;
    if(u.loop) {
      ASSERT_TRUE(pending_seen);
      std::cout<<"Loop at frame "<<i<<": RMSE="<<u.registration.rmse
        <<", position error="<<(backend.poses().back().t-truth.t).norm()
        <<", optimized="<<backend.poses().back().t.transpose()<<std::endl;
    }
  }
  ASSERT_GT(backend.loopCount(),0u);EXPECT_GT(backend.correction().t.norm(),0.15);
  const double last_angle=2*M_PI*42/40;
  const Eigen::Vector3d last_truth(2*(1-std::cos(last_angle)),2*std::sin(last_angle),0);
  EXPECT_LT((backend.poses().back().t-last_truth).norm(),0.25);
  ASSERT_FALSE(backend.buildMap()->empty());
  auto folder=std::filesystem::temp_directory_path()/("fastlio_loop_test_"+std::to_string(getpid()));
  std::filesystem::create_directory(folder);auto file=(folder/"test.pcd").string();
  EXPECT_NO_THROW(backend.save(file));
  Cloud saved;ASSERT_EQ(pcl::io::loadPCDFile(file,saved),0);EXPECT_EQ(saved.size(),backend.buildMap()->size());
  size_t bundles=0;for(const auto &entry:std::filesystem::directory_iterator(folder)) if(entry.is_directory()) {
    ++bundles;EXPECT_TRUE(std::filesystem::exists(entry.path()/"poses.csv"));EXPECT_TRUE(std::filesystem::exists(entry.path()/"edges.csv"));
  }
  EXPECT_EQ(bundles,1u);EXPECT_THROW(backend.save(file),std::runtime_error);
  std::filesystem::remove_all(folder);
}
TEST(Backend, RejectsEmptyAndOldData) {
  Backend backend;EXPECT_THROW(backend.save("/tmp/unused_loop_map.pcd"),std::runtime_error);
  ASSERT_TRUE(backend.addFrame(2,Pose{},room()).keyframe);
  Pose moved;moved.t.x()=2;EXPECT_FALSE(backend.addFrame(1,moved,room()).keyframe);
  EXPECT_EQ(backend.keyframes().size(),1u);
}
TEST(Initialization, RequiresContinuousStationaryWindow) {
  StaticImuInit gate;gate.configure(2,0.1,0.25);
  for(int i=0;i<150;++i) EXPECT_FALSE(gate.add(i*0.01,{0,0,1},{0,0,0}));
  EXPECT_FALSE(gate.add(1.5,{0,0,1},{0,0,0.2}));EXPECT_EQ(gate.count(),0u);
  for(int i=151;i<351;++i) EXPECT_FALSE(gate.add(i*0.01,{0,0,1},{0,0,0}));
  EXPECT_TRUE(gate.add(3.52,{0,0,1},{0,0,0}));
}
TEST(Initialization, RejectsAccelerationVibrationAndInvalidInput) {
  StaticImuInit gate;gate.configure(2,0.1,0.25);
  for(int i=0;i<250;++i) EXPECT_FALSE(gate.add(i*0.01,{i%2?0.2:-0.2,0,1},{0,0,0}));
  EXPECT_FALSE(gate.add(3,{NAN,0,1},{0,0,0}));EXPECT_EQ(gate.count(),0u);
}
