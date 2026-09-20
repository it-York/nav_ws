#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <string>
#include <vector>

namespace fastlio_loop {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
struct Pose {
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Matrix4d matrix() const;
  Pose inverse() const;
  Pose operator*(const Pose &other) const;
  static Pose fromMatrix(const Eigen::Matrix4d &matrix);
  bool finite() const;
};
double rotationDistance(const Pose &a, const Pose &b);
struct Edge {
  size_t from = 0, to = 0;
  Pose measurement;
  double translation_sigma = 0.1, rotation_sigma = 0.05;
  bool loop = false;
};
bool optimizeGraph(std::vector<Pose> &poses, const std::vector<Edge> &edges,
                   std::string &reason);
struct RegistrationOptions {
  double leaf = 0.25;
  double coarse_distance = 3.0;
  double inlier_distance = 0.35;
  double min_overlap = 0.65;
  double min_reverse_overlap = 0.30;
  double max_rmse = 0.18;
  double min_observability = 0.0001;
  double max_correction = 6.0;
  double max_rotation = 0.8;
  int min_inliers = 150;
};
struct RegistrationResult {
  bool accepted = false;
  Pose relative;
  double rmse = std::numeric_limits<double>::infinity();
  double overlap = 0, reverse_overlap = 0, observability = 0;
  std::string reason;
};
Cloud::Ptr downsample(const Cloud::ConstPtr &cloud, double leaf);
Cloud::Ptr transform(const Cloud::ConstPtr &cloud, const Pose &pose);
RegistrationResult registerClouds(const Cloud::ConstPtr &source,
  const Cloud::ConstPtr &target, const Pose &initial, const RegistrationOptions &options);

struct Options {
  double keyframe_distance = 0.75, keyframe_angle = 0.25;
  double min_loop_age = 30.0, search_radius = 8.0;
  int min_loop_separation = 25, submap_neighbors = 3, max_candidates = 3;
  int confirmation_count = 2, loop_cooldown = 10, max_keyframes = 2000;
  double confirmation_distance = 0.5, confirmation_angle = 0.12;
  double cloud_max_range = 40.0, map_leaf = 0.10;
  RegistrationOptions registration;
};
struct Keyframe {
  double stamp = 0;
  Pose odom;
  Cloud::Ptr cloud;
};
struct Update {
  bool keyframe = false, loop = false;
  std::string reason;
  RegistrationResult registration;
};
class Backend {
 public:
  explicit Backend(Options options = {});
  Update addFrame(double stamp, const Pose &odom, const Cloud::ConstPtr &cloud);
  Cloud::Ptr buildMap() const;
  // Save a self-contained bundle first, then atomically replace the PCD.
  // Existing destinations are refused unless overwrite was explicitly enabled.
  std::string save(const std::string &filename, bool overwrite = false) const;
  Pose correction() const;
  const std::vector<Keyframe> &keyframes() const {return frames_;}
  const std::vector<Pose> &poses() const {return poses_;}
  const std::vector<Edge> &edges() const {return edges_;}
  size_t loopCount() const;
 private:
  Options options_;
  std::vector<Keyframe> frames_;
  std::vector<Pose> poses_;
  std::vector<Edge> edges_;
  int last_loop_ = -100000;
  struct Pending {int current = -1, reference = -1, count = 0; Pose correction;} pending_;
};
}  // namespace fastlio_loop
