#include "fastlio_loop/backend.hpp"
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fastlio_loop {
Backend::Backend(Options options):options_(options) {
  const auto &o=options_;
  const auto &r=o.registration;
  for(double value:{o.keyframe_distance,o.keyframe_angle,o.min_loop_age,o.search_radius,
      o.confirmation_distance,o.confirmation_angle,o.cloud_max_range,o.map_leaf,
      r.leaf,r.coarse_distance,r.inlier_distance,r.min_overlap,r.min_reverse_overlap,
      r.max_rmse,r.min_observability,r.max_correction,r.max_rotation}) {
    if(!std::isfinite(value) || value<=0) throw std::invalid_argument("backend parameters must be finite and positive");
  }
  if(o.min_loop_separation<2 || o.submap_neighbors<0 || o.max_candidates<1 ||
      o.confirmation_count<2 || o.loop_cooldown<1 || o.max_keyframes<2 ||
      r.min_inliers<10 || r.min_overlap>1 || r.min_reverse_overlap>1 || r.min_observability>=1)
    throw std::invalid_argument("invalid backend counts or ratios");
}
Pose Backend::correction() const {
  return frames_.empty()?Pose{}:poses_.back()*frames_.back().odom.inverse();
}
size_t Backend::loopCount() const {
  return std::count_if(edges_.begin(),edges_.end(),[](const Edge &e){return e.loop;});
}
Update Backend::addFrame(double stamp,const Pose &odom,const Cloud::ConstPtr &input) {
  Update update;
  if(!std::isfinite(stamp) || !odom.finite()) {update.reason="invalid timestamp or pose";return update;}
  if(!frames_.empty()) {
    if(stamp<=frames_.back().stamp) {update.reason="nonmonotonic timestamp; restart for a new recording";return update;}
    if((odom.t-frames_.back().odom.t).norm()<options_.keyframe_distance &&
       rotationDistance(odom,frames_.back().odom)<options_.keyframe_angle) {
      update.reason="below keyframe motion threshold";return update;
    }
  }
  if(frames_.size()>=static_cast<size_t>(options_.max_keyframes)) {
    update.reason="keyframe limit reached; save and start a new mapping session";return update;
  }
  Cloud::Ptr clean(new Cloud);
  for(const auto &p:*input) {
    if(p.getVector3fMap().allFinite() && p.getVector3fMap().norm()<options_.cloud_max_range) clean->push_back(p);
  }
  auto cloud=downsample(clean,options_.registration.leaf);
  if(cloud->size()<static_cast<size_t>(options_.registration.min_inliers)) {
    update.reason="too few usable keyframe points";return update;
  }
  // Bound keyframe memory while retaining spatial coverage after voxelization.
  if(cloud->size()>15000) {
    Cloud::Ptr limited(new Cloud);
    const size_t stride=(cloud->size()+14999)/15000;
    for(size_t i=0;i<cloud->size();i+=stride) limited->push_back((*cloud)[i]);
    cloud=limited;
  }
  const Pose corrected=correction()*odom;
  const size_t current=frames_.size();
  if(current) {
    Edge edge;edge.from=current-1;edge.to=current;
    edge.measurement=frames_.back().odom.inverse()*odom;
    edge.translation_sigma=0.04+0.025*edge.measurement.t.norm();
    edge.rotation_sigma=0.015+0.02*rotationDistance(Pose{},edge.measurement);
    edges_.push_back(edge);
  }
  frames_.push_back({stamp,odom,cloud});poses_.push_back(corrected);
  update.keyframe=true;update.reason="keyframe added; no eligible loop";
  if(static_cast<int>(current)-last_loop_<options_.loop_cooldown) return update;
  std::vector<std::pair<double,size_t>> candidates;
  for(size_t i=0;i<current;++i) {
    if(current-i<static_cast<size_t>(options_.min_loop_separation) ||
       stamp-frames_[i].stamp<options_.min_loop_age) continue;
    // XY search tolerates vertical drift on the user's single-floor route.
    const double distance=(poses_[i].t-poses_.back().t).head<2>().norm();
    if(distance<options_.search_radius) candidates.emplace_back(distance,i);
  }
  std::sort(candidates.begin(),candidates.end());
  std::vector<size_t> tried;
  for(const auto &[distance,reference]:candidates) {
    (void)distance;
    bool redundant=false;
    for(size_t previous:tried) if(std::abs(static_cast<int>(previous)-static_cast<int>(reference))<=options_.submap_neighbors) redundant=true;
    if(redundant) continue;
    if(tried.size()>=static_cast<size_t>(options_.max_candidates)) break;
    tried.push_back(reference);
    Cloud::Ptr target(new Cloud);
    const int begin=std::max(0,static_cast<int>(reference)-options_.submap_neighbors);
    const int end=std::min(static_cast<int>(current)-options_.min_loop_separation,
                          static_cast<int>(reference)+options_.submap_neighbors);
    for(int j=begin;j<=end;++j) {
      if(stamp-frames_[j].stamp<options_.min_loop_age) continue;
      *target+=*transform(frames_[j].cloud,poses_[reference].inverse()*poses_[j]);
    }
    auto result=registerClouds(cloud,target,poses_[reference].inverse()*poses_.back(),options_.registration);
    update.registration=result;update.reason=result.reason;
    if(!result.accepted) continue;
    const Pose proposed_correction=poses_[reference]*result.relative*odom.inverse();
    const bool consistent=pending_.current>=0 && static_cast<int>(current)-pending_.current<=3 &&
      std::abs(static_cast<int>(reference)-pending_.reference)<=options_.submap_neighbors*2+3 &&
      (proposed_correction.t-pending_.correction.t).norm()<options_.confirmation_distance &&
      rotationDistance(proposed_correction,pending_.correction)<options_.confirmation_angle;
    pending_.count=consistent?pending_.count+1:1;
    pending_.current=current;pending_.reference=reference;pending_.correction=proposed_correction;
    if(pending_.count<options_.confirmation_count) {
      update.reason="valid candidate; waiting for independent keyframe confirmation";return update;
    }
    Edge edge;edge.from=reference;edge.to=current;edge.measurement=result.relative;
    edge.translation_sigma=std::max(0.04,result.rmse);edge.rotation_sigma=0.035;edge.loop=true;
    auto backup=poses_;edges_.push_back(edge);
    std::string reason;
    bool valid=optimizeGraph(poses_,edges_,reason);
    const Pose closed=poses_[reference].inverse()*poses_[current];
    valid=valid && (closed.t-edge.measurement.t).norm()<0.30 && rotationDistance(closed,edge.measurement)<0.10;
    if(!valid) {
      poses_=backup;edges_.pop_back();pending_={};
      update.reason="loop rejected by pose-graph consistency check";return update;
    }
    last_loop_=current;pending_={};update.loop=true;update.reason="loop accepted and historical poses optimized";
    return update;
  }
  return update;
}
Cloud::Ptr Backend::buildMap() const {
  Cloud::Ptr map(new Cloud);
  for(size_t i=0;i<frames_.size();++i) {
    *map+=*transform(frames_[i].cloud,poses_[i]);
    if(i%30==29) map=downsample(map,options_.map_leaf);
  }
  return downsample(map,options_.map_leaf);
}
std::string Backend::save(const std::string &filename,bool overwrite) const {
  namespace fs=std::filesystem;
  if(frames_.empty()) throw std::runtime_error("No keyframes received; map not saved");
  const fs::path path(filename);
  if(path.extension()!=".pcd") throw std::runtime_error("Map filename must end in .pcd");
  if(fs::exists(path) && !overwrite) throw std::runtime_error("Map exists; choose a new map_path or explicitly enable overwrite_map");
  if(!path.parent_path().empty()) fs::create_directories(path.parent_path());
  auto map=buildMap();
  if(map->empty()) throw std::runtime_error("Optimized map is empty");
  const std::string suffix=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const fs::path bundle(path.string()+".keyframes_"+suffix);
  fs::create_directory(bundle);
  std::ofstream poses(bundle/"poses.csv"),edges(bundle/"edges.csv");
  poses.exceptions(std::ios::badbit|std::ios::failbit);edges.exceptions(std::ios::badbit|std::ios::failbit);
  poses<<std::setprecision(17)<<"id,stamp,odom_x,odom_y,odom_z,odom_qx,odom_qy,odom_qz,odom_qw,map_x,map_y,map_z,map_qx,map_qy,map_qz,map_qw\n";
  auto write_pose=[](std::ostream &s,const Pose &p) {
    s<<','<<p.t.x()<<','<<p.t.y()<<','<<p.t.z()<<','<<p.q.x()<<','<<p.q.y()<<','<<p.q.z()<<','<<p.q.w();
  };
  for(size_t i=0;i<frames_.size();++i) {
    if(pcl::io::savePCDFileBinary((bundle/(std::to_string(i)+".pcd")).string(),*frames_[i].cloud)!=0)
      throw std::runtime_error("Failed to save keyframe cloud");
    poses<<i<<','<<frames_[i].stamp;write_pose(poses,frames_[i].odom);write_pose(poses,poses_[i]);poses<<'\n';
  }
  edges<<std::setprecision(17)<<"from,to,x,y,z,qx,qy,qz,qw,translation_sigma,rotation_sigma,loop\n";
  for(const auto &e:edges_) {
    edges<<e.from<<','<<e.to;write_pose(edges,e.measurement);
    edges<<','<<e.translation_sigma<<','<<e.rotation_sigma<<','<<e.loop<<'\n';
  }
  poses.close();edges.close();
  const fs::path temporary(path.string()+".tmp_"+suffix);
  if(pcl::io::savePCDFileBinary(temporary.string(),*map)!=0) throw std::runtime_error("Failed to write map");
  if(overwrite) fs::rename(temporary,path);
  else { // Atomic no-clobber publication on the same filesystem.
    try {fs::create_hard_link(temporary,path);}
    catch(...) {fs::remove(temporary);throw;}
    fs::remove(temporary);
  }
  std::ostringstream result;
  result<<"Saved "<<path<<"; keyframes="<<frames_.size()<<"; accepted_loops="<<loopCount()
    <<"; bundle="<<bundle;
  if(!loopCount()) result<<"; WARNING: no loop accepted, map still contains odometry drift";
  return result.str();
}
}  // namespace fastlio_loop
