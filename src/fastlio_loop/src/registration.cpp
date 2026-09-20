#include "fastlio_loop/backend.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <cmath>

namespace fastlio_loop {
namespace {
// Point-to-point ICP can lock onto the sampling grid along walls. Refine its
// basin using surface distances, which do not penalize tangential sampling.
Pose refinePlanes(const Cloud::ConstPtr &source, const Cloud::ConstPtr &target,
                  const pcl::PointCloud<pcl::Normal> &normals,
                  pcl::KdTreeFLANN<pcl::PointXYZ> &tree, Pose pose, double distance) {
  for(int iteration=0;iteration<15;++iteration) {
    auto aligned=transform(source,pose);
    Eigen::Vector3d centre=Eigen::Vector3d::Zero();
    for(const auto &p:*aligned) centre+=p.getVector3fMap().cast<double>();
    centre/=aligned->size();
    Eigen::Matrix<double,6,6> h=Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,1> b=Eigen::Matrix<double,6,1>::Zero();
    std::vector<int> ids(1);std::vector<float> distances(1);size_t count=0;
    for(const auto &point:*aligned) {
      if(tree.nearestKSearch(point,1,ids,distances)<1 || distances[0]>distance*distance) continue;
      const auto &normal=normals[ids[0]];
      const Eigen::Vector3d n(normal.normal_x,normal.normal_y,normal.normal_z);
      if(!n.allFinite() || normal.curvature>0.1) continue;
      const Eigen::Vector3d p=point.getVector3fMap().cast<double>();
      const double error=n.dot(p-(*target)[ids[0]].getVector3fMap().cast<double>());
      const double weight=std::abs(error)>0.1?0.1/std::abs(error):1.0;
      Eigen::Matrix<double,6,1> j;j.head<3>()=n;j.tail<3>()=(p-centre).cross(n);
      h.noalias()+=weight*j*j.transpose();b.noalias()+=weight*j*error;++count;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigen(h);
    if(count<50 || eigen.info()!=Eigen::Success || eigen.eigenvalues()[0]<1e-8*eigen.eigenvalues()[5]) break;
    Eigen::Matrix<double,6,1> delta=-h.ldlt().solve(b);
    if(!delta.allFinite()) break;
    const double scale=std::max({1.0,delta.head<3>().norm()/0.5,delta.tail<3>().norm()/0.15});
    delta/=scale;
    Pose increment;const double angle=delta.tail<3>().norm();
    if(angle>1e-12) increment.q=Eigen::AngleAxisd(angle,delta.tail<3>()/angle);
    increment.t=centre-increment.q*centre+delta.head<3>();pose=increment*pose;
    if(delta.norm()<1e-5) break;
  }
  return pose;
}
}  // namespace
Cloud::Ptr downsample(const Cloud::ConstPtr &cloud,double leaf) {
  Cloud::Ptr out(new Cloud);
  if(cloud->empty()) return out;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);filter.setLeafSize(leaf,leaf,leaf);filter.filter(*out);
  return out;
}
Cloud::Ptr transform(const Cloud::ConstPtr &cloud,const Pose &pose) {
  Cloud::Ptr out(new Cloud);pcl::transformPointCloud(*cloud,*out,pose.matrix().cast<float>());return out;
}
RegistrationResult registerClouds(const Cloud::ConstPtr &source,const Cloud::ConstPtr &target,
                                 const Pose &initial,const RegistrationOptions &o) {
  RegistrationResult result;result.relative=initial;
  if(!initial.finite() || source->size()<static_cast<size_t>(o.min_inliers) ||
    target->size()<static_cast<size_t>(o.min_inliers)) {result.reason="insufficient points";return result;}
  Eigen::Matrix4f guess=initial.matrix().cast<float>();
  for(int stage=0;stage<2;++stage) {
    auto src=downsample(source,o.leaf*(stage==0?2.0:1.0));
    auto dst=downsample(target,o.leaf*(stage==0?2.0:1.0));
    if(src->size()<20 || dst->size()<20) {result.reason="sparse registration cloud";return result;}
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    icp.setInputSource(src);icp.setInputTarget(dst);
    icp.setMaximumIterations(stage==0?35:50);
    icp.setMaxCorrespondenceDistance(stage==0?o.coarse_distance:o.inlier_distance*2);
    icp.setTransformationEpsilon(1e-7);icp.setEuclideanFitnessEpsilon(1e-6);
    Cloud aligned;icp.align(aligned,guess);
    if(!icp.hasConverged()) {result.reason="ICP did not converge";return result;}
    guess=icp.getFinalTransformation();
  }
  if(!guess.allFinite()) {result.reason="nonfinite ICP transform";return result;}
  result.relative=Pose::fromMatrix(guess.cast<double>());
  auto dst=downsample(target,o.leaf);
  auto local_source=downsample(source,o.leaf);
  pcl::KdTreeFLANN<pcl::PointXYZ> target_tree,source_tree;
  target_tree.setInputCloud(dst);
  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> estimation;
  estimation.setInputCloud(dst);estimation.setKSearch(15);
  pcl::PointCloud<pcl::Normal> normals;estimation.compute(normals);
  result.relative=refinePlanes(local_source,dst,normals,target_tree,result.relative,o.inlier_distance);
  if((result.relative.t-initial.t).norm()>o.max_correction ||
    rotationDistance(result.relative,initial)>o.max_rotation) {
    result.reason="correction exceeds configured limits";return result;
  }
  auto src=transform(local_source,result.relative);
  source_tree.setInputCloud(src);
  std::vector<int> matches(src->size(),-1),ids(1);std::vector<float> distances(1);
  size_t inliers=0,reverse=0;double squared_sum=0;
  const double limit=o.inlier_distance*o.inlier_distance;
  for(size_t i=0;i<src->size();++i) {
    if(target_tree.nearestKSearch((*src)[i],1,ids,distances)>0 && distances[0]<limit) {
      matches[i]=ids[0];++inliers;squared_sum+=distances[0];
    }
  }
  for(const auto &point:*dst) {
    if(source_tree.nearestKSearch(point,1,ids,distances)>0 && distances[0]<limit) ++reverse;
  }
  result.overlap=static_cast<double>(inliers)/src->size();
  result.reverse_overlap=static_cast<double>(reverse)/dst->size();
  if(inliers) result.rmse=std::sqrt(squared_sum/inliers);
  if(inliers<static_cast<size_t>(o.min_inliers) || result.overlap<o.min_overlap ||
    result.reverse_overlap<o.min_reverse_overlap || result.rmse>o.max_rmse) {
    result.reason="overlap or RMSE check failed";return result;
  }
  // Point-to-plane information reveals directions unconstrained by a flat
  // wall/floor or featureless corridor, even if point-to-point ICP converges.
  Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Zero();
  Eigen::Vector3d centre=Eigen::Vector3d::Zero();
  for(const auto &p:*src) centre+=p.getVector3fMap().cast<double>();
  centre/=src->size();
  for(size_t i=0;i<src->size();++i) {
    if(matches[i]<0) continue;
    const auto &normal=normals[matches[i]];
    Eigen::Vector3d n(normal.normal_x,normal.normal_y,normal.normal_z);
    if(!n.allFinite() || normal.curvature>0.15) continue;
    const Eigen::Vector3d p=(*src)[i].getVector3fMap().cast<double>()-centre;
    Eigen::Matrix<double,6,1> jacobian;
    jacobian.head<3>()=n;jacobian.tail<3>()=p.cross(n)/10.0;
    information.noalias()+=jacobian*jacobian.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigen(information);
  if(eigen.info()==Eigen::Success && eigen.eigenvalues()[5]>0) {
    result.observability=eigen.eigenvalues()[0]/eigen.eigenvalues()[5];
  }
  if(result.observability<o.min_observability) {
    result.reason="geometrically degenerate closure";return result;
  }
  result.accepted=true;result.reason="geometric checks passed";return result;
}
}  // namespace fastlio_loop
