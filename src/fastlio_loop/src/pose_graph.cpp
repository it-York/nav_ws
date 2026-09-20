#include "fastlio_loop/backend.hpp"
#include <ceres/ceres.h>
#include <cmath>

namespace fastlio_loop {
Eigen::Matrix4d Pose::matrix() const {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3,3>(0,0) = q.toRotationMatrix(); m.block<3,1>(0,3) = t; return m;
}
Pose Pose::inverse() const {Pose p; p.q=q.conjugate(); p.t=-(p.q*t); return p;}
Pose Pose::operator*(const Pose &b) const {Pose p; p.q=(q*b.q).normalized(); p.t=t+q*b.t; return p;}
Pose Pose::fromMatrix(const Eigen::Matrix4d &m) {
  Pose p; p.t=m.block<3,1>(0,3); p.q=Eigen::Quaterniond(m.block<3,3>(0,0)).normalized(); return p;
}
bool Pose::finite() const {return t.allFinite() && q.coeffs().allFinite() && std::abs(q.norm()-1)<1e-5;}
double rotationDistance(const Pose &a,const Pose &b) {return a.q.angularDistance(b.q);}
namespace {
struct RelativeResidual {
  explicit RelativeResidual(const Edge &e): edge(e) {}
  template<typename T> bool operator()(const T *ta,const T *qa,const T *tb,const T *qb,T *out) const {
    Eigen::Map<const Eigen::Matrix<T,3,1>> a(ta), b(tb);
    Eigen::Map<const Eigen::Quaternion<T>> ra(qa), rb(qb);
    const Eigen::Matrix<T,3,1> translation=ra.conjugate()*(b-a);
    Eigen::Quaternion<T> error=edge.measurement.q.cast<T>().conjugate()*(ra.conjugate()*rb);
    if(error.w()<T(0)) error.coeffs() *= T(-1);
    for(int i=0;i<3;++i) {
      out[i]=(translation[i]-T(edge.measurement.t[i]))/T(edge.translation_sigma);
      out[i+3]=T(2)*error.vec()[i]/T(edge.rotation_sigma);
    }
    return true;
  }
  Edge edge;
};
}
bool optimizeGraph(std::vector<Pose> &poses,const std::vector<Edge> &edges,std::string &reason) {
  if(poses.empty()) {reason="empty graph";return false;}
  for(const auto &p:poses) if(!p.finite()) {reason="invalid pose";return false;}
  for(const auto &e:edges) {
    if(e.from>=poses.size() || e.to>=poses.size() || e.from==e.to || !e.measurement.finite() ||
      !std::isfinite(e.translation_sigma) || !std::isfinite(e.rotation_sigma) ||
      e.translation_sigma<=0 || e.rotation_sigma<=0) {reason="invalid edge";return false;}
  }
  auto original=poses;
  ceres::Problem problem;
  for(auto &p:poses) {
    problem.AddParameterBlock(p.t.data(),3);
    problem.AddParameterBlock(p.q.coeffs().data(),4,new ceres::EigenQuaternionParameterization());
  }
  for(const auto &e:edges) {
    auto *cost=new ceres::AutoDiffCostFunction<RelativeResidual,6,3,4,3,4>(new RelativeResidual(e));
    problem.AddResidualBlock(cost,e.loop?new ceres::HuberLoss(2.0):nullptr,
      poses[e.from].t.data(),poses[e.from].q.coeffs().data(),
      poses[e.to].t.data(),poses[e.to].q.coeffs().data());
  }
  problem.SetParameterBlockConstant(poses.front().t.data());
  problem.SetParameterBlockConstant(poses.front().q.coeffs().data());
  ceres::Solver::Options options;
  options.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations=40; options.num_threads=1;
  options.max_solver_time_in_seconds=3.0;
  ceres::Solver::Summary summary; ceres::Solve(options,&problem,&summary);
  bool valid=summary.IsSolutionUsable();
  for(auto &p:poses) {p.q.normalize();valid=valid && p.finite();}
  // An incorrect closure must not be allowed to tear apart adjacent frames.
  for(const auto &e:edges) {
    const Pose relative=poses[e.from].inverse()*poses[e.to];
    if(!e.loop && ((relative.t-e.measurement.t).norm()>0.5 || rotationDistance(relative,e.measurement)>0.25)) valid=false;
  }
  if(!valid) {poses=original;reason="graph solution failed consistency checks";return false;}
  reason=summary.BriefReport();return true;
}
}  // namespace fastlio_loop
