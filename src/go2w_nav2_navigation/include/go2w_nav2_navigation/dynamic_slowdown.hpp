#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace go2w_nav2_navigation::dynamic_slowdown {
struct Point { double x, y; };
struct Twist { double x{0}, y{0}, w{0}; };
struct Pose { double x{0}, y{0}, yaw{0}; };
inline double speed(const Twist & v) { return std::hypot(v.x,v.y); }
inline Twist scaled(const Twist & v,double s) { return {v.x*s,v.y*s,v.w*s}; }
struct Config {
  double length{.75}, width{.45}, margin{.04};
  double reaction_time{.20}, preview_time{.60};
  // Assumed conservative limits; must be verified against the physical robot.
  double linear_braking{.50}, angular_braking{.80};
  double linear_acceleration{.40}, angular_acceleration{1.0};
  double recovery_acceleration{.40}, recovery_angular_acceleration{1.0};
  double release_hold{.15}, step_time{.04}, step_distance{.01};
  double max_linear_speed{1.5}, max_angular_speed{2.0};
  int scale_samples{16}, refinement_steps{5};
  void validate() const {
    for(double v:{length,width,reaction_time,preview_time,linear_braking,angular_braking,
        linear_acceleration,angular_acceleration,recovery_acceleration,recovery_angular_acceleration,
        step_time,step_distance,max_linear_speed,max_angular_speed})
      if(!std::isfinite(v)||v<=0) throw std::invalid_argument("dynamic slowdown limits must be finite and positive");
    if(!std::isfinite(margin)||margin<0 || !std::isfinite(release_hold)||release_hold<0 ||
       length>3 || width>3 || margin>.3 || preview_time>2 || reaction_time>1 ||
       linear_braking<.1 || angular_braking<.1 || step_time>.05 || step_time<.005 ||
       step_distance>.02 || step_distance<.005 || max_linear_speed>2 || max_angular_speed>3 ||
       scale_samples<4 || scale_samples>32 || refinement_steps<0 || refinement_steps>8)
      throw std::invalid_argument("dynamic slowdown configuration outside supported numerical budgets");
  }
};
// Integrate a body-fixed twist exactly over one constant-velocity interval.
inline Pose advance(Pose p,const Twist & v,double dt) {
  double x,y;
  if(std::abs(v.w)<1e-8) {x=v.x*dt;y=v.y*dt;}
  else {
    const double s=std::sin(v.w*dt),c=std::cos(v.w*dt);
    x=(v.x*s+v.y*(c-1))/v.w;y=(v.x*(1-c)+v.y*s)/v.w;
  }
  const double c=std::cos(p.yaw),s=std::sin(p.yaw);
  p.x+=c*x-s*y;p.y+=s*x+c*y;p.yaw+=v.w*dt;return p;
}
class Obstacles {
public:
  explicit Obstacles(const std::vector<Point> & points) {
    for(const auto & p:points) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)) throw std::invalid_argument("nonfinite obstacle");
      // Such remote points cannot enter the supported short-horizon envelope.
      if(std::abs(p.x)>100 || std::abs(p.y)>100) continue;
      cells_[key(cell(p.x),cell(p.y))].push_back(p);
    }
  }
  bool collision(const Pose & p,double half_length,double half_width) const {
    const double c=std::cos(p.yaw),s=std::sin(p.yaw);
    const double dx=std::abs(c)*half_length+std::abs(s)*half_width;
    const double dy=std::abs(s)*half_length+std::abs(c)*half_width;
    for(int i=cell(p.x-dx);i<=cell(p.x+dx);++i)
      for(int j=cell(p.y-dy);j<=cell(p.y+dy);++j) {
        const auto it=cells_.find(key(i,j));if(it==cells_.end()) continue;
        for(const auto & q:it->second) {
          const double x=q.x-p.x,y=q.y-p.y;
          if(std::abs(c*x+s*y)<=half_length && std::abs(-s*x+c*y)<=half_width) return true;
        }
      }
    return false;
  }
private:
  static int cell(double x) {return static_cast<int>(std::floor(x/.1));}
  static uint64_t key(int x,int y) {return (uint64_t(uint32_t(x))<<32)|uint32_t(y);}
  std::unordered_map<uint64_t,std::vector<Point>> cells_;
};
struct Result {
  Twist output;
  double scale{0};
  bool measured_stop_unsafe{false}, limited{false};
  std::vector<std::vector<Pose>> paths;
};
class Governor {
public:
  explicit Governor(Config config=Config()) : c_(config) {c_.validate();}
  void reset(double now) {previous_={};hold_until_=now+c_.release_hold;}
  const Config & config() const {return c_;}
  Result evaluate(const std::vector<Point> & points,const Twist & measured,
    const Twist & requested,double reaction,double dt,double now) {
    validTwist(measured);validTwist(requested);
    if(!std::isfinite(reaction)||reaction<c_.reaction_time || reaction>1.5 ||
       !std::isfinite(dt)||dt<=0 || !std::isfinite(now)) throw std::invalid_argument("invalid model timing");
    dt=std::min(dt,.1); // Never allow a large recovery jump after scheduling stalls.
    Obstacles obstacles(points);
    Result out;
    // Show all three physical hypotheses, using the unsuppressed target.
    out.paths={trace(measured,{},reaction,0),trace(measured,requested,reaction,1),trace(measured,requested,reaction,2)};
    if(!safe(out.paths[0],obstacles)) {
      out.measured_stop_unsafe=true;out.limited=true;reset(now);return out;
    }
    double cap=1.0;
    const double add=now>=hold_until_?dt:0.;
    const double requested_speed=speed(requested);
    if(requested_speed>1e-6) {
      const double previous_projection=std::max(0.,(previous_.x*requested.x+previous_.y*requested.y)/requested_speed);
      cap=std::min(cap,(previous_projection+c_.recovery_acceleration*add)/requested_speed);
    }
    if(std::abs(requested.w)>1e-6) {
      const double previous_w=previous_.w*requested.w>0?std::abs(previous_.w):0.;
      cap=std::min(cap,(previous_w+c_.recovery_angular_acceleration*add)/std::abs(requested.w));
    }
    cap=std::clamp(cap,0.,1.);
    auto admissible=[&](double scale) {
      const auto target=scaled(requested,scale);
      return safe(trace(measured,target,reaction,1),obstacles) &&
             safe(trace(measured,target,reaction,2),obstacles);
    };
    double chosen=cap;
    if(!admissible(cap)) {
      out.limited=true;bool found=false;double upper=cap;
      // Curved transition feasibility need not be monotonic. Search descending
      // candidates, then refine only the found boundary. Every output is checked.
      for(int i=1;i<=c_.scale_samples;++i) {
        const double trial=cap*(1.-double(i)/c_.scale_samples);
        if(admissible(trial)) {chosen=trial;found=true;break;}
        upper=trial;
      }
      if(!found) {reset(now);return out;}
      for(int i=0;i<c_.refinement_steps;++i) {
        const double trial=.5*(chosen+upper);
        if(admissible(trial)) chosen=trial; else upper=trial;
      }
      hold_until_=now+c_.release_hold;
    }
    out.scale=chosen;out.output=scaled(requested,chosen);previous_=out.output;
    return out;
  }
  // mode 0: measured inertia + brake. 1: target applied after reaction + brake.
  // mode 2: acceleration-limited transition to target + brake. All modes start
  // with the measured twist throughout reaction; command=0 never erases inertia.
  std::vector<Pose> trace(const Twist & measured,const Twist & target,double reaction,int mode) const {
    std::vector<Pose> poses(1);Pose pose;Twist velocity=measured;
    auto interval=[&](Twist begin,Twist end,double duration) {
      if(duration<=0) return;
      const double r=std::hypot(c_.length*.5+c_.margin,c_.width*.5+c_.margin);
      const double bound=std::max(speed(begin),speed(end))+r*std::max(std::abs(begin.w),std::abs(end.w));
      const double step=std::min(c_.step_time,c_.step_distance/std::max(bound,1e-6));
      const int n=std::max(1,int(std::ceil(duration/step)));
      for(int i=0;i<n;++i) {
        const double a=(i+.5)/n;
        const Twist v{begin.x+(end.x-begin.x)*a,begin.y+(end.y-begin.y)*a,begin.w+(end.w-begin.w)*a};
        pose=advance(pose,v,duration/n);poses.push_back(pose);
      }
    };
    interval(velocity,velocity,reaction);
    if(mode==1) {
      velocity=target;interval(velocity,velocity,c_.preview_time);
    } else if(mode==2) {
      double elapsed=0;
      while(elapsed<c_.preview_time-1e-9) {
        const double dt=std::min(c_.step_time,c_.preview_time-elapsed);
        Twist next=velocity;
        const double dx=target.x-velocity.x,dy=target.y-velocity.y,norm=std::hypot(dx,dy);
        const bool accelerating=target.x*velocity.x+target.y*velocity.y>=0 && speed(target)>speed(velocity);
        const double change=(accelerating?c_.linear_acceleration:c_.linear_braking)*dt;
        if(norm>1e-9) {const double k=std::min(1.,change/norm);next.x+=dx*k;next.y+=dy*k;}
        const bool angular_accelerating=target.w*velocity.w>=0 && std::abs(target.w)>std::abs(velocity.w);
        const double angular_change=(angular_accelerating?c_.angular_acceleration:c_.angular_braking)*dt;
        next.w+=std::clamp(target.w-velocity.w,-angular_change,angular_change);
        interval(velocity,next,dt);velocity=next;elapsed+=dt;
      }
    }
    // Coordinated braking preserves curvature and stays within both limits.
    const double braking_time=std::max(speed(velocity)/c_.linear_braking,std::abs(velocity.w)/c_.angular_braking);
    interval(velocity,{},braking_time);
    return poses;
  }
private:
  void validTwist(const Twist & v) const {
    if(!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.w) ||
       speed(v)>c_.max_linear_speed || std::abs(v.w)>c_.max_angular_speed)
      throw std::invalid_argument("motion outside supported prediction limits");
  }
  bool safe(const std::vector<Pose> & path,const Obstacles & obstacles) const {
    // A sample's rectangle is expanded by the maximum corner displacement
    // between samples. This bounds small obstacles between sampled poses.
    const double half_length=c_.length*.5+c_.margin+c_.step_distance;
    const double half_width=c_.width*.5+c_.margin+c_.step_distance;
    for(const auto & pose:path) if(obstacles.collision(pose,half_length,half_width)) return false;
    return true;
  }
  Config c_;Twist previous_;double hold_until_{0};
};
} // namespace go2w_nav2_navigation::dynamic_slowdown
