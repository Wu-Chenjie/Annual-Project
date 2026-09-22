#pragma once
#include "controller.hpp"
#include <cmath>

// Ports of the Python-only research branches. Euler error semantics are retained
// for geometric_euler; it is intentionally not advertised as a full SO(3) law.
class ExperimentalController : public sim::Controller {
 public:
  explicit ExperimentalController(bool super_twisting) : super_twisting_(super_twisting) {}
  std::array<double,3> kR{48,48,18}, kOmega{10,10,6};
  void reset() override { sim::Controller::reset(); integral_={}; }
  std::array<double,4> compute_control(const std::array<double,12>& state,
      const sim::Vec3& position, const sim::Vec3& velocity={},
      const sim::Vec3& acceleration={}, double yaw=0.) override {
    auto out=compute_position_loop(state,position,velocity,acceleration,yaw);
    double error[3]={state[6]-out.des_att.x,state[7]-out.des_att.y,
                    std::remainder(state[8]-out.des_att.z,2*M_PI)};
    const double lambda[3]={4,4,2}, alpha[3]={8,8,4}, beta[3]={2,2,1};
    std::array<double,4> result{out.thrust,0,0,0};
    for (int i=0;i<3;++i) {
      double command=-kR[i]*error[i]-kOmega[i]*state[9+i];
      if (super_twisting_) {
        // s = actual rate + lambda * (actual - desired angle).
        // This sign convention makes the negative switching term stabilizing.
        const double surface=state[9+i]+lambda[i]*error[i];
        const double sign=std::clamp(surface/.05,-1.,1.);
        integral_[i]=std::clamp(integral_[i]-beta[i]*sign*dt_,-2.,2.);
        command=-lambda[i]*state[9+i]-alpha[i]*std::sqrt(std::abs(surface))*sign+integral_[i];
      }
      result[i+1]=inertia_diag_[i]*command;
    }
    // Rigid-body gyroscopic compensation, also present in the Python variant.
    result[1]+=(inertia_diag_[2]-inertia_diag_[1])*state[10]*state[11];
    result[2]+=(inertia_diag_[0]-inertia_diag_[2])*state[11]*state[9];
    result[3]+=(inertia_diag_[1]-inertia_diag_[0])*state[9]*state[10];
    return result;
  }
 private:
  bool super_twisting_;
  std::array<double,3> integral_{};
};
