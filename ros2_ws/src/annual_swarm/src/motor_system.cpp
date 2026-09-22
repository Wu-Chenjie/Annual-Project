#include <stdexcept>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/Utility.hh>

namespace annual {
// Four rotor forces, motor lag, reaction torques, and aerodynamic drag.
// Gazebo alone integrates the state and resolves contact.
class MotorSystem final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
 public:
  ~MotorSystem() override {
    node_.Unsubscribe("/" + name_ + "/command/motor_speed");
  }
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    name_ = model.Name(ecm);
    link_ = gz::sim::Link(model.LinkByName(ecm, "base_link"));
    if (!link_.Valid(ecm)) throw std::runtime_error("quadrotor base_link missing");
    link_.EnableVelocityChecks(ecm);
    tau_ = sdf->Get<double>("motor_time_constant", 0.025).first;
    if (tau_ <= 0) throw std::runtime_error("motor_time_constant must be positive");
    node_.Subscribe("/" + name_ + "/command/motor_speed", &MotorSystem::Command, this);
    odom_ = node_.Advertise<gz::msgs::Odometry>("/" + name_ + "/odometry");
  }
  void Command(const gz::msgs::Actuators &msg) {
    if (msg.velocity_size() != 4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < 4; ++i) {
      if (!std::isfinite(msg.velocity(i))) return;
    }
    for (int i = 0; i < 4; ++i) requested_[i] = std::clamp(msg.velocity(i), 0.0, 900.0);
    pending_ = true;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    std::lock_guard<std::mutex> lock(mutex_);
    if (dt < 0) { speeds_.fill(0); requested_.fill(0); pending_ = false; last_command_ = -1; return; }
    if (pending_) { last_command_ = t; pending_ = false; }
    const bool fresh = last_command_ >= 0 && t - last_command_ <= 0.25;
    std::array<double, 4> f{};
    for (int i = 0; i < 4; ++i) {
      speeds_[i] += (fresh ? requested_[i] - speeds_[i] : -speeds_[i]) * (1 - std::exp(-dt / tau_));
      f[i] = 1e-5 * speeds_[i] * speeds_[i];
    }
    // Rotor positions: (-a,+a), (-a,-a), (+a,-a), (+a,+a).
    gz::math::Vector3d force(0, 0, f[0]+f[1]+f[2]+f[3]);
    gz::math::Vector3d torque(0.2*(f[0]-f[1]-f[2]+f[3]),
        0.2*(f[0]+f[1]-f[2]-f[3]), 0.02*(-f[0]+f[1]-f[2]+f[3]));
    const auto pose = gz::sim::worldPose(link_.Entity(), ecm);
    auto vel = link_.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    auto angular = link_.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    link_.AddWorldWrench(ecm, pose.Rot().RotateVector(force) - 0.1 * vel,
                       pose.Rot().RotateVector(torque) - 0.002 * angular);
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (t < last_odom_) last_odom_ = -1;
    if (info.paused || t - last_odom_ < 0.0099) return;
    last_odom_ = t;
    const auto pose = gz::sim::worldPose(link_.Entity(), ecm);
    const auto vel = link_.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    const auto angular = link_.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    gz::msgs::Odometry msg;
    gz::msgs::Set(msg.mutable_header()->mutable_stamp(), info.simTime);
    auto *frame = msg.mutable_header()->add_data(); frame->set_key("frame_id"); frame->add_value("world");
    auto *child = msg.mutable_header()->add_data(); child->set_key("child_frame_id"); child->add_value(name_ + "/base_link");
    gz::msgs::Set(msg.mutable_pose(), pose);
    // nav_msgs/Odometry twist is in child_frame_id, NOT the world frame.
    gz::msgs::Set(msg.mutable_twist()->mutable_linear(), pose.Rot().RotateVectorReverse(vel));
    gz::msgs::Set(msg.mutable_twist()->mutable_angular(), pose.Rot().RotateVectorReverse(angular));
    odom_.Publish(msg);
  }
 private:
  gz::sim::Link link_;
  gz::transport::Node::Publisher odom_;
  std::string name_;
  std::mutex mutex_;
  std::array<double, 4> requested_{}, speeds_{};
  bool pending_ = false;
  double tau_ = 0.025, last_command_ = -1, last_odom_ = -1;
  // Destroy transport first, while callback state and mutex are still alive.
  gz::transport::Node node_;
};
}
GZ_ADD_PLUGIN(annual::MotorSystem, gz::sim::System, annual::MotorSystem::ISystemConfigure,
             annual::MotorSystem::ISystemPreUpdate, annual::MotorSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(annual::MotorSystem, "annual::MotorSystem")
