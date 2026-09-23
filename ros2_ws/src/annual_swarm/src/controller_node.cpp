#include <stdexcept>
#include <memory>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include "controller.hpp"
#include "allocator.hpp"
#include "experimental_controllers.hpp"

class FlightController : public rclcpp::Node {
 public:
  FlightController() : Node("controller") {
    const auto kind = declare_parameter<std::string>("controller_type", "pid");
    if (kind == "pid") controller_ = std::make_unique<sim::Controller>();
    else if (kind == "geometric_euler" || kind == "super_twisting") {
      auto value=std::make_unique<ExperimentalController>(kind == "super_twisting");
      value->kR=gains("geometric_kR",{48,48,18});
      value->kOmega=gains("geometric_kOmega",{10,10,6});
      controller_=std::move(value);
    }
    else if (kind == "smc") controller_ = std::make_unique<sim::HybridAttitudeController>();
    else if (kind == "backstepping" || kind == "backstepping_pid") {
      auto value = std::make_unique<sim::BacksteppingController>();
      value->set_use_smc(kind == "backstepping");
      value->K0=gains("backstepping_k0",{.5,.5,.6});
      value->K1=gains("backstepping_k1",{1.8,1.8,2.2});
      value->K2=gains("backstepping_k2",{3,3,3});
      controller_ = std::move(value);
    } else throw std::runtime_error("Unknown controller_type: " + kind);
    status_pub_ = create_publisher<std_msgs::msg::String>("controller_status", rclcpp::QoS(1).transient_local());
    std_msgs::msg::String status; status.data = kind + " (Gazebo actuator control)"; status_pub_->publish(status);
    RCLCPP_INFO(get_logger(), "Controller: %s", kind.c_str());
    controller_->apply_profile(1.0, 1.0, 3.0);
    // Discrete derivative gains of the offline simulator amplify transport jitter.
    // Conservative gains for the 100 Hz Gazebo odometry loop.
    controller_->kp_pos = gains("kp_pos", {1.2,1.2,1.5});
    controller_->ki_pos = gains("ki_pos", {0,0,0});
    controller_->kp_vel = gains("kp_vel", {2,2,3});
    controller_->ki_vel = gains("ki_vel", {0,0,0});
    controller_->kd_vel = gains("kd_vel", {0,0,0});
    controller_->kp_att = gains("kp_att", {6,6,3});
    controller_->kp_rate = gains("kp_rate", {12,12,6});
    controller_->ki_rate = gains("ki_rate", {0,0,0});
    controller_->kd_rate = gains("kd_rate", {0,0,0});
    pub_ = create_publisher<actuator_msgs::msg::Actuators>("command/motor_speed", 1);
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("target", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != "world") return;
        auto p = msg->pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
        const auto &q = msg->pose.orientation;
        tf2::Quaternion orientation(q.x,q.y,q.z,q.w);
        if (!std::isfinite(orientation.length2()) || orientation.length2() < 0.9) return;
        orientation.normalize();
        double roll, pitch;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, target_yaw_);
        // Pose and trajectory messages can share a stamp; preserve feed-forward.
        target_ = {p.x,p.y,p.z}; target_time_ = rclcpp::Time(msg->header.stamp).seconds(); have_target_ = true;
      });
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("trajectory_target", 1,
      [this](trajectory_msgs::msg::MultiDOFJointTrajectory::ConstSharedPtr msg) {
        if (msg->header.frame_id != "world" || msg->points.size() != 1) return;
        const auto &p = msg->points[0];
        if (p.transforms.size()!=1 || p.velocities.size()!=1 || p.accelerations.size()!=1) return;
        const auto &v=p.velocities[0].linear; const auto &a=p.accelerations[0].linear;
        const auto &x=p.transforms[0].translation;
        for (double n : {x.x,x.y,x.z,v.x,v.y,v.z,a.x,a.y,a.z}) if (!std::isfinite(n)) return;
        target_={x.x,x.y,x.z}; target_vel_={v.x,v.y,v.z}; target_acc_={a.x,a.y,a.z};
        target_time_=rclcpp::Time(msg->header.stamp).seconds(); have_target_=true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { update(*msg); });
  }
 private:
  std::array<double,3> gains(const std::string &name, std::vector<double> defaults) {
    auto value = declare_parameter<std::vector<double>>(name, defaults);
    if (value.size()!=3 || !std::all_of(value.begin(), value.end(), [](double v){return std::isfinite(v) && v>=0;}))
      throw std::runtime_error(name + " must contain three finite nonnegative gains");
    return {value[0], value[1], value[2]};
  }
  void update(const nav_msgs::msg::Odometry &msg) {
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    if (last_t_ >= 0 && t <= last_t_) { controller_->reset(); have_target_ = false; }
    if (last_t_ >= 0 && t-last_t_ > 0.1) controller_->reset();
    if (last_t_ >= 0 && t > last_t_) controller_->set_dt(std::clamp(t-last_t_,0.001,0.03));
    last_t_ = t;
    const auto &q = msg.pose.pose.orientation;
    tf2::Quaternion quat(q.x,q.y,q.z,q.w);
    if (quat.length2() < 0.9 || !std::isfinite(quat.length2())) return;
    quat.normalize(); tf2::Matrix3x3 rotation(quat);
    double roll,pitch,yaw; rotation.getRPY(roll,pitch,yaw);
    const auto &v = msg.twist.twist.linear;
    auto world_v = rotation * tf2::Vector3(v.x,v.y,v.z);
    const auto &p = msg.pose.pose.position; const auto &w = msg.twist.twist.angular;
    std::array<double,12> state{p.x,p.y,p.z,world_v.x(),world_v.y(),world_v.z(),roll,pitch,yaw,w.x,w.y,w.z};
    actuator_msgs::msg::Actuators motors; motors.header = msg.header;
    motors.header.frame_id = msg.child_frame_id;
    motors.velocity.resize(4,0);
    if (have_target_ && t-target_time_ >= -0.1 && t-target_time_ < 0.5) {
      auto u = controller_->compute_control(state, target_, target_vel_, target_acc_, target_yaw_);
      auto omega = allocator_.thrusts_to_omegas(allocator_.allocate_thrusts(u));
      for (int i=0; i<4; ++i) motors.velocity[i] = std::isfinite(omega[i]) ? std::clamp(omega[i],0.0,900.0) : 0;
    } else controller_->reset();
    pub_->publish(motors);
  }
  std::unique_ptr<sim::Controller> controller_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_sub_;
  sim::ControlAllocator allocator_;
  sim::Vec3 target_, target_vel_{}, target_acc_{};
  double target_yaw_ = 0;
  bool have_target_ = false;
  double target_time_ = -1, last_t_ = -1;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<FlightController>()); rclcpp::shutdown();
}
