#include "formation_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sim {

FormationSimulation::FormationSimulation(const SimulationConfig& config)
    : config_(config),
      dt_(config.dt),
      max_sim_time_(config.max_sim_time),
      use_smc_(config.use_smc),
      leader_(1.0, {0.01, 0.01, 0.02}, 0.2, config.dt),
      leader_wind_(Vec3{0.4, 0.0, 0.0}, 0.02, 0.8, config.leader_wind_seed),
      topology_(config.num_followers, config.formation_spacing) {
    leader_.set_initial_state(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0});

    if (!config_.custom_initial_offsets.empty()) {
        topology_.set_custom_offsets(config_.custom_initial_offsets);
    }
    if (config_.formation_adaptation_enabled || config_.formation_lookahead_enabled) {
        topology_.set_current_formation(config_.initial_formation);
    }

    if (config_.use_backstepping) {
        leader_ctrl_ = std::make_unique<BacksteppingController>(1.0, 9.81, std::array<double, 3>{0.01, 0.01, 0.02}, dt_);
    } else if (use_smc_) {
        auto ctrl = std::make_unique<HybridAttitudeController>(1.0, 9.81, std::array<double, 3>{0.01, 0.01, 0.02}, dt_);
        ctrl->set_use_smc(true);
        leader_ctrl_ = std::move(ctrl);
    } else {
        leader_ctrl_ = std::make_unique<Controller>(1.0, 9.81, std::array<double, 3>{0.01, 0.01, 0.02}, dt_);
    }

    apply_controller_profile(leader_ctrl_.get(),
                             config_.leader_gain_scale,
                             config_.leader_max_vel,
                             config_.leader_max_acc);

    build_followers();
    build_winds();
}

void FormationSimulation::apply_controller_profile(Controller* ctrl, double gain_scale, double max_vel, double max_acc) {
    if (ctrl == nullptr) {
        throw std::invalid_argument("controller pointer is null");
    }
    ctrl->apply_profile(gain_scale, max_vel, max_acc);
}

void FormationSimulation::build_followers() {
    followers_.clear();
    follower_ctrls_.clear();

    const std::vector<Vec3> offsets = topology_.get_offsets(config_.initial_formation);
    followers_.reserve(offsets.size());
    follower_ctrls_.reserve(offsets.size());

    const Vec3 leader_start = leader_.get_state().position;

    for (const Vec3& offset : offsets) {
        Drone follower(1.0, {0.01, 0.01, 0.02}, 0.2, dt_);
        follower.set_initial_state(leader_start + offset, Vec3{0.0, 0.0, 0.0});
        followers_.push_back(follower);

        if (config_.use_backstepping) {
            auto ctrl = std::make_unique<BacksteppingController>(1.0, 9.81, std::array<double, 3>{0.01, 0.01, 0.02}, dt_);
            apply_controller_profile(ctrl.get(),
                                     config_.follower_gain_scale,
                                     config_.follower_max_vel,
                                     config_.follower_max_acc);
            follower_ctrls_.push_back(std::move(ctrl));
        } else if (use_smc_) {
            auto ctrl = std::make_unique<HybridAttitudeController>(1.0, 9.81, std::array<double, 3>{0.01, 0.01, 0.02}, dt_);
            ctrl->set_use_smc(true);
            apply_controller_profile(ctrl.get(),
                                     config_.follower_gain_scale,
                                     config_.follower_max_vel,
                                     config_.follower_max_acc);
            follower_ctrls_.push_back(std::move(ctrl));
        } else {
            auto ctrl = std::make_unique<Controller>(1.0, 9.81, std::array<double, 3>{0.01, 0.01, 0.02}, dt_);
            apply_controller_profile(ctrl.get(),
                                     config_.follower_gain_scale,
                                     config_.follower_max_vel,
                                     config_.follower_max_acc);
            follower_ctrls_.push_back(std::move(ctrl));
        }
    }
}

void FormationSimulation::build_winds() {
    winds_.clear();
    winds_.reserve(followers_.size());

    for (std::size_t idx = 0; idx < followers_.size(); ++idx) {
        winds_.emplace_back(
            Vec3{0.4, 0.0, 0.0},
            0.02,
            0.8,
            config_.follower_wind_seed_start + static_cast<unsigned int>(idx)
        );
    }
}

void FormationSimulation::maybe_switch_formation(double time_now) {
    if (next_switch_idx_ >= config_.formation_schedule.size()) {
        return;
    }

    const FormationSwitchEvent& event = config_.formation_schedule[next_switch_idx_];
    if (time_now >= event.trigger_time) {
        topology_.switch_formation(event.target_formation, event.transition_time, time_now);
        ++next_switch_idx_;
    }
}

SimulationResult FormationSimulation::run() {
    if (config_.waypoints.empty()) {
        throw std::runtime_error("waypoints cannot be empty");
    }

    const std::size_t steps = static_cast<std::size_t>(max_sim_time_ / dt_) + 1U;
    const std::size_t follower_count = followers_.size();

    SimulationResult result{};
    result.time.resize(steps, 0.0);
    result.leader.resize(steps, Vec3{0.0, 0.0, 0.0});
    result.followers.assign(follower_count, std::vector<Vec3>(steps, Vec3{0.0, 0.0, 0.0}));
    result.targets.assign(follower_count, std::vector<Vec3>(steps, Vec3{0.0, 0.0, 0.0}));
    result.error_vectors.assign(follower_count, std::vector<Vec3>(steps, Vec3{0.0, 0.0, 0.0}));
    result.errors.assign(follower_count, std::vector<double>(steps, 0.0));

    std::size_t current_wp_idx = 0;
    bool finished = false;
    double time_now = 0.0;
    std::size_t step_idx = 0;

    Vec3 leader_acc_filt{0.0, 0.0, 0.0};

    while (time_now < max_sim_time_ && step_idx < steps) {
        maybe_switch_formation(time_now);

        const Vec3 target_wp = finished ? config_.waypoints.back() : config_.waypoints[current_wp_idx];

        const Vec3 wind_leader = leader_wind_.sample(dt_);
        const KinematicState leader_state_before = leader_.get_state();
        const std::array<double, 4> leader_u = leader_ctrl_->compute_control(leader_.state(), target_wp);
        leader_.update_state(leader_u, wind_leader);
        const KinematicState leader_state_after = leader_.get_state();

        if (!finished) {
            const double dist_to_wp = norm(target_wp - leader_state_after.position);
            const double radius = (current_wp_idx == config_.waypoints.size() - 1U) ? config_.wp_radius_final : config_.wp_radius;
            if (dist_to_wp < radius) {
                ++current_wp_idx;
                if (current_wp_idx >= config_.waypoints.size()) {
                    finished = true;
                }
            }
        }

        const Vec3 leader_acc = (leader_state_after.velocity - leader_state_before.velocity) / dt_;
        leader_acc_filt = config_.leader_acc_alpha * leader_acc + (1.0 - config_.leader_acc_alpha) * leader_acc_filt;

        const std::vector<Vec3> offsets = topology_.get_current_offsets(time_now);

        for (std::size_t i = 0; i < follower_count; ++i) {
            const Vec3 wind_follower = winds_[i].sample(dt_);
            const Vec3 target_pos = leader_state_after.position + offsets[i];

            const std::array<double, 4> follower_u = follower_ctrls_[i]->compute_control(
                followers_[i].state(),
                target_pos,
                leader_state_after.velocity,
                leader_acc_filt
            );

            followers_[i].update_state(follower_u, wind_follower);
            const Vec3 follower_pos = followers_[i].get_state().position;
            const Vec3 error_vec = follower_pos - target_pos;

            result.targets[i][step_idx] = target_pos;
            result.error_vectors[i][step_idx] = error_vec;
            result.errors[i][step_idx] = norm(error_vec);
            result.followers[i][step_idx] = follower_pos;
        }

        result.time[step_idx] = time_now;
        result.leader[step_idx] = leader_state_after.position;

        ++step_idx;
        time_now += dt_;
    }

    result.time.resize(step_idx);
    result.leader.resize(step_idx);
    for (std::size_t i = 0; i < follower_count; ++i) {
        result.followers[i].resize(step_idx);
        result.targets[i].resize(step_idx);
        result.error_vectors[i].resize(step_idx);
        result.errors[i].resize(step_idx);
    }

    result.metrics.mean.resize(follower_count, 0.0);
    result.metrics.max.resize(follower_count, 0.0);
    result.metrics.final.resize(follower_count, 0.0);

    for (std::size_t i = 0; i < follower_count; ++i) {
        if (result.errors[i].empty()) {
            continue;
        }

        double sum = 0.0;
        double max_error = std::numeric_limits<double>::lowest();
        for (double e : result.errors[i]) {
            sum += e;
            max_error = std::max(max_error, e);
        }

        result.metrics.mean[i] = sum / static_cast<double>(result.errors[i].size());
        result.metrics.max[i] = max_error;
        result.metrics.final[i] = result.errors[i].back();
    }

    result.completed_waypoint_count = finished ? static_cast<int>(config_.waypoints.size()) : static_cast<int>(current_wp_idx);
    result.waypoints = config_.waypoints;
    return result;
}

}  // namespace sim
