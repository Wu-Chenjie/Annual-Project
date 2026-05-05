#include "obstacle_scenario.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace sim {

ObstacleScenarioSimulation::ObstacleScenarioSimulation(const ObstacleConfig& config)
    : config_(config),
      formation_(config),
      apf_(0.8, 2.5, 2, 0.2, 2.0, 0.5, 8.0),
      num_followers_(config.num_followers) {

    double arm_length = 0.2;
    collision_margin_ = arm_length + config.safety_margin * config.detect_margin_scale;
    task_waypoints_ = config.waypoints;
    waypoints_ = task_waypoints_;

    if (config.enable_obstacles && !config.map_file.empty()) {
        setup_obstacles();
        waypoints_ = sanitize_waypoints(config_.waypoints);
        config_.waypoints = waypoints_;
        if (!waypoints_.empty()) {
            Vec3 first = waypoints_[0];
            formation_.leader_.set_initial_state(first, Vec3{});
            auto init_offsets = formation_.topology_.get_offsets(config.initial_formation);
            for (size_t i = 0; i < init_offsets.size() && i < formation_.followers_.size(); ++i) {
                formation_.followers_[i].set_initial_state(first + init_offsets[i], Vec3{});
            }
        }
        auto path = plan_offline();
        if (!path.empty()) {
            planned_path_ = std::move(path);
            if (config.planner_mode != "online") {
                waypoints_ = planned_path_;
            }
        }
    } else if (!config.waypoints.empty()) {
        Vec3 first = config.waypoints[0];
        formation_.leader_.set_initial_state(first, Vec3{});
        auto init_offsets = formation_.topology_.get_offsets(config.initial_formation);
        for (size_t i = 0; i < init_offsets.size() && i < formation_.followers_.size(); ++i) {
            formation_.followers_[i].set_initial_state(first + init_offsets[i], Vec3{});
        }
    }

    if (config.planner_mode == "online" && !grid_.data.empty()) {
        setup_online();
    }
}

void ObstacleScenarioSimulation::set_obstacles(ObstacleField field, const std::array<Vec3, 2>& bounds) {
    obstacles_ = std::move(field);
    map_bounds_ = bounds;
    task_waypoints_ = config_.waypoints;

    Vec3 extent{bounds[1].x - bounds[0].x, bounds[1].y - bounds[0].y, bounds[1].z - bounds[0].z};
    grid_ = OccupancyGrid::from_obstacles(obstacles_, bounds[0], extent, config_.planner_resolution);
    double ir = inflate_r();
    grid_ = grid_.inflate(ir);

    if (config_.planner_sdf_aware) {
        sdf_grid_ = std::make_unique<SDFAwareGrid>(grid_, obstacles_, compute_clearance());
    }
    apply_planning_z_bounds();

    waypoints_ = sanitize_waypoints(config_.waypoints);
    config_.waypoints = waypoints_;
    auto path = plan_offline();
    if (!path.empty()) {
        planned_path_ = std::move(path);
        waypoints_ = (config_.planner_mode == "online") ? task_waypoints_ : planned_path_;
    } else {
        waypoints_ = task_waypoints_;
    }

    if (config_.planner_mode == "online") {
        setup_online();
    }
}

void ObstacleScenarioSimulation::setup_obstacles() {
    auto [field, bounds] = load_from_json(config_.map_file);
    obstacles_ = std::move(field);
    map_bounds_ = bounds;

    Vec3 extent{bounds[1].x - bounds[0].x, bounds[1].y - bounds[0].y, bounds[1].z - bounds[0].z};
    grid_ = OccupancyGrid::from_obstacles(obstacles_, bounds[0], extent, config_.planner_resolution);

    double ir = inflate_r();
    grid_ = grid_.inflate(ir);

    if (config_.planner_sdf_aware) {
        sdf_grid_ = std::make_unique<SDFAwareGrid>(grid_, obstacles_, compute_clearance());
    }
    apply_planning_z_bounds();
}

double ObstacleScenarioSimulation::inflate_r() const {
    if (!config_.planner_use_formation_envelope) return config_.safety_margin;
    FormationTopology topo(config_.num_followers, config_.formation_spacing, 0.2);
    return topo.envelope_radius(config_.initial_formation) + config_.safety_margin;
}

double ObstacleScenarioSimulation::compute_clearance() const {
    return std::max(config_.safety_margin, collision_margin_) + config_.plan_clearance_extra;
}

void ObstacleScenarioSimulation::apply_planning_z_bounds() {
    if (!config_.planner_has_z_bounds) return;
    const double z_min = std::min(config_.planner_z_min, config_.planner_z_max);
    const double z_max = std::max(config_.planner_z_min, config_.planner_z_max);
    for (int iz = 0; iz < grid_.nz; ++iz) {
        double z = grid_.origin.z + iz * grid_.resolution;
        if (z >= z_min - 1e-9 && z <= z_max + 1e-9) continue;
        for (int iy = 0; iy < grid_.ny; ++iy) {
            for (int ix = 0; ix < grid_.nx; ++ix) {
                grid_.data[(iz * grid_.ny + iy) * grid_.nx + ix] = 1;
            }
        }
    }
}

std::vector<Vec3> ObstacleScenarioSimulation::sanitize_waypoints(const std::vector<Vec3>& waypoints) const {
    std::vector<Vec3> safe;
    safe.reserve(waypoints.size());
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        Vec3 prefer{};
        Vec3* prefer_ptr = nullptr;
        if (i + 1 < waypoints.size()) {
            prefer = waypoints[i + 1] - waypoints[i];
            prefer_ptr = &prefer;
        } else if (i > 0) {
            prefer = waypoints[i] - waypoints[i - 1];
            prefer_ptr = &prefer;
        }
        safe.push_back(project_to_planning_free(waypoints[i], prefer_ptr));
    }
    return safe;
}

Vec3 ObstacleScenarioSimulation::project_to_planning_free(
    const Vec3& point, const Vec3* prefer, double max_radius_m) const {

    if (grid_.data.empty()) return point;

    const OccupancyGrid& plan_grid = sdf_grid_
        ? static_cast<const OccupancyGrid&>(*sdf_grid_)
        : static_cast<const OccupancyGrid&>(grid_);
    const double min_clearance = compute_clearance();
    auto idx = plan_grid.world_to_index(point);
    if (!plan_grid.is_occupied(idx[0], idx[1], idx[2])
        && obstacles_.signed_distance(point) >= min_clearance - 1e-8) {
        return point;
    }

    if (max_radius_m < 0.0) max_radius_m = std::max(6.0, inflate_r() + 2.0);
    int max_radius_vox = std::max(1, static_cast<int>(std::ceil(max_radius_m / plan_grid.resolution)));

    Vec3 prefer_dir{};
    bool has_prefer = false;
    if (prefer && norm(*prefer) > 1e-9) {
        prefer_dir = *prefer / norm(*prefer);
        has_prefer = true;
    }

    Vec3 best = point;
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int radius = 0; radius <= max_radius_vox; ++radius) {
        const int ix_min = std::max(0, idx[0] - radius);
        const int ix_max = std::min(plan_grid.nx - 1, idx[0] + radius);
        const int iy_min = std::max(0, idx[1] - radius);
        const int iy_max = std::min(plan_grid.ny - 1, idx[1] + radius);
        const int iz_min = std::max(0, idx[2] - radius);
        const int iz_max = std::min(plan_grid.nz - 1, idx[2] + radius);

        for (int ix = ix_min; ix <= ix_max; ++ix) {
            for (int iy = iy_min; iy <= iy_max; ++iy) {
                for (int iz = iz_min; iz <= iz_max; ++iz) {
                    if (std::max({std::abs(ix - idx[0]), std::abs(iy - idx[1]), std::abs(iz - idx[2])}) != radius) {
                        continue;
                    }
                    if (plan_grid.is_occupied(ix, iy, iz)) continue;
                    Vec3 candidate = plan_grid.index_to_world(ix, iy, iz);
                    double sd = obstacles_.signed_distance(candidate);
                    if (sd < min_clearance - 1e-8) continue;

                    Vec3 delta = candidate - point;
                    double score = norm(delta);
                    if (has_prefer && norm(delta) > 1e-9) {
                        Vec3 dir = delta / norm(delta);
                        score += 0.25 * (1.0 - dot(dir, prefer_dir));
                    }
                    score += 0.05 * std::abs(candidate.z - point.z);
                    if (score < best_score) {
                        best_score = score;
                        best = candidate;
                        found = true;
                    }
                }
            }
        }
        if (found) return best;
    }
    return point;
}

std::vector<Vec3> ObstacleScenarioSimulation::plan_offline() {
    const auto& wps = waypoints_.empty() ? config_.waypoints : waypoints_;
    if (wps.size() < 2) return {};

    double min_clearance = compute_clearance();
    std::vector<Vec3> full_path = {wps[0]};

    const OccupancyGrid& plan_grid = sdf_grid_
        ? static_cast<const OccupancyGrid&>(*sdf_grid_)
        : static_cast<const OccupancyGrid&>(grid_);

    HybridAStarPlanner ha_planner(2.0, 1.0, 1.0472, 0.5, 72, 10000, 50);
    bool use_hybrid = (config_.planner_kind == "hybrid_astar");

    for (std::size_t i = 0; i < wps.size() - 1; ++i) {
        Vec3 start = wps[i], goal = wps[i + 1];

        std::vector<Vec3> path_segment;
        bool seg_ok = false;

        if (use_hybrid) {
            auto res = ha_planner.plan(start, goal, plan_grid, static_cast<int>(i * 100 + 42));
            if (res.success) { path_segment = res.path; seg_ok = true; }
        } else {
            auto res = astar_plan(start, goal, plan_grid);
            if (res.success) { path_segment = res.path; seg_ok = true; }
        }

        if (!seg_ok) {
            continue;
        }

        auto smoothed = smooth_path(path_segment, 4);
        smoothed = enforce_path_clearance(smoothed, min_clearance);
        if (config_.firi_enabled) {
            FIRIRefiner refiner(obstacles_, min_clearance, config_.firi_sample_step,
                                config_.firi_max_projection_iter);
            smoothed = refiner.refine(smoothed, path_segment);
            smoothed = enforce_path_clearance(smoothed, min_clearance);
        }

        if (smoothed.size() >= 2) {
            std::vector<Vec3> dense;
            double spacing = std::max(config_.planner_resolution * 2.0, 0.8);
            double total = 0.0;
            for (size_t j = 1; j < smoothed.size(); ++j) total += norm(smoothed[j] - smoothed[j - 1]);
            int n = std::max(2, static_cast<int>(std::ceil(total / spacing)));
            dense.resize(n);
            std::vector<double> cum(smoothed.size(), 0.0);
            for (size_t j = 1; j < smoothed.size(); ++j) {
                cum[j] = cum[j - 1] + norm(smoothed[j] - smoothed[j - 1]);
            }
            for (int j = 0; j < n; ++j) {
                double t = j * total / (n - 1);
                auto it = std::lower_bound(cum.begin(), cum.end(), t);
                size_t k = std::min(static_cast<size_t>(it - cum.begin()), cum.size() - 1);
                if (k == 0) {
                    dense[j] = smoothed[0];
                    continue;
                }
                double seg = cum[k] - cum[k - 1];
                double frac = (seg > 1e-12) ? (t - cum[k - 1]) / seg : 0.0;
                frac = std::max(0.0, std::min(1.0, frac));
                dense[j] = smoothed[k - 1] + (smoothed[k] - smoothed[k - 1]) * frac;
            }
            smoothed = std::move(dense);
        }

        if (!full_path.empty() && !smoothed.empty()) {
            double d = norm(smoothed[0] - full_path.back());
            if (d < 1e-4) smoothed.erase(smoothed.begin());
        }
        full_path.insert(full_path.end(), smoothed.begin(), smoothed.end());
    }

    return full_path;
}

std::vector<Vec3> ObstacleScenarioSimulation::enforce_path_clearance(
    const std::vector<Vec3>& path, double min_clearance) {

    auto out = path;
    FormationTopology topo(config_.num_followers, config_.formation_spacing);
    auto offsets = topo.get_offsets(config_.initial_formation);

    for (auto& wp : out) {
        for (int iter = 0; iter < 60; ++iter) {
            double min_sd = obstacles_.signed_distance(wp);
            Vec3 worst = wp;
            for (const auto& off : offsets) {
                Vec3 check = wp + off;
                double sd = obstacles_.signed_distance(check);
                if (sd < min_sd) { min_sd = sd; worst = check; }
            }
            if (min_sd >= min_clearance) break;

            double eps = 0.05;
            Vec3 grad{
                obstacles_.signed_distance({worst.x + eps, worst.y, worst.z})
                    - obstacles_.signed_distance({worst.x - eps, worst.y, worst.z}),
                obstacles_.signed_distance({worst.x, worst.y + eps, worst.z})
                    - obstacles_.signed_distance({worst.x, worst.y - eps, worst.z}),
                obstacles_.signed_distance({worst.x, worst.y, worst.z + eps})
                    - obstacles_.signed_distance({worst.x, worst.y, worst.z - eps}),
            };
            grad = grad / (2.0 * eps);
            double gn = norm(grad);
            if (gn < 1e-10) break;
            wp = wp + (grad / gn) * 0.10;
        }
    }
    return out;
}

double ObstacleScenarioSimulation::path_segment_clearance(
    const std::vector<Vec3>& path, double min_clearance) const {
    if (path.empty()) return -std::numeric_limits<double>::infinity();
    if (path.size() == 1) return obstacles_.signed_distance(path.front());

    const double sample_step = std::max(
        0.02,
        std::min(std::max(config_.planner_resolution * 0.5, 0.02), 0.10));
    double worst = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec3 a = path[i];
        const Vec3 b = path[i + 1];
        const double dist = norm(b - a);
        const int samples = std::max(1, static_cast<int>(std::ceil(dist / sample_step)));
        for (int j = 0; j <= samples; ++j) {
            const double u = static_cast<double>(j) / static_cast<double>(samples);
            const Vec3 p = a * (1.0 - u) + b * u;
            worst = std::min(worst, obstacles_.signed_distance(p));
            if (worst < min_clearance) return worst;
        }
    }
    return worst;
}

bool ObstacleScenarioSimulation::path_is_clearance_safe(
    const std::vector<Vec3>& path, double min_clearance) const {
    return path_segment_clearance(path, min_clearance) >= min_clearance - 1e-6;
}

Vec3 ObstacleScenarioSimulation::obstacle_repulsion_acc(const Vec3& pos, const Vec3& goal) {
    return apf_.compute_avoidance_acceleration(pos, goal, obstacles_);
}

void ObstacleScenarioSimulation::setup_online() {
    if (grid_.data.empty()) return;

    const OccupancyGrid& pg = sdf_grid_
        ? static_cast<const OccupancyGrid&>(*sdf_grid_)
        : static_cast<const OccupancyGrid&>(grid_);
    replanner_ = std::make_unique<WindowReplanner>(pg, config_.planner_replan_interval,
                                                    config_.planner_horizon, 0.5, 3);
    replanner_->set_obstacle_field(&obstacles_);
    if (config_.replan_adaptive_interval) {
        replanner_->enable_adaptive_interval(config_.replan_interval_min, config_.replan_interval_max);
    }
    if (config_.danger_mode_enabled) {
        replanner_->set_dual_mode(std::make_unique<DualModeScheduler>(
            config_.danger_sensor_threshold,
            config_.danger_sensor_safe_threshold,
            config_.danger_sdf_threshold,
            config_.danger_hysteresis_margin));
        replanner_->set_danger_planner(std::make_unique<GNNPlanner>());
    }

    auto gp = std::make_unique<HybridAStarPlanner>(2.0, 1.0, 1.0472, 0.5, 72, 4000, 50);
    replanner_->set_global_planner(std::move(gp));

    if (config_.sensor_enabled) {
        sensor_ = std::make_unique<RangeSensor6>(config_.sensor_max_range, config_.sensor_noise_std,
                                                  config_.leader_wind_seed);
    }
}

std::vector<Vec3> ObstacleScenarioSimulation::stitch_local_path_to_task_goal(
    const std::vector<Vec3>& local_path, const Vec3& task_goal) const {
    if (local_path.empty()) return {};
    std::vector<Vec3> stitched = local_path;
    if (norm(stitched.back() - task_goal) > config_.wp_radius) {
        stitched.push_back(task_goal);
    }
    return stitched;
}

SimulationResult ObstacleScenarioSimulation::run() {
    int steps = static_cast<int>(config_.max_sim_time / config_.dt) + 1;
    int follower_count = num_followers_;

    SimulationResult result;
    result.time.resize(steps);
    result.leader.resize(steps);
    result.followers.resize(follower_count, std::vector<Vec3>(steps));
    result.targets.resize(follower_count, std::vector<Vec3>(steps));
    result.error_vectors.resize(follower_count, std::vector<Vec3>(steps));
    result.errors.resize(follower_count, std::vector<double>(steps));

    collision_log_.clear();
    replanned_waypoints_.clear();
    executed_path_.clear();
    fault_log_.clear();

    auto& leader = formation_.leader_;
    auto& leader_ctrl = formation_.leader_ctrl_;
    auto& followers = formation_.followers_;
    auto& follower_ctrls = formation_.follower_ctrls_;
    auto& leader_wind = formation_.leader_wind_;
    auto& winds = formation_.winds_;
    auto& topology = formation_.topology_;

    bool online = config_.planner_mode == "online" && replanner_ != nullptr;
    const std::vector<Vec3>& tasks = task_waypoints_.empty() ? config_.waypoints : task_waypoints_;
    std::vector<Vec3> active_path = online ? tasks : waypoints_;
    int task_wp_idx = 0;
    int local_wp_idx = 0;
    bool finished = tasks.empty();
    double t = 0.0;
    Vec3 leader_acc_filt{};
    std::size_t next_switch_idx = 0;
    auto& schedule = config_.formation_schedule;
    double alpha = config_.leader_acc_alpha;
    double dt = config_.dt;
    bool fault_injected = false;
    std::set<int> faulted_followers;
    std::unique_ptr<FaultDetector> fault_detector;
    if (config_.fault_detection_enabled) {
        fault_detector = std::make_unique<FaultDetector>(
            config_.fault_detector_max_acc,
            config_.fault_detector_pos_dev,
            config_.fault_detector_saturate_steps,
            config_.dt);
    }
    double fault_time = config_.fault_injection_time >= 0.0
        ? config_.fault_injection_time
        : std::min(8.0, 0.25 * config_.max_sim_time);

    int step_idx = 0;
    while (t < config_.max_sim_time && step_idx < steps) {
        auto ls_before_replan = leader.get_state();

        if (online && !finished && task_wp_idx < static_cast<int>(tasks.size())) {
            std::array<double, 6> sdata{};
            const std::array<double, 6>* sp = nullptr;
            if (sensor_) {
                sdata = sensor_->sense(ls_before_replan.position, obstacles_);
                sp = &sdata;
            }

            Vec3 task_goal = tasks[task_wp_idx];
            auto new_path = replanner_->step(t, ls_before_replan.position, sp, task_goal);
            if (!new_path.empty()) {
                auto candidate_path = stitch_local_path_to_task_goal(new_path, task_goal);
                const double clearance = compute_clearance();
                if (config_.firi_enabled && candidate_path.size() >= 2) {
                    FIRIRefiner refiner(obstacles_, clearance, config_.firi_sample_step,
                                        config_.firi_max_projection_iter);
                    candidate_path = refiner.refine(candidate_path, candidate_path);
                    candidate_path = enforce_path_clearance(candidate_path, clearance);
                }
                bool candidate_safe = path_is_clearance_safe(candidate_path, clearance);
                if (!candidate_safe) {
                    candidate_path = enforce_path_clearance(candidate_path, clearance);
                    candidate_safe = path_is_clearance_safe(candidate_path, clearance);
                }
                if (candidate_safe) {
                    active_path = std::move(candidate_path);
                    waypoints_ = active_path;
                    replanned_waypoints_.insert(replanned_waypoints_.end(), active_path.begin(), active_path.end());
                    local_wp_idx = 0;
                    if (!active_path.empty() && norm(active_path[0] - ls_before_replan.position) < config_.wp_radius) {
                        local_wp_idx = std::min(1, static_cast<int>(active_path.size()));
                    }
                }
            }
        }

        if (next_switch_idx < schedule.size() && t >= schedule[next_switch_idx].trigger_time) {
            auto& ev = schedule[next_switch_idx];
            topology.switch_formation(ev.target_formation, ev.transition_time, t);
            ++next_switch_idx;
        }

        if (config_.fault_injection_enabled && !fault_injected && !followers.empty() && t >= fault_time) {
            int follower_idx = std::clamp(config_.fault_injection_follower, 0,
                                          static_cast<int>(followers.size()) - 1);
            followers[static_cast<std::size_t>(follower_idx)].inject_fault(
                config_.fault_injection_rotor, config_.fault_injection_severity);
            fault_injected = true;
            fault_log_.push_back(
                "inject:follower_" + std::to_string(follower_idx)
                + ":rotor_" + std::to_string(config_.fault_injection_rotor)
                + ":severity_" + std::to_string(config_.fault_injection_severity)
                + ":t_" + std::to_string(t));
        }

        Vec3 target{};
        if (finished) {
            target = tasks.empty() ? Vec3{} : tasks.back();
        } else if (online) {
            if (local_wp_idx < static_cast<int>(active_path.size())) {
                target = active_path[local_wp_idx];
            } else if (task_wp_idx < static_cast<int>(tasks.size())) {
                target = tasks[task_wp_idx];
            } else {
                target = tasks.back();
            }
        } else if (local_wp_idx < static_cast<int>(active_path.size())) {
            target = active_path[local_wp_idx];
        } else {
            target = active_path.empty() ? Vec3{} : active_path.back();
        }

        auto ls0 = leader.get_state();
        Vec3 rep_acc = obstacle_repulsion_acc(ls0.position, target);

        auto u = leader_ctrl->compute_control(leader.state(), target, Vec3{}, rep_acc);
        leader.update_state(u, leader_wind.sample(dt));

        auto ls = leader.get_state();
        result.leader[step_idx] = ls.position;
        executed_path_.push_back(ls.position);

        Vec3 leader_acc{(ls.velocity.x - ls0.velocity.x) / dt,
                        (ls.velocity.y - ls0.velocity.y) / dt,
                        (ls.velocity.z - ls0.velocity.z) / dt};
        leader_acc_filt = leader_acc * alpha + leader_acc_filt * (1.0 - alpha);

        if (obstacles_.is_collision(ls.position, collision_margin_)) {
            collision_log_.push_back({t, "leader", ls.position});
        }

        if (!finished) {
            if (online && task_wp_idx < static_cast<int>(tasks.size())) {
                double task_radius = (task_wp_idx == static_cast<int>(tasks.size()) - 1)
                    ? config_.wp_radius_final
                    : config_.wp_radius;
                if (norm(tasks[task_wp_idx] - ls.position) < task_radius) {
                    ++task_wp_idx;
                    local_wp_idx = 0;
                    active_path = (task_wp_idx < static_cast<int>(tasks.size()))
                        ? std::vector<Vec3>{tasks[task_wp_idx]}
                        : std::vector<Vec3>{tasks.back()};
                    if (task_wp_idx >= static_cast<int>(tasks.size())) finished = true;
                } else if (local_wp_idx < static_cast<int>(active_path.size())) {
                    double local_radius = (local_wp_idx == static_cast<int>(active_path.size()) - 1)
                        ? task_radius
                        : config_.wp_radius;
                    if (norm(active_path[local_wp_idx] - ls.position) < local_radius) {
                        ++local_wp_idx;
                    }
                }
            } else if (local_wp_idx < static_cast<int>(active_path.size())) {
                double radius = (local_wp_idx == static_cast<int>(active_path.size()) - 1)
                    ? config_.wp_radius_final
                    : config_.wp_radius;
                if (norm(active_path[local_wp_idx] - ls.position) < radius) {
                    ++local_wp_idx;
                    if (local_wp_idx >= static_cast<int>(active_path.size())) finished = true;
                }
            }
        }

        auto offsets = topology.get_current_offsets(t);
        for (int i = 0; i < follower_count; ++i) {
            Vec3 target_pos = ls.position + offsets[i];
            Vec3 follower_pos = followers[i].get_state().position;
            Vec3 rep_acc_f = obstacle_repulsion_acc(follower_pos, target_pos);
            auto u_f = follower_ctrls[i]->compute_control(
                followers[i].state(), target_pos, ls.velocity,
                leader_acc_filt + rep_acc_f);

            if (fault_detector && faulted_followers.count(i) == 0) {
                if (fault_detector->check(i, followers[i].state(), target_pos, ls.velocity, u_f, 20.0)) {
                    faulted_followers.insert(i);
                    fault_log_.push_back("detect:follower_" + std::to_string(i) + ":t_" + std::to_string(t));
                    if (config_.fault_reconfig_enabled) {
                        std::vector<int> failed(faulted_followers.begin(), faulted_followers.end());
                        std::string topo = topology.fault_reconfigure(failed, 3.0, t);
                        fault_log_.push_back("reconfigure:" + topo + ":t_" + std::to_string(t));
                    }
                }
            }

            followers[i].update_state(u_f, winds[i].sample(dt));

            auto fs = followers[i].get_state();
            Vec3 err_v = fs.position - target_pos;
            result.targets[i][step_idx] = target_pos;
            result.error_vectors[i][step_idx] = err_v;
            result.errors[i][step_idx] = norm(err_v);
            result.followers[i][step_idx] = fs.position;

            if (obstacles_.is_collision(fs.position, collision_margin_)) {
                collision_log_.push_back({t, "follower_" + std::to_string(i), fs.position});
            }
        }

        result.time[step_idx] = t;
        ++step_idx;
        t += dt;
    }

    result.completed_waypoint_count = online ? task_wp_idx : local_wp_idx;
    result.waypoints = waypoints_;
    result.task_waypoints = tasks;
    result.replanned_waypoints = replanned_waypoints_;
    result.executed_path = executed_path_;
    result.fault_log = fault_log_;

    result.metrics.mean.resize(follower_count);
    result.metrics.max.resize(follower_count);
    result.metrics.final.resize(follower_count);
    int valid = step_idx;
    for (int i = 0; i < follower_count; ++i) {
        double sum = 0.0, mx = 0.0;
        for (int j = 0; j < valid; ++j) {
            double e = result.errors[i][j];
            sum += e;
            if (e > mx) mx = e;
        }
        result.metrics.mean[i] = (valid > 0) ? (sum / valid) : 0.0;
        result.metrics.max[i] = mx;
        result.metrics.final[i] = (valid > 0) ? result.errors[i][valid - 1] : 0.0;
    }

    return result;
}

}  // namespace sim
