#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "astar_planner.hpp"
#include "config.hpp"
#include "dstar_lite.hpp"
#include "hybrid_astar_planner.hpp"
#include "map_loader.hpp"
#include "obstacle_scenario.hpp"
#include "occupancy_grid.hpp"
#include "sensor.hpp"

namespace sim {

struct ObstacleDesc {
    std::string id;
    std::string type;
    Vec3 center_or_min;
    Vec3 size_or_max;
};

struct DynamicEvent {
    double t = 0.0;
    std::string action = "add";
    ObstacleDesc obstacle;
    std::string target_id;
};

struct DynamicSimInput {
    ObstacleConfig base_config;
    std::string map_file;
    std::vector<DynamicEvent> events;
    std::vector<std::string> compare_planners{"dstar_lite", "astar"};
    int repeat_count = 1;
};

struct PlannerResult {
    std::vector<Vec3> path;
    double compute_time_ms = 0.0;
    std::string phase;
    bool replan_triggered = false;
    bool success = false;
};

struct FrameSnapshot {
    double t = 0.0;
    Vec3 leader_pose;
    std::vector<Vec3> follower_poses;
    std::vector<ObstacleDesc> obstacles;
    std::array<double, 6> sensor_readings{};
    std::unordered_map<std::string, PlannerResult> planner_results;
    bool collision = false;
    bool clearance_blocked = false;
    bool obstacle_changed = false;
};

struct SummaryMetrics {
    int total_replans = 0;
    double avg_compute_ms = 0.0;
    double max_compute_ms = 0.0;
    double p95_compute_ms = 0.0;
    int collisions = 0;
    double path_length_m = 0.0;
    double final_dist_to_goal = 0.0;
    std::unordered_map<std::string, double> phase_distribution;
};

struct ReplayReplanEvent {
    double t = 0.0;
    std::string planner;
    std::string phase;
    bool success = false;
};

struct ReplayCollisionEvent {
    double t = 0.0;
    std::string actor;
    Vec3 pos;
};

struct ReplayFaultEvent {
    double t = 0.0;
    std::string type;
    std::string detail;
};

struct ReplayOutput {
    struct Metadata {
        std::string map_file;
        std::string scenario = "dynamic_obstacle";
        std::string execution_scope = "leader_centric";
        std::string follower_pose_mode = "illustrative_lateral_offsets";
        std::string summary_mode = "shared_leader_frame_eval";
        std::string apf_profile_scope = "cpp_subset_runtime";
        std::vector<std::string> apf_runtime_fields;
        std::vector<std::string> apf_python_only_fields;
        std::vector<std::string> apf_cpp_pending_fields;
        std::string initial_formation;
        std::vector<std::string> formation_adaptation_candidates;
        bool formation_adaptation_enabled = false;
        bool formation_lookahead_enabled = false;
        bool formation_lookahead_rrt_enabled = false;
        double formation_lookahead_distance = 0.0;
        double formation_lookahead_turn_threshold_rad = 0.0;
        int step_count = 0;
        double dt = 0.0;
        double total_time = 0.0;
        int repeat_index = 0;
        int seed = 42;
    } metadata;

    std::vector<ObstacleDesc> static_obstacles;
    std::array<Vec3, 2> bounds{};
    std::vector<DynamicEvent> dynamic_events;
    std::vector<FormationAdaptationEvent> formation_adaptation_events;
    std::vector<Vec3> waypoints;
    std::vector<Vec3> task_waypoints;
    std::vector<Vec3> replanned_waypoints;
    std::vector<Vec3> executed_path;
    std::vector<ReplayReplanEvent> replan_events;
    std::vector<std::array<double, 6>> sensor_logs;
    std::vector<ReplayCollisionEvent> collision_log;
    std::vector<ReplayFaultEvent> fault_log;
    std::vector<FrameSnapshot> frames;
    std::unordered_map<std::string, SummaryMetrics> summaries;
};

class DynamicScenarioRunner {
public:
    explicit DynamicScenarioRunner(const DynamicSimInput& input, int seed = 42);

    ReplayOutput run();
    std::vector<ReplayOutput> run_batch();

private:
    bool apply_events(double t);
    PlannerResult run_one_planner(const std::string& kind, const Vec3& leader_pose,
                                  const Vec3& goal, bool obstacle_changed);
    OccupancyGrid build_planning_grid() const;
    std::vector<std::pair<std::array<int, 3>, bool>> rebuild_grid_and_diff();
    double planning_margin() const;
    void reset_incremental_planner(const Vec3& leader_pose, const Vec3& goal);
    void apply_planning_z_bounds();
    std::vector<Vec3> sanitize_waypoints(const std::vector<Vec3>& waypoints) const;
    Vec3 project_to_planning_free(const Vec3& point, const Vec3* prefer = nullptr) const;
    Vec3 reachable_planning_start(const Vec3& point, const Vec3& goal) const;
    void ensure_reachable_replan_grid(const Vec3& leader_pose, const Vec3& goal);
    void handle_add_obstacle(const DynamicEvent& event);
    void handle_remove_obstacle(const DynamicEvent& event);
    void handle_move_obstacle(const DynamicEvent& event);
    std::vector<ObstacleDesc> describe_obstacles() const;
    SummaryMetrics summarize(const std::string& planner, const Vec3& final_pose,
                             const Vec3& goal) const;
    // Returns the minimum SDF clearance sampled along every segment of a candidate path.
    // Stops early once the sampled clearance falls below min_clearance.
    double path_segment_clearance(const std::vector<Vec3>& path, double min_clearance) const;
    // Path-level acceptance gate: true only when all sampled path segments satisfy min_clearance.
    bool path_is_clearance_safe(const std::vector<Vec3>& path, double min_clearance) const;
    double clearance_tolerance(double scale) const;
    bool capsule_clear(const Vec3& start, const Vec3& end, double radius) const;
    Vec3 safe_advance_along_path(const Vec3& pose, const std::vector<Vec3>& path,
                                 double distance, double radius, bool& capsule_blocked) const;

    static double path_length(const std::vector<Vec3>& path);
    static Vec3 advance_along_path(const Vec3& pose, const std::vector<Vec3>& path, double distance);
    static double percentile95(std::vector<double> values);

    DynamicSimInput input_;
    int seed_ = 42;
    ObstacleConfig config_;
    ObstacleField obstacles_;
    std::array<Vec3, 2> bounds_{};
    OccupancyGrid grid_;
    std::unique_ptr<DStarLite> dstar_lite_;
    std::unique_ptr<HybridAStarPlanner> hybrid_astar_;
    std::size_t event_cursor_ = 0;
    std::vector<DynamicEvent> events_;
    std::unordered_map<std::string, std::vector<double>> compute_times_;
    std::unordered_map<std::string, std::vector<double>> path_lengths_;
    std::unordered_map<std::string, int> replans_;
    std::unordered_map<std::string, int> collisions_;
    std::unordered_map<std::string, int> phase_counts_;
    double clearance_extra_margin_ = 0.0;
    int clearance_extra_frames_ = 0;
    RangeSensor6 sensor_;
};

inline DynamicScenarioRunner::DynamicScenarioRunner(const DynamicSimInput& input, int seed)
    : input_(input), seed_(seed), config_(input.base_config), events_(input.events),
      sensor_(input.base_config.sensor_max_range, input.base_config.sensor_noise_std, static_cast<unsigned int>(seed)) {
    config_.leader_wind_seed = static_cast<unsigned int>(seed_);
    config_.follower_wind_seed_start = static_cast<unsigned int>(seed_ + 100);
    if (!input_.map_file.empty()) config_.map_file = input_.map_file;

    if (!config_.map_file.empty()) {
        auto loaded = load_from_json(config_.map_file);
        obstacles_ = std::move(loaded.first);
        bounds_ = loaded.second;
    } else {
        bounds_ = {Vec3{-5, -5, 0}, Vec3{50, 35, 12}};
        obstacles_.regenerate_ids("obs_");
    }

    grid_ = build_planning_grid();
    config_.waypoints = sanitize_waypoints(config_.waypoints);

    const Vec3 start = config_.waypoints.empty() ? Vec3{} : config_.waypoints.front();
    const Vec3 initial_goal = config_.waypoints.size() > 1 ? config_.waypoints[1] : start;
    reset_incremental_planner(start, initial_goal);
    hybrid_astar_ = std::make_unique<HybridAStarPlanner>(
        config_.leader_max_vel, config_.leader_max_acc, 1.0472,
        config_.planner_resolution, 72, 10000, 50);

    std::sort(events_.begin(), events_.end(), [](const DynamicEvent& a, const DynamicEvent& b) {
        return a.t < b.t;
    });
}

inline ReplayOutput DynamicScenarioRunner::run() {
    ReplayOutput output;
    output.metadata.map_file = config_.map_file;
    output.metadata.dt = std::max(config_.dt, std::max(0.05, config_.planner_replan_interval));
    output.metadata.total_time = config_.max_sim_time;
    output.metadata.seed = seed_;
    output.metadata.initial_formation = config_.initial_formation;
    output.metadata.formation_adaptation_candidates = config_.formation_adaptation_candidates;
    output.metadata.formation_adaptation_enabled = config_.formation_adaptation_enabled;
    output.metadata.formation_lookahead_enabled = config_.formation_lookahead_enabled;
    output.metadata.formation_lookahead_rrt_enabled = config_.formation_lookahead_rrt_enabled;
    output.metadata.formation_lookahead_distance = config_.formation_lookahead_distance;
    output.metadata.formation_lookahead_turn_threshold_rad = config_.formation_lookahead_turn_threshold_rad;
    output.metadata.apf_runtime_fields = {
        "apf_paper1_profile",
        "apf_comm_range",
        "apf_comm_constraint",
        "apf_adaptive_n_decay",
        "apf_formation_centroid",
        "apf_centroid_alpha",
        "apf_centroid_beta",
        "apf_rotational_escape",
        "mu_escape",
        "k_inter",
        "k_comm",
        "comm_range",
        "adaptive_n_decay",
        "formation_apf_runtime",
    };
    output.metadata.apf_python_only_fields = {
    };
    output.metadata.apf_cpp_pending_fields = {
    };
    output.bounds = bounds_;
    output.static_obstacles = describe_obstacles();
    output.dynamic_events = events_;
    output.waypoints = config_.waypoints;
    output.task_waypoints = config_.waypoints;

    const Vec3 start = config_.waypoints.empty() ? Vec3{} : config_.waypoints.front();
    Vec3 leader_pose = start;
    std::vector<Vec3> active_path;
    std::vector<Vec3> replanned_waypoints;
    std::vector<Vec3> executed_path;
    std::vector<ReplayReplanEvent> replan_events;
    std::vector<std::array<double, 6>> sensor_logs;
    std::vector<ReplayCollisionEvent> collision_log;
    std::size_t task_goal_idx = config_.waypoints.size() > 1 ? 1U : 0U;
    const double task_radius = std::max(config_.wp_radius, config_.planner_resolution);

    for (double t = 0.0; t <= config_.max_sim_time + 1e-9; t += output.metadata.dt) {
        const bool obstacle_changed = apply_events(t);
        const Vec3 goal = config_.waypoints.empty() ? leader_pose : config_.waypoints[task_goal_idx];
        FrameSnapshot frame;
        frame.t = t;
        frame.obstacle_changed = obstacle_changed;
        frame.leader_pose = leader_pose;
        frame.obstacles = describe_obstacles();
        frame.sensor_readings = sensor_.sense(leader_pose, obstacles_);
        sensor_logs.push_back(frame.sensor_readings);
        for (int i = 0; i < config_.num_followers; ++i) {
            const double lateral = (static_cast<double>(i) - (config_.num_followers - 1) * 0.5)
                                 * config_.formation_spacing;
            frame.follower_poses.push_back(leader_pose + Vec3{0.0, lateral, 0.0});
        }

        const double collision_tol = clearance_tolerance(config_.safety_margin);
        frame.collision = obstacles_.signed_distance(leader_pose) < config_.safety_margin - collision_tol;
        if (frame.collision) {
            collision_log.push_back({t, "leader", leader_pose});
        }
        std::vector<Vec3> selected_path;
        for (const auto& planner : input_.compare_planners) {
            auto result = run_one_planner(planner, leader_pose, goal, obstacle_changed || t == 0.0);
            if (result.success) {
                if (planner == "dstar_lite") {
                    selected_path = result.path;
                } else if (selected_path.empty()) {
                    selected_path = result.path;
                }
            }
            frame.planner_results[planner] = result;
            compute_times_[planner].push_back(result.compute_time_ms);
            if (result.success) path_lengths_[planner].push_back(path_length(result.path));
            if (result.replan_triggered) ++replans_[planner];
            if (result.replan_triggered) {
                replan_events.push_back({t, planner, result.phase, result.success});
            }
            if (frame.collision) ++collisions_[planner];
            if (!result.phase.empty()) ++phase_counts_[result.phase];
        }
        if (!selected_path.empty()) {
            const double accept_clearance = std::max(
                config_.safety_margin + config_.plan_clearance_extra,
                config_.safety_margin);
            if (path_is_clearance_safe(selected_path, accept_clearance)) {
                active_path = std::move(selected_path);
                replanned_waypoints.insert(replanned_waypoints.end(), active_path.begin(), active_path.end());
            } else {
                frame.clearance_blocked = true;
                if (config_.formation_adaptation_enabled || config_.formation_lookahead_enabled) {
                    FormationAdaptationEvent event;
                    event.t = t;
                    event.kind = "lookahead_reference_blocked";
                    event.from = config_.initial_formation;
                    event.to = config_.initial_formation;
                    event.reason = "dynamic_replay_clearance_gate";
                    event.has_clearance_margin = true;
                    event.clearance_margin = path_segment_clearance(selected_path, accept_clearance) - accept_clearance;
                    event.planner = "dynamic_replay";
                    event.point_count = static_cast<int>(selected_path.size());
                    output.formation_adaptation_events.push_back(event);
                }
                ensure_reachable_replan_grid(leader_pose, goal);
            }
        }

        const Vec3 before_advance = leader_pose;
        bool capsule_blocked = false;
        leader_pose = safe_advance_along_path(
            leader_pose, active_path,
            std::max(0.05, config_.leader_max_vel) * output.metadata.dt,
            config_.safety_margin, capsule_blocked);
        if (capsule_blocked) {
            frame.clearance_blocked = true;
            if (norm(leader_pose - before_advance) < 1e-6) {
                leader_pose = before_advance;
            }
            active_path.clear();
            ensure_reachable_replan_grid(leader_pose, goal);
        } else if (clearance_extra_frames_ > 0) {
            --clearance_extra_frames_;
            if (clearance_extra_frames_ == 0 && clearance_extra_margin_ > 0.0) {
                clearance_extra_margin_ = 0.0;
                rebuild_grid_and_diff();
                reset_incremental_planner(leader_pose, goal);
            }
        }
        executed_path.push_back(leader_pose);
        output.frames.push_back(frame);
        if (!config_.waypoints.empty() && norm(leader_pose - goal) <= task_radius) {
            if (task_goal_idx + 1 < config_.waypoints.size()) {
                ++task_goal_idx;
                active_path.clear();
                reset_incremental_planner(leader_pose, config_.waypoints[task_goal_idx]);
            } else if (norm(leader_pose - goal) <= std::max(config_.wp_radius_final, config_.planner_resolution)) {
                break;
            }
        }
    }

    output.metadata.step_count = static_cast<int>(output.frames.size());
    const Vec3 final_goal = config_.waypoints.empty() ? leader_pose : config_.waypoints.back();
    const double one_step = std::max(0.05, config_.leader_max_vel) * output.metadata.dt;
    if (!config_.waypoints.empty()
        && norm(leader_pose - final_goal) <= std::max(config_.wp_radius, config_.wp_radius_final) + one_step + 1e-9) {
        leader_pose = final_goal;
    }
    if (!output.frames.empty()) {
        output.frames.back().leader_pose = leader_pose;
    }
    output.replanned_waypoints = std::move(replanned_waypoints);
    output.executed_path = std::move(executed_path);
    output.replan_events = std::move(replan_events);
    output.sensor_logs = std::move(sensor_logs);
    output.collision_log = std::move(collision_log);
    output.fault_log = {};
    for (const auto& planner : input_.compare_planners) {
        output.summaries[planner] = summarize(planner, leader_pose, final_goal);
    }
    return output;
}

inline std::vector<ReplayOutput> DynamicScenarioRunner::run_batch() {
    std::vector<ReplayOutput> outputs;
    int count = std::max(1, input_.repeat_count);
    outputs.reserve(static_cast<std::size_t>(count));
    for (int run = 0; run < count; ++run) {
        DynamicScenarioRunner runner(input_, seed_ + run);
        ReplayOutput output = runner.run();
        output.metadata.repeat_index = run;
        output.metadata.seed = seed_ + run;
        outputs.push_back(std::move(output));
    }
    return outputs;
}

inline bool DynamicScenarioRunner::apply_events(double t) {
    bool changed = false;
    while (event_cursor_ < events_.size() && events_[event_cursor_].t <= t + 1e-9) {
        const DynamicEvent& event = events_[event_cursor_];
        if (event.action == "remove") handle_remove_obstacle(event);
        else if (event.action == "move") handle_move_obstacle(event);
        else handle_add_obstacle(event);
        auto changed_cells = rebuild_grid_and_diff();
        if (dstar_lite_ && !changed_cells.empty()) {
            dstar_lite_->update_cells(changed_cells);
        }
        changed = true;
        ++event_cursor_;
    }
    return changed;
}

inline PlannerResult DynamicScenarioRunner::run_one_planner(const std::string& kind,
                                                            const Vec3& leader_pose,
                                                            const Vec3& goal,
                                                            bool obstacle_changed) {
    PlannerResult result;
    result.phase = kind == "dstar_lite" ? (obstacle_changed ? "incremental" : "local")
                 : kind == "hybrid_astar" ? "global_hybrid" : "global";
    result.replan_triggered = true;
    const auto begin = std::chrono::high_resolution_clock::now();
    const Vec3 plan_start = reachable_planning_start(leader_pose, goal);
    if (kind == "dstar_lite") {
        dstar_lite_->update_start(plan_start);
        dstar_lite_->compute_shortest_path();
        result.path = dstar_lite_->extract_path();
        result.success = result.path.size() >= 2;
    } else if (kind == "hybrid_astar") {
        auto ha_result = hybrid_astar_->plan(plan_start, goal, grid_, seed_);
        result.path = std::move(ha_result.path);
        result.success = ha_result.success;
    } else {
        auto astar = astar_plan(plan_start, goal, grid_);
        result.path = std::move(astar.path);
        result.success = astar.success;
    }
    if (result.success && norm(plan_start - leader_pose) > 1e-6
        && capsule_clear(leader_pose, plan_start, config_.safety_margin)) {
        result.path.insert(result.path.begin(), leader_pose);
    }
    const auto end = std::chrono::high_resolution_clock::now();
    result.compute_time_ms = std::chrono::duration<double, std::milli>(end - begin).count();
    return result;
}

inline OccupancyGrid DynamicScenarioRunner::build_planning_grid() const {
    Vec3 extent = bounds_[1] - bounds_[0];
    OccupancyGrid new_grid = OccupancyGrid::from_obstacles(
        obstacles_, bounds_[0], extent, config_.planner_resolution);

    const double margin = planning_margin();
    if (margin > 0.0) {
        for (int iz = 0; iz < new_grid.nz; ++iz) {
            for (int iy = 0; iy < new_grid.ny; ++iy) {
                for (int ix = 0; ix < new_grid.nx; ++ix) {
                    Vec3 p = new_grid.index_to_world(ix, iy, iz);
                    if (obstacles_.signed_distance(p) < margin - 1e-9) {
                        new_grid.data[(iz * new_grid.ny + iy) * new_grid.nx + ix] = 1;
                    }
                }
            }
        }
    }

    if (config_.planner_has_z_bounds) {
        const double z_min = std::min(config_.planner_z_min, config_.planner_z_max);
        const double z_max = std::max(config_.planner_z_min, config_.planner_z_max);
        for (int iz = 0; iz < new_grid.nz; ++iz) {
            double z = new_grid.origin.z + iz * new_grid.resolution;
            if (z >= z_min - 1e-9 && z <= z_max + 1e-9) continue;
            for (int iy = 0; iy < new_grid.ny; ++iy) {
                for (int ix = 0; ix < new_grid.nx; ++ix) {
                    new_grid.data[(iz * new_grid.ny + iy) * new_grid.nx + ix] = 1;
                }
            }
        }
    }
    return new_grid;
}

inline std::vector<std::pair<std::array<int, 3>, bool>> DynamicScenarioRunner::rebuild_grid_and_diff() {
    OccupancyGrid old_grid = grid_;
    OccupancyGrid new_grid = build_planning_grid();

    std::vector<std::pair<std::array<int, 3>, bool>> changed;
    const std::size_t n = std::min(old_grid.data.size(), new_grid.data.size());
    for (std::size_t flat = 0; flat < n; ++flat) {
        if ((old_grid.data[flat] >= 1) == (new_grid.data[flat] >= 1)) continue;
        int ix = static_cast<int>(flat % static_cast<std::size_t>(new_grid.nx));
        int iy = static_cast<int>((flat / static_cast<std::size_t>(new_grid.nx)) % static_cast<std::size_t>(new_grid.ny));
        int iz = static_cast<int>(flat / static_cast<std::size_t>(new_grid.nx * new_grid.ny));
        changed.push_back({{ix, iy, iz}, new_grid.data[flat] >= 1});
    }
    grid_ = std::move(new_grid);
    return changed;
}

inline double DynamicScenarioRunner::planning_margin() const {
    return std::max(0.0, config_.safety_margin + config_.plan_clearance_extra + clearance_extra_margin_);
}

inline void DynamicScenarioRunner::reset_incremental_planner(const Vec3& leader_pose,
                                                             const Vec3& goal) {
    const Vec3 plan_start = reachable_planning_start(leader_pose, goal);
    dstar_lite_ = std::make_unique<DStarLite>(grid_, plan_start, goal);
    dstar_lite_->compute_shortest_path();
}

inline void DynamicScenarioRunner::apply_planning_z_bounds() {
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

inline std::vector<Vec3> DynamicScenarioRunner::sanitize_waypoints(const std::vector<Vec3>& waypoints) const {
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

inline Vec3 DynamicScenarioRunner::project_to_planning_free(const Vec3& point, const Vec3* prefer) const {
    if (grid_.data.empty()) return point;
    auto idx = grid_.world_to_index(point);
    const double min_clearance = planning_margin();
    if (!grid_.is_occupied(idx[0], idx[1], idx[2])
        && obstacles_.signed_distance(point) >= min_clearance - 1e-8) {
        return point;
    }

    Vec3 prefer_dir{};
    bool has_prefer = false;
    if (prefer && norm(*prefer) > 1e-9) {
        prefer_dir = *prefer / norm(*prefer);
        has_prefer = true;
    }

    int max_radius_vox = std::max(1, static_cast<int>(std::ceil(6.0 / grid_.resolution)));
    Vec3 best = point;
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int radius = 0; radius <= max_radius_vox; ++radius) {
        const int ix_min = std::max(0, idx[0] - radius);
        const int ix_max = std::min(grid_.nx - 1, idx[0] + radius);
        const int iy_min = std::max(0, idx[1] - radius);
        const int iy_max = std::min(grid_.ny - 1, idx[1] + radius);
        const int iz_min = std::max(0, idx[2] - radius);
        const int iz_max = std::min(grid_.nz - 1, idx[2] + radius);
        for (int ix = ix_min; ix <= ix_max; ++ix) {
            for (int iy = iy_min; iy <= iy_max; ++iy) {
                for (int iz = iz_min; iz <= iz_max; ++iz) {
                    if (std::max({std::abs(ix - idx[0]), std::abs(iy - idx[1]), std::abs(iz - idx[2])}) != radius) {
                        continue;
                    }
                    if (grid_.is_occupied(ix, iy, iz)) continue;
                    Vec3 candidate = grid_.index_to_world(ix, iy, iz);
                    double sd = obstacles_.signed_distance(candidate);
                    if (sd < min_clearance - 1e-8) continue;
                    Vec3 delta = candidate - point;
                    double score = norm(delta) + 0.05 * std::abs(candidate.z - point.z);
                    if (has_prefer && norm(delta) > 1e-9) {
                        Vec3 dir = delta / norm(delta);
                        score += 0.25 * (1.0 - dot(dir, prefer_dir));
                    }
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

inline Vec3 DynamicScenarioRunner::reachable_planning_start(const Vec3& point, const Vec3& goal) const {
    if (grid_.data.empty()) return point;
    auto idx = grid_.world_to_index(point);
    if (!grid_.is_occupied(idx[0], idx[1], idx[2])) return point;

    Vec3 prefer = goal - point;
    Vec3 prefer_dir = norm(prefer) > 1e-9 ? prefer / norm(prefer) : Vec3{1.0, 0.0, 0.0};
    const double min_clearance = config_.safety_margin;
    const int max_radius_vox = std::max(1, static_cast<int>(std::ceil(8.0 / grid_.resolution)));
    Vec3 best = point;
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int radius = 1; radius <= max_radius_vox; ++radius) {
        const int ix_min = std::max(0, idx[0] - radius);
        const int ix_max = std::min(grid_.nx - 1, idx[0] + radius);
        const int iy_min = std::max(0, idx[1] - radius);
        const int iy_max = std::min(grid_.ny - 1, idx[1] + radius);
        const int iz_min = std::max(0, idx[2] - radius);
        const int iz_max = std::min(grid_.nz - 1, idx[2] + radius);
        for (int ix = ix_min; ix <= ix_max; ++ix) {
            for (int iy = iy_min; iy <= iy_max; ++iy) {
                for (int iz = iz_min; iz <= iz_max; ++iz) {
                    if (std::max({std::abs(ix - idx[0]), std::abs(iy - idx[1]), std::abs(iz - idx[2])}) != radius) {
                        continue;
                    }
                    if (grid_.is_occupied(ix, iy, iz)) continue;
                    Vec3 candidate = grid_.index_to_world(ix, iy, iz);
                    if (obstacles_.signed_distance(candidate) < min_clearance - 1e-8) continue;
                    if (!capsule_clear(point, candidate, config_.safety_margin)) continue;
                    Vec3 delta = candidate - point;
                    const double d = norm(delta);
                    if (d < 1e-9) continue;
                    const double toward_goal_penalty = 0.4 * (1.0 - dot(delta / d, prefer_dir));
                    const double score = d + toward_goal_penalty + 0.05 * std::abs(candidate.z - point.z);
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

inline void DynamicScenarioRunner::ensure_reachable_replan_grid(const Vec3& leader_pose,
                                                                const Vec3& goal) {
    const double requested_extra = std::max(0.15, std::min(0.60, config_.planner_resolution));
    const std::array<double, 4> candidates{
        requested_extra,
        requested_extra * 0.5,
        std::min(0.10, requested_extra * 0.25),
        0.0
    };
    for (double extra : candidates) {
        clearance_extra_margin_ = std::max(0.0, extra);
        rebuild_grid_and_diff();
        const Vec3 plan_start = reachable_planning_start(leader_pose, goal);
        auto start_idx = grid_.world_to_index(plan_start);
        auto goal_idx = grid_.world_to_index(goal);
        if (!grid_.is_occupied(start_idx[0], start_idx[1], start_idx[2])
            && !grid_.is_occupied(goal_idx[0], goal_idx[1], goal_idx[2])) {
            clearance_extra_frames_ = clearance_extra_margin_ > 0.0 ? 6 : 0;
            reset_incremental_planner(leader_pose, goal);
            return;
        }
    }
    clearance_extra_margin_ = 0.0;
    clearance_extra_frames_ = 0;
    rebuild_grid_and_diff();
    reset_incremental_planner(leader_pose, goal);
}

inline void DynamicScenarioRunner::handle_add_obstacle(const DynamicEvent& event) {
    const auto& obs = event.obstacle;
    if (obs.type == "aabb") {
        obstacles_.add_aabb_id(obs.center_or_min, obs.size_or_max, obs.id);
    } else if (obs.type == "cylinder") {
        obstacles_.add_cylinder_id(obs.center_or_min, obs.size_or_max.x,
                                   obs.size_or_max.y, obs.size_or_max.z, obs.id);
    } else {
        obstacles_.add_sphere_id(obs.center_or_min, obs.size_or_max.x, obs.id);
    }
}

inline void DynamicScenarioRunner::handle_remove_obstacle(const DynamicEvent& event) {
    obstacles_.remove_by_id(event.target_id);
}

inline void DynamicScenarioRunner::handle_move_obstacle(const DynamicEvent& event) {
    DynamicEvent moved = event;
    if (moved.obstacle.id.empty()) moved.obstacle.id = moved.target_id;
    obstacles_.remove_by_id(moved.target_id);
    handle_add_obstacle(moved);
}

inline std::vector<ObstacleDesc> DynamicScenarioRunner::describe_obstacles() const {
    std::vector<ObstacleDesc> out;
    const auto& obs = obstacles_.obstacles();
    const auto& ids = obstacles_.ids();
    out.reserve(obs.size());
    for (std::size_t i = 0; i < obs.size(); ++i) {
        ObstacleDesc desc;
        desc.id = i < ids.size() ? ids[i] : "obs_" + std::to_string(i);
        std::visit([&desc](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                desc.type = "sphere";
                desc.center_or_min = value.center;
                desc.size_or_max = Vec3{value.radius, 0.0, 0.0};
            } else if constexpr (std::is_same_v<T, AABB>) {
                desc.type = "aabb";
                desc.center_or_min = value.min_corner;
                desc.size_or_max = value.max_corner;
            } else {
                desc.type = "cylinder";
                desc.center_or_min = value.center_xy;
                desc.size_or_max = Vec3{value.radius, value.z_min, value.z_max};
            }
        }, obs[i]);
        out.push_back(desc);
    }
    return out;
}

inline SummaryMetrics DynamicScenarioRunner::summarize(const std::string& planner,
                                                       const Vec3& final_pose,
                                                       const Vec3& goal) const {
    SummaryMetrics summary;
    auto time_it = compute_times_.find(planner);
    if (time_it != compute_times_.end() && !time_it->second.empty()) {
        const auto& values = time_it->second;
        summary.avg_compute_ms = std::accumulate(values.begin(), values.end(), 0.0)
                               / static_cast<double>(values.size());
        summary.max_compute_ms = *std::max_element(values.begin(), values.end());
        summary.p95_compute_ms = percentile95(values);
    }
    auto length_it = path_lengths_.find(planner);
    if (length_it != path_lengths_.end() && !length_it->second.empty()) {
        summary.path_length_m = length_it->second.back();
    }
    if (auto it = replans_.find(planner); it != replans_.end()) summary.total_replans = it->second;
    if (auto it = collisions_.find(planner); it != collisions_.end()) summary.collisions = it->second;
    summary.final_dist_to_goal = norm(final_pose - goal);
    if (planner == "dstar_lite") {
        int total = 0;
        for (const auto& kv : phase_counts_) total += kv.second;
        if (total > 0) {
            for (const auto& kv : phase_counts_) {
                summary.phase_distribution[kv.first] = static_cast<double>(kv.second) / total;
            }
        }
    }
    return summary;
}

inline double DynamicScenarioRunner::path_length(const std::vector<Vec3>& path) {
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) total += norm(path[i] - path[i - 1]);
    return total;
}

inline double DynamicScenarioRunner::path_segment_clearance(
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

inline bool DynamicScenarioRunner::path_is_clearance_safe(
    const std::vector<Vec3>& path, double min_clearance) const {
    return path_segment_clearance(path, min_clearance) >= min_clearance - clearance_tolerance(min_clearance);
}

inline double DynamicScenarioRunner::clearance_tolerance(double scale) const {
    const double local_scale = std::max({
        std::abs(scale),
        std::abs(config_.safety_margin),
        std::abs(config_.planner_resolution)
    });
    return std::max(1e-4, local_scale * 0.01);
}

inline bool DynamicScenarioRunner::capsule_clear(const Vec3& start, const Vec3& end,
                                                 double radius) const {
    const double clearance_tol = clearance_tolerance(radius);
    const double length = norm(end - start);
    if (length < 1e-9) return obstacles_.signed_distance(start) >= radius - clearance_tol;
    const double step = std::max(0.02, std::min({
        std::max(0.02, config_.planner_resolution * 0.25),
        std::max(0.02, radius * 0.5),
        0.10
    }));
    const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int i = 0; i <= samples; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(samples);
        const Vec3 p = start * (1.0 - u) + end * u;
        if (obstacles_.signed_distance(p) < radius - clearance_tol) return false;
    }
    return true;
}

inline Vec3 DynamicScenarioRunner::safe_advance_along_path(
    const Vec3& pose, const std::vector<Vec3>& path, double distance, double radius,
    bool& capsule_blocked) const {
    capsule_blocked = false;
    if (path.empty() || distance <= 0.0) return pose;
    if (path.size() == 1) {
        if (capsule_clear(pose, path.front(), radius)) return path.front();
        capsule_blocked = true;
        return pose;
    }

    std::size_t best_seg = 0;
    Vec3 current = path.front();
    double best_projection_dist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec3 a = path[i];
        const Vec3 b = path[i + 1];
        const Vec3 ab = b - a;
        const double len2 = dot(ab, ab);
        double u = 0.0;
        if (len2 > 1e-12) {
            u = std::max(0.0, std::min(1.0, dot(pose - a, ab) / len2));
        }
        const Vec3 projection = a + ab * u;
        const double d = norm(pose - projection);
        if (d < best_projection_dist) {
            best_projection_dist = d;
            best_seg = i;
            current = projection;
        }
    }

    if (norm(current - pose) > 1e-6) {
        if (!capsule_clear(pose, current, radius)) {
            capsule_blocked = true;
            return pose;
        }
    }

    double remaining = distance;
    for (std::size_t i = best_seg + 1; i < path.size() && remaining > 1e-9; ++i) {
        const Vec3 target = path[i];
        const Vec3 delta = target - current;
        const double d = norm(delta);
        if (d < 1e-9) continue;
        const Vec3 next = d <= remaining ? target : current + delta / d * remaining;
        if (!capsule_clear(current, next, radius)) {
            capsule_blocked = true;
            double lo = 0.0;
            double hi = 1.0;
            for (int iter = 0; iter < 18; ++iter) {
                const double mid = (lo + hi) * 0.5;
                const Vec3 candidate = current * (1.0 - mid) + next * mid;
                if (capsule_clear(current, candidate, radius)) lo = mid;
                else hi = mid;
            }
            const Vec3 safe = current * (1.0 - lo) + next * lo;
            return norm(safe - pose) > 1e-6 ? safe : pose;
        }
        current = next;
        remaining -= std::min(d, remaining);
    }
    return current;
}

inline Vec3 DynamicScenarioRunner::advance_along_path(const Vec3& pose,
                                                      const std::vector<Vec3>& path,
                                                      double distance) {
    if (path.empty() || distance <= 0.0) return pose;
    if (path.size() == 1) return path.front();

    std::size_t best_seg = 0;
    Vec3 current = path.front();
    double best_projection_dist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec3 a = path[i];
        const Vec3 b = path[i + 1];
        const Vec3 ab = b - a;
        const double len2 = dot(ab, ab);
        double u = 0.0;
        if (len2 > 1e-12) {
            u = std::max(0.0, std::min(1.0, dot(pose - a, ab) / len2));
        }
        const Vec3 projection = a + ab * u;
        const double d = norm(pose - projection);
        if (d < best_projection_dist) {
            best_projection_dist = d;
            best_seg = i;
            current = projection;
        }
    }

    double remaining = distance;
    for (std::size_t i = best_seg + 1; i < path.size() && remaining > 1e-9; ++i) {
        const Vec3 target = path[i];
        const Vec3 delta = target - current;
        const double d = norm(delta);
        if (d < 1e-9) continue;
        if (d >= remaining) return current + delta / d * remaining;
        current = target;
        remaining -= d;
    }
    return current;
}

inline double DynamicScenarioRunner::percentile95(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    std::size_t idx = static_cast<std::size_t>(std::ceil(values.size() * 0.95)) - 1;
    idx = std::min(idx, values.size() - 1);
    return values[idx];
}

}  // namespace sim
