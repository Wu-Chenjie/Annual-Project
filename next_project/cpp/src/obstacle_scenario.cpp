#include "obstacle_scenario.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace sim {

void TelemetryObserver::clear_runtime() {
    waypoint_events_.clear();
    collision_events_.clear();
}

void TelemetryObserver::clear_all() {
    planning_events_.clear();
    clear_runtime();
}

void TelemetryObserver::record_planning(const PlanningEvent& event) {
    planning_events_.push_back(event);
}

void TelemetryObserver::record_waypoint(const WaypointEvent& event) {
    waypoint_events_.push_back(event);
}

void TelemetryObserver::record_collision(const CollisionEvent& event) {
    collision_events_.push_back(event);
}

ObstacleScenarioSimulation::ObstacleScenarioSimulation(const ObstacleConfig& config)
    : config_(config),
      formation_(config),
      apf_(build_apf()),
      formation_apf_(build_formation_apf()),
      num_followers_(config.num_followers) {

    double arm_length = 0.2;
    collision_margin_ = arm_length + config.safety_margin * config.detect_margin_scale;
    task_waypoints_ = config.waypoints;
    waypoints_ = task_waypoints_;
    formation_recovery_counts_.assign(static_cast<std::size_t>(std::max(0, config.num_followers)), 0);
    downwash_zone_ = downwash_zone(config.formation_downwash_radius, config.formation_downwash_height);

    if (config.enable_obstacles && !config.map_file.empty()) {
        setup_obstacles();
        waypoints_ = sanitize_waypoints(config_.waypoints);
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

ImprovedArtificialPotentialField ObstacleScenarioSimulation::build_apf() const {
    double k_rep = 0.8;
    double r_rep = 2.5;
    int n_decay = 2;
    double k_inter = 0.2;
    double s_inter = 2.0;
    double mu_escape = 0.5;
    double max_acc = 8.0;
    double k_comm = 0.0;
    double comm_range = config_.apf_comm_range;
    bool adaptive_n_decay = false;

    if (config_.apf_paper1_profile == "conservative") {
        mu_escape = 0.35;
        k_comm = 0.15;
        adaptive_n_decay = true;
    } else if (config_.apf_paper1_profile == "aggressive") {
        mu_escape = 0.60;
        k_inter = 0.25;
        k_comm = 0.30;
        adaptive_n_decay = true;
    }

    if (config_.apf_dev_override) {
        k_comm = config_.apf_comm_constraint ? 0.3 : 0.0;
        adaptive_n_decay = config_.apf_adaptive_n_decay;
    }

    if (config_.apf_dev_override && config_.apf_rotational_escape) {
        mu_escape = 0.5;
    }

    // 未知地图: APF 同样工作，排斥力来自传感器动态发现的障碍场
    return ImprovedArtificialPotentialField(
        k_rep, r_rep, n_decay, k_inter, s_inter, mu_escape, max_acc,
        k_comm, comm_range, adaptive_n_decay);
}

std::unique_ptr<FormationAPF> ObstacleScenarioSimulation::build_formation_apf() const {
    if (!config_.apf_formation_centroid) return nullptr;
    return std::make_unique<FormationAPF>(
        apf_.k_rep, apf_.r_rep,
        config_.apf_centroid_alpha,
        config_.apf_centroid_beta);
}

void ObstacleScenarioSimulation::set_obstacles(const ObstacleField& field, const std::array<Vec3, 2>& bounds) {
    obstacles_ = field;
    map_bounds_ = bounds;
    task_waypoints_ = config_.waypoints;

    Vec3 extent{bounds[1].x - bounds[0].x, bounds[1].y - bounds[0].y, bounds[1].z - bounds[0].z};
    ObstacleField empty_planning_field;
    const ObstacleField& planning_field = config_.planner_initial_map_unknown ? empty_planning_field : obstacles_;
    grid_ = OccupancyGrid::from_obstacles(planning_field, bounds[0], extent, config_.planner_resolution);
    grid_ = grid_.inflate(inflate_margin_xyz());
    apply_planning_z_bounds();

    if (config_.planner_sdf_aware && !config_.planner_initial_map_unknown) {
        sdf_grid_ = std::make_unique<SDFAwareGrid>(grid_, obstacles_, compute_clearance());
    } else {
        sdf_grid_.reset();
    }

    waypoints_ = sanitize_waypoints(config_.waypoints);
    observer_.clear_all();
    planned_path_.clear();
    auto path = plan_offline();
    if (!path.empty()) {
        planned_path_ = std::move(path);
        if (config_.planner_mode != "online") {
            waypoints_ = planned_path_;
        }
    } else {
        waypoints_ = config_.waypoints;
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
    ObstacleField empty_planning_field;
    const ObstacleField& planning_field = config_.planner_initial_map_unknown ? empty_planning_field : obstacles_;
    grid_ = OccupancyGrid::from_obstacles(planning_field, bounds[0], extent, config_.planner_resolution);
    grid_ = grid_.inflate(inflate_margin_xyz());
    apply_planning_z_bounds();

    if (config_.planner_sdf_aware && !config_.planner_initial_map_unknown) {
        sdf_grid_ = std::make_unique<SDFAwareGrid>(grid_, obstacles_, compute_clearance());
    } else {
        sdf_grid_.reset();
    }
}

double ObstacleScenarioSimulation::inflate_r() const {
    if (!config_.planner_use_formation_envelope) return config_.safety_margin;
    auto [lateral, longitudinal, vertical] = formation_.topology_.envelope_per_axis();
    (void) longitudinal;
    return std::max(lateral, vertical) + config_.safety_margin;
}

std::array<double, 3> ObstacleScenarioSimulation::inflate_margin_xyz() const {
    if (!config_.planner_use_formation_envelope) {
        return {config_.safety_margin, config_.safety_margin, config_.safety_margin};
    }
    auto [lateral, longitudinal, vertical] = formation_.topology_.envelope_per_axis();
    const double margin = config_.safety_margin;
    return {
        lateral + margin,
        longitudinal + margin,
        vertical + margin,
    };
}

double ObstacleScenarioSimulation::compute_clearance() const {
    return std::max(config_.safety_margin, collision_margin_) + config_.plan_clearance_extra;
}

Vec3 ObstacleScenarioSimulation::safe_follower_target(
    const Vec3& leader_pos, const Vec3& raw_target, const Vec3* current_pos) const {
    const double min_clearance = (config_.planner_use_formation_envelope || config_.plan_clearance_extra > 0.0)
        ? std::max(compute_clearance(), collision_margin_ + 0.15)
        : std::max(collision_margin_ + 0.05, config_.safety_margin);
    std::vector<Vec3> candidates;
    candidates.reserve(16);
    for (int i = 10; i >= 0; --i) {
        const double scale = static_cast<double>(i) / 10.0;
        candidates.push_back(leader_pos + (raw_target - leader_pos) * scale);
    }

    try {
        Vec3 prefer = leader_pos - raw_target;
        Vec3 projected = project_to_planning_free(
            raw_target, &prefer, std::max(2.0, norm(raw_target - leader_pos) + 1.0));
        candidates.insert(candidates.begin() + std::min<std::size_t>(1, candidates.size()), projected);
    } catch (...) {
    }

    std::set<std::tuple<int, int, int>> seen;
    for (const auto& candidate : candidates) {
        auto key = std::make_tuple(
            static_cast<int>(std::lround(candidate.x * 10000.0)),
            static_cast<int>(std::lround(candidate.y * 10000.0)),
            static_cast<int>(std::lround(candidate.z * 10000.0)));
        if (!seen.insert(key).second) continue;
        if (planning_signed_distance(candidate) < min_clearance - 1e-8) continue;
        if (current_pos) {
            std::vector<Vec3> segment{*current_pos, candidate};
            if (!path_is_clearance_safe(segment, min_clearance)) continue;
        }
        return candidate;
    }

    if (current_pos && planning_signed_distance(*current_pos) >= 0.0) {
        return *current_pos;
    }
    return leader_pos;
}

Vec3 ObstacleScenarioSimulation::deconflict_follower_target(
    const Vec3& leader_pos,
    const Vec3& candidate_target,
    const Vec3& nominal_target,
    const std::vector<Vec3>& reserved_positions,
    int follower_idx,
    const Vec3* current_pos) const {
    if (!config_.formation_safety_enabled) return candidate_target;

    const double min_clearance = (config_.planner_use_formation_envelope || config_.plan_clearance_extra > 0.0)
        ? std::max(compute_clearance(), collision_margin_ + 0.15)
        : std::max(collision_margin_ + 0.05, config_.safety_margin);
    const double min_inter_distance = config_.formation_min_inter_drone_distance;
    const double vertical_step = std::max(config_.formation_downwash_height * 0.5, 0.25);
    const double lateral_step = std::max(config_.formation_min_inter_drone_distance * 0.75, 0.20);

    bool segment_safe = true;
    if (current_pos) {
        std::vector<Vec3> segment{*current_pos, nominal_target};
        segment_safe = path_is_clearance_safe(segment, min_clearance);
    }
    const bool nominal_ready = nominal_target_ready_for_recovery(
        nominal_target,
        reserved_positions,
        current_pos,
        planning_signed_distance(nominal_target),
        segment_safe,
        min_clearance,
        min_inter_distance,
        &downwash_zone_,
        config_.formation_recovery_clearance_margin);
    if (follower_idx >= 0 && follower_idx < static_cast<int>(formation_recovery_counts_.size())) {
        auto& counter = formation_recovery_counts_[static_cast<std::size_t>(follower_idx)];
        if (nominal_ready) {
            counter += 1;
            if (counter >= config_.formation_recovery_hold_steps) {
                return nominal_target;
            }
        } else {
            counter = 0;
        }
    }

    Vec3 base_dir = candidate_target - leader_pos;
    base_dir.z = 0.0;
    const double base_norm = norm(base_dir);
    Vec3 tangent = (base_norm > 1e-9)
        ? Vec3{-base_dir.y / base_norm, base_dir.x / base_norm, 0.0}
        : Vec3{0.0, 1.0, 0.0};

    std::vector<Vec3> deltas{
        Vec3{},
        tangent * lateral_step,
        tangent * (-lateral_step),
        Vec3{0.0, 0.0, vertical_step},
        Vec3{0.0, 0.0, -vertical_step},
        tangent * lateral_step + Vec3{0.0, 0.0, vertical_step},
        tangent * (-lateral_step) + Vec3{0.0, 0.0, vertical_step},
        tangent * lateral_step + Vec3{0.0, 0.0, -vertical_step},
        tangent * (-lateral_step) + Vec3{0.0, 0.0, -vertical_step},
    };

    for (const auto& delta : deltas) {
        const Vec3 probe = candidate_target + delta;
        if (planning_signed_distance(probe) < min_clearance - 1e-8) continue;
        if (current_pos) {
            std::vector<Vec3> segment{*current_pos, probe};
            if (!path_is_clearance_safe(segment, min_clearance)) continue;
        }
        bool conflict = false;
        for (const auto& reserved : reserved_positions) {
            if (norm(probe - reserved) < min_inter_distance - 1e-8) {
                conflict = true;
                break;
            }
            if (is_in_downwash_zone(probe, reserved, downwash_zone_)
                || is_in_downwash_zone(reserved, probe, downwash_zone_)) {
                conflict = true;
                break;
            }
        }
        if (!conflict) return probe;
    }
    return candidate_target;
}

std::tuple<double, double, double> ObstacleScenarioSimulation::channel_width_from_sensor(
    const std::array<double, 6>* sensor_reading) const {
    if (sensor_reading == nullptr) {
        return {0.0, 0.0, 0.0};
    }
    return {
        (*sensor_reading)[2] + (*sensor_reading)[3],
        (*sensor_reading)[0] + (*sensor_reading)[1],
        (*sensor_reading)[4] + (*sensor_reading)[5],
    };
}

std::vector<Vec3> ObstacleScenarioSimulation::path_window_from_position(
    const std::vector<Vec3>& path, const Vec3& position, double lookahead_distance) const {
    if (path.empty()) return {};
    if (path.size() == 1) return path;

    std::size_t closest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double d = norm(path[i] - position);
        if (d < best) {
            best = d;
            closest = i;
        }
    }

    std::vector<Vec3> window{position};
    double traveled = 0.0;
    Vec3 prev = position;
    const double horizon = std::max(lookahead_distance, grid_.resolution);
    for (std::size_t i = closest + 1; i < path.size(); ++i) {
        const Vec3 point = path[i];
        const double seg_len = norm(point - prev);
        if (seg_len < 1e-9) {
            prev = point;
            continue;
        }
        if (traveled + seg_len >= horizon) {
            const double ratio = std::clamp((horizon - traveled) / seg_len, 0.0, 1.0);
            window.push_back(prev + (point - prev) * ratio);
            break;
        }
        window.push_back(point);
        traveled += seg_len;
        prev = point;
    }
    if (window.size() == 1) {
        window.push_back(path[std::min(closest + 1, path.size() - 1)]);
    }
    return window;
}

double ObstacleScenarioSimulation::path_max_turn_angle(const std::vector<Vec3>& path) const {
    if (path.size() < 3) return 0.0;
    double max_angle = 0.0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const Vec3 left = path[i] - path[i - 1];
        const Vec3 right = path[i + 1] - path[i];
        const double ln = norm(left);
        const double rn = norm(right);
        if (ln < 1e-9 || rn < 1e-9) continue;
        const double cos_angle = std::clamp(dot(left, right) / (ln * rn), -1.0, 1.0);
        max_angle = std::max(max_angle, std::acos(cos_angle));
    }
    return max_angle;
}

double ObstacleScenarioSimulation::online_lookahead_distance(double task_distance) const {
    const double base = std::max({
        config_.leader_max_vel * 1.2,
        config_.wp_radius * 2.0,
        config_.planner_resolution * 3.0,
    });
    const double cap = std::max(config_.planner_resolution * 2.0, config_.planner_horizon * 0.6);
    return std::max(
        config_.planner_resolution,
        std::min({base, cap, std::max(task_distance, config_.planner_resolution)}));
}

Vec3 ObstacleScenarioSimulation::select_online_target(
    const std::vector<Vec3>& path,
    const Vec3& position,
    const Vec3& task_goal,
    int& previous_idx) const {
    if (path.empty()) return task_goal;
    if (path.size() == 1) {
        previous_idx = 0;
        return path[0];
    }

    const double task_distance = norm(task_goal - position);
    if (task_distance <= std::max(config_.wp_radius, config_.planner_resolution)) {
        previous_idx = static_cast<int>(path.size()) - 1;
        return task_goal;
    }

    const double tracking_clearance = std::max(config_.safety_margin, collision_margin_ + 0.03);
    if (path_is_clearance_safe(std::vector<Vec3>{position, task_goal}, tracking_clearance)) {
        previous_idx = static_cast<int>(path.size()) - 1;
        return task_goal;
    }

    const int last_idx = static_cast<int>(path.size()) - 1;
    const int search_start = std::clamp(previous_idx - 6, 0, last_idx);
    int closest_idx = search_start;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int i = search_start; i <= last_idx; ++i) {
        const double distance = norm(path[static_cast<std::size_t>(i)] - position);
        if (distance < best_distance) {
            best_distance = distance;
            closest_idx = i;
        }
    }

    if (closest_idx == last_idx) {
        previous_idx = last_idx;
        return task_goal;
    }

    const double lookahead = online_lookahead_distance(task_distance);
    double remaining = lookahead;
    Vec3 candidate = path[static_cast<std::size_t>(closest_idx)];
    int target_idx = closest_idx;

    for (int idx = closest_idx; idx < last_idx; ++idx) {
        const Vec3 left = path[static_cast<std::size_t>(idx)];
        const Vec3 right = path[static_cast<std::size_t>(idx + 1)];
        const double seg_len = norm(right - left);
        if (seg_len < 1e-9) {
            target_idx = idx + 1;
            candidate = right;
            continue;
        }
        if (remaining <= seg_len) {
            const double ratio = std::clamp(remaining / seg_len, 0.0, 1.0);
            candidate = left + (right - left) * ratio;
            target_idx = idx + 1;
            break;
        }
        remaining -= seg_len;
        candidate = right;
        target_idx = idx + 1;
    }

    auto segment_safe = [&](const Vec3& point) {
        return path_is_clearance_safe(std::vector<Vec3>{position, point}, tracking_clearance);
    };
    if (segment_safe(candidate)) {
        previous_idx = target_idx;
        return candidate;
    }
    for (int i = target_idx; i > closest_idx; --i) {
        const Vec3 point = path[static_cast<std::size_t>(i)];
        if (segment_safe(point)) {
            previous_idx = i;
            return point;
        }
    }

    const int fallback_idx = std::min(closest_idx + 1, last_idx);
    previous_idx = fallback_idx;
    return path[static_cast<std::size_t>(fallback_idx)];
}

bool ObstacleScenarioSimulation::apply_formation_adaptation(
    double time_now,
    const std::tuple<double, double, double>* channel_width,
    double clearance_margin,
    bool has_clearance_margin,
    const std::string& reason_override) {
    if (!config_.formation_adaptation_enabled) return false;

    auto& topology = formation_.topology_;
    const std::string current = topology.current_formation();
    if (last_formation_adaptation_time_ >= 0.0
        && time_now - last_formation_adaptation_time_ < config_.formation_adaptation_min_hold_time) {
        return false;
    }

    auto fits = [](const std::tuple<double, double, double>& width,
                   const std::tuple<double, double, double>& envelope) {
        return std::get<0>(width) >= 2.0 * std::get<0>(envelope)
            && std::get<1>(width) >= 2.0 * std::get<1>(envelope)
            && std::get<2>(width) >= 2.0 * std::get<2>(envelope);
    };
    auto deficit_score = [](const std::tuple<double, double, double>& width,
                            const std::tuple<double, double, double>& envelope) {
        return std::max(2.0 * std::get<0>(envelope) - std::get<0>(width), 0.0)
            + std::max(2.0 * std::get<1>(envelope) - std::get<1>(width), 0.0)
            + std::max(2.0 * std::get<2>(envelope) - std::get<2>(width), 0.0);
    };
    auto compact_score = [](const std::tuple<double, double, double>& envelope) {
        return std::max({std::get<0>(envelope), std::get<1>(envelope), std::get<2>(envelope)});
    };

    const bool clearance_bad = has_clearance_margin && clearance_margin < 0.0;
    const auto current_env = topology.envelope_per_axis(current);
    const bool current_fits = channel_width == nullptr || fits(*channel_width, current_env);
    std::string reason = reason_override.empty()
        ? (clearance_bad ? "clearance_violation" : "channel_too_narrow")
        : reason_override;
    std::string target = current;

    if (clearance_bad || !current_fits) {
        bool found = false;
        double best_score = std::numeric_limits<double>::infinity();
        for (const auto& candidate : config_.formation_adaptation_candidates) {
            auto env = topology.envelope_per_axis(candidate);
            double score = channel_width == nullptr
                ? compact_score(env)
                : (fits(*channel_width, env) ? compact_score(env) : 1000.0 + deficit_score(*channel_width, env));
            if (score < best_score) {
                best_score = score;
                target = candidate;
                found = true;
            }
        }
        if (!found) target = "line";
    } else if (
        current != config_.initial_formation
        && has_clearance_margin
        && clearance_margin >= config_.formation_adaptation_recovery_margin
        && (channel_width == nullptr || fits(*channel_width, topology.envelope_per_axis(config_.initial_formation)))) {
        target = config_.initial_formation;
        reason = "recover_default";
    } else {
        return false;
    }

    if (target == current) return false;

    if (time_now <= 0.0) {
        topology.set_current_formation(target);
    } else {
        topology.switch_formation(
            target,
            std::max(config_.formation_adaptation_transition_time, config_.dt),
            time_now);
    }
    last_formation_adaptation_time_ = time_now;

    FormationAdaptationEvent event;
    event.t = time_now;
    event.from = current;
    event.to = target;
    event.reason = reason;
    if (channel_width != nullptr) {
        event.has_channel_width = true;
        event.channel_width = {std::get<0>(*channel_width), std::get<1>(*channel_width), std::get<2>(*channel_width)};
    }
    const auto selected_env = topology.envelope_per_axis(target);
    event.has_selected_envelope = true;
    event.selected_envelope = {std::get<0>(selected_env), std::get<1>(selected_env), std::get<2>(selected_env)};
    if (has_clearance_margin) {
        event.has_clearance_margin = true;
        event.clearance_margin = clearance_margin;
    }
    formation_adaptation_events_.push_back(event);
    return true;
}

std::vector<Vec3> ObstacleScenarioSimulation::maybe_lookahead_escape(
    double time_now,
    const Vec3& leader_pos,
    const std::vector<Vec3>& active_path,
    const Vec3& task_goal) {
    if (!config_.formation_lookahead_enabled || !config_.formation_lookahead_rrt_enabled) return {};
    if (active_path.size() < 2) return {};
    if (last_lookahead_escape_time_ >= 0.0
        && time_now - last_lookahead_escape_time_ < config_.formation_lookahead_min_interval) {
        return {};
    }

    const double min_clearance = compute_clearance();
    const auto window = path_window_from_position(active_path, leader_pos, config_.formation_lookahead_distance);
    const double max_turn = path_max_turn_angle(window);
    const double clearance = window.size() >= 2 ? path_segment_clearance(window, min_clearance) : min_clearance;
    const double clearance_margin = clearance - min_clearance;
    bool blocked = clearance_margin < 0.0;
    std::string reason = blocked ? "lookahead_reference_blocked" : "lookahead_clear";
    if (max_turn >= config_.formation_lookahead_turn_threshold_rad) {
        blocked = true;
        reason = "lookahead_sharp_turn";
    }
    if (!blocked) return {};

    FormationAdaptationEvent blocked_event;
    blocked_event.t = time_now;
    blocked_event.kind = "lookahead_reference_blocked";
    blocked_event.from = formation_.topology_.current_formation();
    blocked_event.to = blocked_event.from;
    blocked_event.reason = reason;
    blocked_event.has_clearance_margin = true;
    blocked_event.clearance_margin = clearance_margin;
    blocked_event.has_max_turn_angle = true;
    blocked_event.max_turn_angle_rad = max_turn;
    formation_adaptation_events_.push_back(blocked_event);

    apply_formation_adaptation(time_now, nullptr, clearance_margin, true, reason);

    FormationAdaptationEvent attempt;
    attempt.t = time_now;
    attempt.kind = "rrt_escape_attempt";
    attempt.from = formation_.topology_.current_formation();
    attempt.to = attempt.from;
    attempt.reason = reason;
    attempt.planner = "rrt_star_escape";
    attempt.goal_count = 2;
    formation_adaptation_events_.push_back(attempt);

    std::vector<std::pair<std::string, Vec3>> goals;
    if (!window.empty()) goals.push_back({"lookahead_window_end", window.back()});
    goals.push_back({"task_goal", task_goal});
    for (const auto& item : goals) {
        if (norm(item.second - leader_pos) < std::max(grid_.resolution, 1e-6)) continue;
        std::vector<Vec3> candidate{leader_pos, item.second};
        if (!path_is_clearance_safe(candidate, min_clearance)) continue;
        FormationAdaptationEvent accepted;
        accepted.t = time_now;
        accepted.kind = "rrt_escape_accepted";
        accepted.from = formation_.topology_.current_formation();
        accepted.to = accepted.from;
        accepted.reason = reason;
        accepted.planner = "direct_clear_escape";
        accepted.goal_kind = item.first;
        accepted.point_count = static_cast<int>(candidate.size());
        formation_adaptation_events_.push_back(accepted);
        last_lookahead_escape_time_ = time_now;
        observer_.record_planning(PlanningEvent{
            time_now,
            "online_lookahead_rrt_escape",
            "rrt_star_escape",
            -1,
            0.0,
            static_cast<int>(candidate.size()),
            true,
            "",
        });
        return candidate;
    }

    FormationAdaptationEvent failed;
    failed.t = time_now;
    failed.kind = "rrt_escape_failed";
    failed.from = formation_.topology_.current_formation();
    failed.to = failed.from;
    failed.reason = reason;
    failed.planner = "rrt_star_escape";
    failed.goal_count = static_cast<int>(goals.size());
    failed.point_count = 0;
    formation_adaptation_events_.push_back(failed);
    last_lookahead_escape_time_ = time_now;
    observer_.record_planning(PlanningEvent{
        time_now,
        "online_lookahead_rrt_escape",
        "rrt_star_escape",
        -1,
        0.0,
        0,
        false,
        "rrt_escape_failed",
    });
    return {};
}

void ObstacleScenarioSimulation::rebuild_planning_grid() {
    Vec3 extent{
        map_bounds_[1].x - map_bounds_[0].x,
        map_bounds_[1].y - map_bounds_[0].y,
        map_bounds_[1].z - map_bounds_[0].z,
    };
    ObstacleField empty_planning_field;
    const ObstacleField& planning_field = config_.planner_initial_map_unknown ? empty_planning_field : obstacles_;
    grid_ = OccupancyGrid::from_obstacles(planning_field, map_bounds_[0], extent, config_.planner_resolution);
    grid_ = grid_.inflate(inflate_margin_xyz());
    apply_planning_z_bounds();
    if (config_.planner_sdf_aware && !config_.planner_initial_map_unknown) {
        sdf_grid_ = std::make_unique<SDFAwareGrid>(grid_, obstacles_, compute_clearance());
    } else {
        sdf_grid_.reset();
    }
    if (replanner_) {
        const OccupancyGrid& pg = sdf_grid_
            ? static_cast<const OccupancyGrid&>(*sdf_grid_)
            : static_cast<const OccupancyGrid&>(grid_);
        replanner_ = std::make_unique<WindowReplanner>(pg, config_.planner_replan_interval,
                                                       config_.planner_horizon, 0.5, 3);
        replanner_->set_obstacle_field(config_.planner_initial_map_unknown ? &discovered_obstacles_ : &obstacles_);
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
    }
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
        && planning_signed_distance(point) >= min_clearance - 1e-8) {
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
                    double sd = planning_signed_distance(candidate);
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
        auto segment_started = std::chrono::high_resolution_clock::now();
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
            const double wall_time_s = std::chrono::duration<double>(
                std::chrono::high_resolution_clock::now() - segment_started).count();
            observer_.record_planning(PlanningEvent{
                0.0,
                "offline_segment",
                config_.planner_kind,
                static_cast<int>(i),
                wall_time_s,
                0,
                false,
                "planner_failed",
            });
            continue;
        }

        auto smoothed = smooth_path(path_segment, 4);
        smoothed = enforce_path_clearance(smoothed, min_clearance);
        if (config_.firi_enabled && !config_.planner_initial_map_unknown) {
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
        const double wall_time_s = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - segment_started).count();
        observer_.record_planning(PlanningEvent{
            0.0,
            "offline_segment",
            config_.planner_kind,
            static_cast<int>(i),
            wall_time_s,
            static_cast<int>(smoothed.size()),
            smoothed.size() >= 1,
            smoothed.empty() ? "empty_smoothed_path" : "",
        });
        full_path.insert(full_path.end(), smoothed.begin(), smoothed.end());
    }

    return full_path;
}

std::vector<Vec3> ObstacleScenarioSimulation::enforce_path_clearance(
    const std::vector<Vec3>& path, double min_clearance) {

    auto out = path;
    FormationTopology topo(config_.num_followers, config_.formation_spacing);
    auto offsets = topo.get_offsets(config_.initial_formation);

    if (config_.planner_initial_map_unknown) {
        for (auto& wp : out) {
            if (planning_signed_distance(wp) < min_clearance) {
                wp = project_to_planning_free(
                    wp,
                    nullptr,
                    std::max(config_.planner_resolution * 2.0, min_clearance + config_.planner_resolution));
            }
        }
        return out;
    }

    for (auto& wp : out) {
        for (int iter = 0; iter < 60; ++iter) {
            double min_sd = planning_signed_distance(wp);
            Vec3 worst = wp;
            for (const auto& off : offsets) {
                Vec3 check = wp + off;
                double sd = planning_signed_distance(check);
                if (sd < min_sd) { min_sd = sd; worst = check; }
            }
            if (min_sd >= min_clearance) break;

            double eps = 0.05;
            Vec3 grad{
                planning_signed_distance({worst.x + eps, worst.y, worst.z})
                    - planning_signed_distance({worst.x - eps, worst.y, worst.z}),
                planning_signed_distance({worst.x, worst.y + eps, worst.z})
                    - planning_signed_distance({worst.x, worst.y - eps, worst.z}),
                planning_signed_distance({worst.x, worst.y, worst.z + eps})
                    - planning_signed_distance({worst.x, worst.y, worst.z - eps}),
            };
            grad = grad / (2.0 * eps);
            if (!std::isfinite(grad.x) || !std::isfinite(grad.y) || !std::isfinite(grad.z)) break;
            double gn = norm(grad);
            if (gn < 1e-10) break;
            wp = wp + (grad / gn) * 0.10;
        }
    }
    return out;
}

bool ObstacleScenarioSimulation::project_drone_state_to_safe(Drone& drone, double min_clearance) {
    const KinematicState state = drone.get_state();
    const Vec3 pos = state.position;
    const double sd = obstacles_.signed_distance(pos);
    if (sd >= min_clearance) return false;

    const double eps = 0.03;
    Vec3 grad{
        (obstacles_.signed_distance(pos + Vec3{eps, 0.0, 0.0})
         - obstacles_.signed_distance(pos - Vec3{eps, 0.0, 0.0})) / (2.0 * eps),
        (obstacles_.signed_distance(pos + Vec3{0.0, eps, 0.0})
         - obstacles_.signed_distance(pos - Vec3{0.0, eps, 0.0})) / (2.0 * eps),
        (obstacles_.signed_distance(pos + Vec3{0.0, 0.0, eps})
         - obstacles_.signed_distance(pos - Vec3{0.0, 0.0, eps})) / (2.0 * eps),
    };

    Vec3 normal{};
    Vec3 corrected = pos;
    const double grad_norm = norm(grad);
    if (std::isfinite(grad.x) && std::isfinite(grad.y) && std::isfinite(grad.z)
        && grad_norm >= 1e-9) {
        normal = grad / grad_norm;
        corrected = pos + normal * (min_clearance - sd + 1e-3);
    } else {
        const double search_radius = std::max(1.0, min_clearance + 0.8);
        const double step = std::max(0.05, std::min(config_.planner_resolution, 0.2));
        const int max_steps = std::max(1, static_cast<int>(std::ceil(search_radius / step)));
        double best_score = std::numeric_limits<double>::infinity();
        bool found = false;

        for (int radius = 1; radius <= max_steps; ++radius) {
            for (int ix = -radius; ix <= radius; ++ix) {
                for (int iy = -radius; iy <= radius; ++iy) {
                    for (int iz = -radius; iz <= radius; ++iz) {
                        if (std::max({std::abs(ix), std::abs(iy), std::abs(iz)}) != radius) {
                            continue;
                        }
                        Vec3 candidate = pos + Vec3{
                            static_cast<double>(ix) * step,
                            static_cast<double>(iy) * step,
                            static_cast<double>(iz) * step,
                        };
                        if (obstacles_.signed_distance(candidate) < min_clearance) continue;
                        Vec3 delta = candidate - pos;
                        const double score = norm(delta) + 0.05 * std::abs(delta.z);
                        if (score < best_score) {
                            best_score = score;
                            corrected = candidate;
                            normal = normalized(delta);
                            found = true;
                        }
                    }
                }
            }
            if (found) break;
        }
        if (!found) return false;
    }

    Vec3 velocity = state.velocity;
    const double normal_vel = dot(velocity, normal);
    if (normal_vel < 0.0) {
        velocity -= normal * normal_vel;
    }
    drone.set_initial_state(corrected, velocity, state.attitude, state.angular_velocity, drone.dt());
    return true;
}

double ObstacleScenarioSimulation::path_segment_clearance(
    const std::vector<Vec3>& path, double min_clearance) const {
    if (path.empty()) return -std::numeric_limits<double>::infinity();
    if (path.size() == 1) return planning_signed_distance(path.front());

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
            worst = std::min(worst, planning_signed_distance(p));
            if (worst < min_clearance) return worst;
        }
    }
    return worst;
}

bool ObstacleScenarioSimulation::path_is_clearance_safe(
    const std::vector<Vec3>& path, double min_clearance) const {
    return path_segment_clearance(path, min_clearance) >= min_clearance - 1e-6;
}

double ObstacleScenarioSimulation::planning_signed_distance(const Vec3& point) const {
    if (config_.planner_initial_map_unknown) {
        auto idx = grid_.world_to_index(point);
        return grid_.is_occupied(idx[0], idx[1], idx[2])
            ? -grid_.resolution
            : std::numeric_limits<double>::infinity();
    }
    return obstacles_.signed_distance(point);
}

ObstacleField ObstacleScenarioSimulation::build_discovered_obstacle_field() const {
    ObstacleField field;
    if (grid_.data.empty()) return field;

    const int nx = grid_.nx, ny = grid_.ny, nz = grid_.nz;
    std::vector<int> labels(static_cast<std::size_t>(nx) * ny * nz, 0);
    int current_label = 0;

    // 3D 6-connected component labelling
    const int di[] = {-1, 1, 0, 0, 0, 0};
    const int dj[] = {0, 0, -1, 1, 0, 0};
    const int dk[] = {0, 0, 0, 0, -1, 1};

    for (int iz = 0; iz < nz; ++iz) {
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                const std::size_t idx = (static_cast<std::size_t>(iz) * ny + iy) * nx + ix;
                if (grid_.data[idx] < 1 || labels[idx] != 0) continue;

                ++current_label;
                std::vector<std::size_t> q{idx};
                labels[idx] = current_label;

                for (std::size_t qi = 0; qi < q.size(); ++qi) {
                    const std::size_t ci = q[qi];
                    const int cx = static_cast<int>(ci % nx);
                    const int cy = static_cast<int>((ci / nx) % ny);
                    const int cz = static_cast<int>(ci / (nx * ny));

                    for (int d = 0; d < 6; ++d) {
                        const int nx_idx = cx + di[d];
                        const int ny_idx = cy + dj[d];
                        const int nz_idx = cz + dk[d];
                        if (nx_idx < 0 || nx_idx >= nx || ny_idx < 0 || ny_idx >= ny || nz_idx < 0 || nz_idx >= nz)
                            continue;
                        const std::size_t nidx = (static_cast<std::size_t>(nz_idx) * ny + ny_idx) * nx + nx_idx;
                        if (grid_.data[nidx] >= 1 && labels[nidx] == 0) {
                            labels[nidx] = current_label;
                            q.push_back(nidx);
                        }
                    }
                }
            }
        }
    }

    // Build AABB per component
    const double res = grid_.resolution;
    std::vector<int> imin(current_label, nx + ny + nz);
    std::vector<int> jmin(current_label, nx + ny + nz);
    std::vector<int> kmin(current_label, nx + ny + nz);
    std::vector<int> imax(current_label, -1);
    std::vector<int> jmax(current_label, -1);
    std::vector<int> kmax(current_label, -1);

    for (int iz = 0; iz < nz; ++iz) {
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                const std::size_t idx = (static_cast<std::size_t>(iz) * ny + iy) * nx + ix;
                const int lbl = labels[idx];
                if (lbl == 0) continue;
                const int li = lbl - 1;
                if (ix < imin[li]) imin[li] = ix;
                if (iy < jmin[li]) jmin[li] = iy;
                if (iz < kmin[li]) kmin[li] = iz;
                if (ix > imax[li]) imax[li] = ix;
                if (iy > jmax[li]) jmax[li] = iy;
                if (iz > kmax[li]) kmax[li] = iz;
            }
        }
    }

    for (int li = 0; li < current_label; ++li) {
        if (imax[li] < 0) continue;
        Vec3 min_world = grid_.index_to_world(imin[li], jmin[li], kmin[li]);
        Vec3 max_world = grid_.index_to_world(imax[li], jmax[li], kmax[li]) + Vec3{res, res, res};
        field.add_aabb(min_world, max_world);
    }

    return field;
}

void ObstacleScenarioSimulation::update_discovered_obstacles() {
    if (!config_.planner_initial_map_unknown) return;
    // Sync scenario grid with replanner's sensor-updated grid so all
    // downstream consumers (planning_signed_distance, clearance checks,
    // project_to_planning_free, etc.) see the latest sensor data.
    if (replanner_) {
        const auto& sg = replanner_->current_grid();
        if (sg.data.size() == grid_.data.size()) {
            grid_.data = sg.data;
        } else if (!sg.data.empty()) {
            fault_log_.push_back(
                "grid_size_mismatch:scenario_" + std::to_string(grid_.data.size())
                + ":replanner_" + std::to_string(sg.data.size())
                + ":discovered_obstacles_stale");
        }
    }
    discovered_obstacles_ = build_discovered_obstacle_field();
}

Vec3 ObstacleScenarioSimulation::obstacle_repulsion_acc(
    const Vec3& pos, const Vec3& goal, const std::vector<Vec3>& other_positions) {
    const ObstacleField& avoid_field = config_.planner_initial_map_unknown
        ? discovered_obstacles_ : obstacles_;
    return apf_.compute_avoidance_acceleration(pos, goal, avoid_field, other_positions);
}

void ObstacleScenarioSimulation::setup_online() {
    if (grid_.data.empty()) return;

    const OccupancyGrid& pg = sdf_grid_
        ? static_cast<const OccupancyGrid&>(*sdf_grid_)
        : static_cast<const OccupancyGrid&>(grid_);
    replanner_ = std::make_unique<WindowReplanner>(pg, config_.planner_replan_interval,
                                                    config_.planner_horizon, 0.5, 3);
    replanner_->set_obstacle_field(config_.planner_initial_map_unknown ? &discovered_obstacles_ : &obstacles_);
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

std::vector<Vec3> ObstacleScenarioSimulation::planned_segment_for_task(
    const Vec3& start, const Vec3& task_goal) const {
    if (planned_path_.size() < 2) return {};

    int start_i = 0;
    double start_best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < planned_path_.size(); ++i) {
        const double d = norm(planned_path_[i] - start);
        if (d < start_best) { start_best = d; start_i = static_cast<int>(i); }
    }

    int goal_i = 0;
    double goal_best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < planned_path_.size(); ++i) {
        const double d = norm(planned_path_[i] - task_goal);
        if (d < goal_best) { goal_best = d; goal_i = static_cast<int>(i); }
    }

    std::vector<Vec3> segment;
    segment.push_back(start);
    if (goal_i <= start_i) {
        segment.push_back(task_goal);
    } else {
        for (int i = start_i + 1; i <= goal_i; ++i) {
            segment.push_back(planned_path_[static_cast<std::size_t>(i)]);
        }
        const double tail_dist = norm(segment.back() - task_goal);
        if (tail_dist > std::max(config_.wp_radius * 0.5, config_.planner_resolution)) {
            segment.push_back(task_goal);
        }
    }
    return segment;
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
    formation_adaptation_events_.clear();
    last_formation_adaptation_time_ = -1.0;
    last_lookahead_escape_time_ = -1.0;
    observer_.clear_runtime();
    std::fill(formation_recovery_counts_.begin(), formation_recovery_counts_.end(), 0);

    auto& leader = formation_.leader_;
    auto& leader_ctrl = formation_.leader_ctrl_;
    auto& followers = formation_.followers_;
    auto& follower_ctrls = formation_.follower_ctrls_;
    auto& leader_wind = formation_.leader_wind_;
    auto& winds = formation_.winds_;
    auto& topology = formation_.topology_;

    bool online = config_.planner_mode == "online" && replanner_ != nullptr;
    const std::vector<Vec3> execution_tasks = waypoints_.empty() ? config_.waypoints : waypoints_;
    const std::vector<Vec3>& reported_tasks = task_waypoints_.empty() ? execution_tasks : task_waypoints_;
    std::vector<Vec3> active_path = online ? execution_tasks : waypoints_;
    int task_wp_idx = 0;
    int local_wp_idx = 0;
    bool finished = online ? execution_tasks.empty() : active_path.empty();
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

        if (online && !finished && task_wp_idx < static_cast<int>(execution_tasks.size())) {
            std::array<double, 6> sdata{};
            const std::array<double, 6>* sp = nullptr;
            if (sensor_) {
                sdata = sensor_->sense(ls_before_replan.position, obstacles_);
                sp = &sdata;
                // 未知模式：每步持续将传感器读数注入栅格并重建动态障碍场
                if (config_.planner_initial_map_unknown) {
                    replanner_->observe_sensor(ls_before_replan.position, *sp);
                    update_discovered_obstacles();
                }
                if (config_.formation_adaptation_enabled) {
                    auto channel_width = channel_width_from_sensor(sp);
                    if (apply_formation_adaptation(t, &channel_width, 0.0, false)) {
                        rebuild_planning_grid();
                    }
                } else if (config_.planner_use_formation_envelope) {
                    const std::string before = topology.current_formation();
                    if (topology.auto_shrink(channel_width_from_sensor(sp)) && topology.current_formation() != before) {
                        rebuild_planning_grid();
                    }
                }
            }

            Vec3 task_goal = execution_tasks[task_wp_idx];
            auto replan_started = std::chrono::high_resolution_clock::now();
            auto new_path = maybe_lookahead_escape(t, ls_before_replan.position, active_path, task_goal);
            const bool lookahead_used = !new_path.empty();
            if (!lookahead_used) {
                const std::array<double, 6>* replan_sensor = config_.planner_initial_map_unknown ? nullptr : sp;
                new_path = replanner_->step(t, ls_before_replan.position, replan_sensor, task_goal);
                if (config_.planner_initial_map_unknown) {
                    update_discovered_obstacles();
                }
            }
            if (!new_path.empty()) {
                update_discovered_obstacles();
                auto candidate_path = stitch_local_path_to_task_goal(new_path, task_goal);
                const double clearance = compute_clearance();
                if (config_.firi_enabled && !config_.planner_initial_map_unknown && candidate_path.size() >= 2) {
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
                const double replan_wall_time_s = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now() - replan_started).count();
                if (!lookahead_used) {
                    observer_.record_planning(PlanningEvent{
                        t,
                        "online_replan",
                        config_.planner_kind,
                        task_wp_idx,
                        replan_wall_time_s,
                        static_cast<int>(candidate_path.size()),
                        candidate_safe,
                        candidate_safe ? "" : "clearance_rejected",
                    });
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

        auto ls0 = leader.get_state();
        Vec3 target{};
        if (finished) {
            target = online
                ? (execution_tasks.empty() ? Vec3{} : execution_tasks.back())
                : (active_path.empty() ? Vec3{} : active_path.back());
        } else if (online) {
            if (task_wp_idx < static_cast<int>(execution_tasks.size())) {
                target = select_online_target(active_path, ls0.position, execution_tasks[task_wp_idx], local_wp_idx);
            } else {
                target = execution_tasks.back();
            }
        } else if (local_wp_idx < static_cast<int>(active_path.size())) {
            target = active_path[local_wp_idx];
        } else {
            target = active_path.empty() ? Vec3{} : active_path.back();
        }

        std::vector<Vec3> follower_positions_now;
        follower_positions_now.reserve(followers.size());
        for (const auto& follower : followers) {
            follower_positions_now.push_back(follower.get_state().position);
        }
        auto offsets = topology.get_current_offsets(t);
        Vec3 formation_leader_acc{};
        std::vector<Vec3> formation_follower_accs(followers.size(), Vec3{});
        const ObstacleField& apf_field = config_.planner_initial_map_unknown
            ? discovered_obstacles_ : obstacles_;
        if (formation_apf_ && !follower_positions_now.empty()) {
            auto formation_forces = formation_apf_->compute_formation_avoidance(
                ls0.position, follower_positions_now, target, apf_field, offsets);
            formation_leader_acc = formation_forces.first;
            formation_follower_accs = std::move(formation_forces.second);
        }
        std::vector<Vec3> leader_other_positions;
        leader_other_positions.reserve(followers.size());
        leader_other_positions = follower_positions_now;
        Vec3 rep_acc = obstacle_repulsion_acc(ls0.position, target, leader_other_positions);
        rep_acc += formation_leader_acc;

        Vec3 leader_target_vel{};
        if (online && !finished) {
            const Vec3 to_target = target - ls0.position;
            const double to_target_dist = norm(to_target);
            if (to_target_dist > 1e-9) {
                const double speed_cmd = std::min(
                    config_.leader_max_vel,
                    std::max(0.0, (to_target_dist - 0.25 * config_.wp_radius) / std::max(dt * 8.0, 1e-6)));
                leader_target_vel = to_target * (speed_cmd / to_target_dist);
            }
        }
        auto u = leader_ctrl->compute_control(leader.state(), target, leader_target_vel, rep_acc);
        leader.update_state(u, leader_wind.sample(dt));
        project_drone_state_to_safe(leader, collision_margin_);

        auto ls = leader.get_state();
        result.leader[step_idx] = ls.position;
        executed_path_.push_back(ls.position);

        Vec3 leader_acc{(ls.velocity.x - ls0.velocity.x) / dt,
                        (ls.velocity.y - ls0.velocity.y) / dt,
                        (ls.velocity.z - ls0.velocity.z) / dt};
        leader_acc_filt = leader_acc * alpha + leader_acc_filt * (1.0 - alpha);

        if (obstacles_.is_collision(ls.position, collision_margin_)) {
            CollisionEvent event{t, "leader", ls.position};
            collision_log_.push_back(event);
            observer_.record_collision(event);
        }

        if (!finished) {
            if (online && task_wp_idx < static_cast<int>(execution_tasks.size())) {
                double task_radius = (task_wp_idx == static_cast<int>(execution_tasks.size()) - 1)
                    ? config_.wp_radius_final
                    : config_.wp_radius;
                double distance = norm(execution_tasks[task_wp_idx] - ls.position);
                if (distance < task_radius) {
                    observer_.record_waypoint(WaypointEvent{
                        t,
                        "waypoint_reached",
                        task_wp_idx,
                        distance,
                    });
                    ++task_wp_idx;
                    local_wp_idx = 0;
                    active_path = (task_wp_idx < static_cast<int>(execution_tasks.size()))
                        ? std::vector<Vec3>{execution_tasks[task_wp_idx]}
                        : std::vector<Vec3>{execution_tasks.back()};
                    if (task_wp_idx >= static_cast<int>(execution_tasks.size())) finished = true;
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
                double distance = norm(active_path[local_wp_idx] - ls.position);
                if (distance < radius) {
                    observer_.record_waypoint(WaypointEvent{
                        t,
                        "path_point_reached",
                        local_wp_idx,
                        distance,
                    });
                    ++local_wp_idx;
                    if (local_wp_idx >= static_cast<int>(active_path.size())) finished = true;
                }
            }
        }

        std::vector<Vec3> reserved_targets;
        reserved_targets.reserve(static_cast<std::size_t>(follower_count) + 1);
        reserved_targets.push_back(ls.position);
        for (int i = 0; i < follower_count; ++i) {
            Vec3 follower_pos = followers[i].get_state().position;
            Vec3 nominal_target = ls.position + offsets[i];
            Vec3 target_pos = safe_follower_target(ls.position, nominal_target, &follower_pos);
            target_pos = deconflict_follower_target(
                ls.position,
                target_pos,
                nominal_target,
                reserved_targets,
                i,
                &follower_pos);
            reserved_targets.push_back(target_pos);
            std::vector<Vec3> other_positions;
            other_positions.reserve(followers.size());
            other_positions.push_back(ls.position);
            for (int j = 0; j < follower_count; ++j) {
                if (j == i) continue;
                other_positions.push_back(followers[j].get_state().position);
            }
            Vec3 rep_acc_f = obstacle_repulsion_acc(follower_pos, target_pos, other_positions);
            if (static_cast<std::size_t>(i) < formation_follower_accs.size()) {
                rep_acc_f += formation_follower_accs[static_cast<std::size_t>(i)];
            }
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
            project_drone_state_to_safe(followers[i], collision_margin_);

            auto fs = followers[i].get_state();
            Vec3 err_v = fs.position - target_pos;
            result.targets[i][step_idx] = target_pos;
            result.error_vectors[i][step_idx] = err_v;
            result.errors[i][step_idx] = norm(err_v);
            result.followers[i][step_idx] = fs.position;

            if (obstacles_.is_collision(fs.position, collision_margin_)) {
                CollisionEvent event{t, "follower_" + std::to_string(i), fs.position};
                collision_log_.push_back(event);
                observer_.record_collision(event);
            }
        }

        result.time[step_idx] = t;
        ++step_idx;
        t += dt;
    }

    result.completed_waypoint_count = online ? task_wp_idx : local_wp_idx;
    result.waypoints = waypoints_;
    result.task_waypoints = reported_tasks;
    result.planned_path = planned_path_;
    result.replanned_waypoints = replanned_waypoints_;
    result.executed_path = executed_path_;
    result.planning_events = observer_.planning_events();
    result.waypoint_events = observer_.waypoint_events();
    result.collision_log = observer_.collision_events();
    result.formation_adaptation_events = formation_adaptation_events_;
    result.fault_log = fault_log_;
    int valid = step_idx;

    if (follower_count > 0 && valid > 0) {
        double min_pair_distance = std::numeric_limits<double>::infinity();
        int downwash_hits = 0;
        for (int step = 0; step < valid; ++step) {
            std::vector<Vec3> poses;
            poses.reserve(static_cast<std::size_t>(follower_count) + 1);
            poses.push_back(result.leader[step]);
            for (int i = 0; i < follower_count; ++i) {
                poses.push_back(result.followers[i][step]);
            }
            min_pair_distance = std::min(min_pair_distance, min_inter_drone_distance(poses));
            if (config_.formation_safety_enabled) {
                for (std::size_t upper_i = 0; upper_i < poses.size(); ++upper_i) {
                    for (std::size_t lower_i = 0; lower_i < poses.size(); ++lower_i) {
                        if (upper_i == lower_i) continue;
                        if (is_in_downwash_zone(poses[upper_i], poses[lower_i], downwash_zone_)) {
                            downwash_hits += 1;
                        }
                    }
                }
            }
        }
        result.safety_metrics.min_inter_drone_distance = std::isfinite(min_pair_distance) ? min_pair_distance : 0.0;
        result.safety_metrics.downwash_hits = downwash_hits;
    }

    result.metrics.mean.resize(follower_count);
    result.metrics.max.resize(follower_count);
    result.metrics.final.resize(follower_count);
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
