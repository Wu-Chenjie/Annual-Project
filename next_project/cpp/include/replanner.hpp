#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "astar_planner.hpp"
#include "dual_mode.hpp"
#include "dstar_lite.hpp"
#include "gnn_planner.hpp"
#include "sensor.hpp"

namespace sim {

struct ReplanEvent { double t; std::string phase; Vec3 from, to; std::string reason; };

class WindowReplanner {
public:
    WindowReplanner(const OccupancyGrid& grid, double interval = 0.4, double horizon = 6.0,
                    double epsilon = 0.5, int local_fail_threshold = 3);

    std::vector<Vec3> step(double t, const Vec3& leader_pose,
                           const std::array<double, 6>* sensor_reading,
                           const Vec3& goal);

    void set_global_planner(std::unique_ptr<HybridAStarPlanner> gp) { global_planner_ = std::move(gp); }
    void set_incremental_planner(std::unique_ptr<DStarLite> ip) { incremental_planner_ = std::move(ip); }
    void set_obstacle_field(const ObstacleField* field) { obstacle_field_ = field; }
    void set_danger_planner(std::unique_ptr<GNNPlanner> dp) { danger_planner_ = std::move(dp); }
    void set_dual_mode(std::unique_ptr<DualModeScheduler> dm) { dual_mode_ = std::move(dm); }
    void enable_adaptive_interval(double min_interval, double max_interval);

    [[nodiscard]] const std::vector<ReplanEvent>& events() const { return events_; }
    [[nodiscard]] double current_interval() const;
    [[nodiscard]] double last_risk() const { return last_risk_; }
    std::vector<ReplanEvent> get_new_events();

private:
    enum Phase { PHASE_LOCAL, PHASE_INCREMENTAL, PHASE_GLOBAL, PHASE_DANGER };

    std::vector<Vec3> replan_local(const Vec3& pose, const Vec3& goal);
    std::vector<Vec3> replan_incremental(const Vec3& pose, const Vec3& goal);
    std::vector<Vec3> replan_global(const Vec3& pose, const Vec3& goal);
    std::vector<Vec3> replan_danger(const Vec3& pose, const Vec3& goal);
    Phase classify_phase(const Vec3& pose);
    [[nodiscard]] const char* phase_name() const;
    bool ref_path_feasible_in_horizon(const Vec3& pose) const;
    Vec3 compute_subgoal(const Vec3& pose, const Vec3& goal) const;
    std::vector<Vec3> publish_path(double t, std::vector<Vec3> new_path);
    void decay_sensor_obstacles();
    std::vector<std::pair<std::array<int,3>, bool>> update_grid_from_sensor(
        const Vec3& pose, const std::array<double, 6>& readings);
    void update_risk(const Vec3& pose, const std::array<double, 6>* readings);
    double path_deviation(const std::vector<Vec3>& old_p, const std::vector<Vec3>& new_p) const;
    [[nodiscard]] int flat_index(const std::array<int,3>& idx) const;

    OccupancyGrid mutable_grid_;
    double interval_, horizon_, epsilon_;
    double adaptive_min_interval_ = 0.1;
    double adaptive_max_interval_ = 1.0;
    bool adaptive_interval_enabled_ = false;
    double last_risk_ = 0.0;
    std::vector<double> risk_history_;
    int local_fail_threshold_, local_fail_count_ = 0;
    std::vector<Vec3> current_path_, global_ref_path_;
    double last_replan_time_ = -1e9;
    int events_consumed_ = 0;
    std::vector<ReplanEvent> events_;
    Phase phase_ = PHASE_LOCAL;
    int step_count_ = 0;

    std::unique_ptr<HybridAStarPlanner> global_planner_;
    std::unique_ptr<DStarLite> incremental_planner_;
    std::unique_ptr<GNNPlanner> danger_planner_;
    std::unique_ptr<DualModeScheduler> dual_mode_;
    const ObstacleField* obstacle_field_ = nullptr;
    std::vector<std::uint8_t> static_occupied_;
    std::vector<std::uint8_t> sensor_occupied_;
    std::vector<int> sensor_ttl_;
    std::vector<int> sensor_clear_hits_;
    int sensor_obstacle_ttl_steps_ = 3;
    int sensor_clear_confirm_steps_ = 1;
};

// ====================================================================
// Implementation
// ====================================================================

inline WindowReplanner::WindowReplanner(const OccupancyGrid& grid, double interval,
                                         double horizon, double epsilon, int lft)
    : mutable_grid_(grid), interval_(interval), horizon_(horizon), epsilon_(epsilon),
      local_fail_threshold_(lft) {
    static_occupied_.resize(mutable_grid_.data.size(), 0);
    sensor_occupied_.resize(mutable_grid_.data.size(), 0);
    sensor_ttl_.resize(mutable_grid_.data.size(), 0);
    sensor_clear_hits_.resize(mutable_grid_.data.size(), 0);
    for (std::size_t i = 0; i < mutable_grid_.data.size(); ++i) {
        static_occupied_[i] = mutable_grid_.data[i] >= 1 ? 1 : 0;
    }
}

inline void WindowReplanner::enable_adaptive_interval(double min_interval, double max_interval) {
    adaptive_interval_enabled_ = true;
    adaptive_min_interval_ = std::max(1e-6, std::min(min_interval, max_interval));
    adaptive_max_interval_ = std::max(adaptive_min_interval_, max_interval);
}

inline double WindowReplanner::current_interval() const {
    if (!adaptive_interval_enabled_ || risk_history_.empty()) return interval_;
    double avg_risk = std::accumulate(risk_history_.begin(), risk_history_.end(), 0.0)
                    / static_cast<double>(risk_history_.size());
    avg_risk = clamp(avg_risk, 0.0, 1.0);
    return adaptive_max_interval_ - (adaptive_max_interval_ - adaptive_min_interval_) * avg_risk;
}

inline std::vector<ReplanEvent> WindowReplanner::get_new_events() {
    std::vector<ReplanEvent> out(events_.begin() + events_consumed_, events_.end());
    events_consumed_ = static_cast<int>(events_.size());
    return out;
}

inline std::vector<Vec3> WindowReplanner::step(
    double t, const Vec3& leader_pose, const std::array<double, 6>* sensor_reading, const Vec3& goal) {
    update_risk(leader_pose, sensor_reading);
    double interval_now = current_interval();
    if (t - last_replan_time_ < interval_now) return {};
    last_replan_time_ = t;
    ++step_count_;

    decay_sensor_obstacles();
    if (sensor_reading) update_grid_from_sensor(leader_pose, *sensor_reading);

    std::vector<Vec3> new_path;

    if (dual_mode_ && danger_planner_ && obstacle_field_) {
        std::string mode = dual_mode_->classify(t, sensor_reading, obstacle_field_, leader_pose);
        if (mode == "danger") {
            phase_ = PHASE_DANGER;
            try {
                new_path = replan_danger(leader_pose, goal);
                if (!new_path.empty()) {
                    local_fail_count_ = 0;
                    return publish_path(t, new_path);
                }
                ++local_fail_count_;
            } catch (...) {
                ++local_fail_count_;
            }
        }
    }

    Phase ph = classify_phase(leader_pose);
    phase_ = ph;

    try {
        if (ph == PHASE_GLOBAL && global_planner_) {
            new_path = replan_global(leader_pose, goal);
            if (!new_path.empty()) local_fail_count_ = 0;
        } else if (ph == PHASE_INCREMENTAL && incremental_planner_) {
            new_path = replan_incremental(leader_pose, goal);
            if (!new_path.empty()) local_fail_count_ = 0;
            else ++local_fail_count_;
        } else {
            new_path = replan_local(leader_pose, goal);
            if (!new_path.empty()) local_fail_count_ = 0;
            else ++local_fail_count_;
        }
    } catch (...) { ++local_fail_count_; return {}; }

    return new_path.empty() ? std::vector<Vec3>{} : publish_path(t, new_path);
}

inline WindowReplanner::Phase WindowReplanner::classify_phase(const Vec3& pose) {
    if (global_planner_ && global_ref_path_.empty()) return PHASE_GLOBAL;
    if (local_fail_count_ >= local_fail_threshold_) {
        if (global_planner_) return PHASE_GLOBAL;
    }
    if (!global_ref_path_.empty()) {
        if (!ref_path_feasible_in_horizon(pose)) {
            if (incremental_planner_) return PHASE_INCREMENTAL;
            if (global_planner_) return PHASE_GLOBAL;
        }
    }
    return PHASE_LOCAL;
}

inline bool WindowReplanner::ref_path_feasible_in_horizon(const Vec3& pose) const {
    if (global_ref_path_.empty()) return false;
    for (const auto& wp : global_ref_path_) {
        if (norm(wp - pose) > horizon_) break;
        auto idx = mutable_grid_.world_to_index(wp);
        if (mutable_grid_.data[(idx[2]*mutable_grid_.ny+idx[1])*mutable_grid_.nx+idx[0]] >= 1) return false;
    }
    return true;
}

inline Vec3 WindowReplanner::compute_subgoal(const Vec3& pose, const Vec3& goal) const {
    double d = norm(goal - pose);
    if (d > horizon_) return pose + (goal - pose) / d * horizon_;
    return goal;
}

inline std::vector<Vec3> WindowReplanner::replan_local(const Vec3& pose, const Vec3& goal) {
    Vec3 sg = compute_subgoal(pose, goal);
    auto res = astar_plan(pose, sg, mutable_grid_);
    if (!res.success || res.path.size() < 2) return {};
    return smooth_path(res.path, 4);
}

inline std::vector<Vec3> WindowReplanner::replan_incremental(const Vec3& pose, const Vec3& goal) {
    if (!incremental_planner_) return replan_local(pose, goal);
    incremental_planner_->update_start(pose);
    incremental_planner_->compute_shortest_path();
    if (!incremental_planner_->is_consistent()) return {};
    auto path = incremental_planner_->extract_path();
    if (path.size() < 2) return {};
    return smooth_path(path, 4);
}

inline std::vector<Vec3> WindowReplanner::replan_global(const Vec3& pose, const Vec3& goal) {
    if (!global_planner_) return {};
    auto res = global_planner_->plan(pose, goal, mutable_grid_, step_count_ * 42);
    if (!res.success) return {};
    global_ref_path_ = res.path;
    return smooth_path(global_ref_path_, 4);
}

inline std::vector<Vec3> WindowReplanner::replan_danger(const Vec3& pose, const Vec3& goal) {
    if (!danger_planner_ || !obstacle_field_) return {};
    auto path = danger_planner_->plan(pose, goal, mutable_grid_, *obstacle_field_, horizon_ * 4.0);
    if (path.size() < 2) return {};
    return path;
}

inline const char* WindowReplanner::phase_name() const {
    if (phase_ == PHASE_LOCAL) return "local";
    if (phase_ == PHASE_INCREMENTAL) return "incremental";
    if (phase_ == PHASE_GLOBAL) return "global";
    return "danger";
}

inline std::vector<Vec3> WindowReplanner::publish_path(double t, std::vector<Vec3> new_path) {
    if (!current_path_.empty() && !new_path.empty()) {
        double dev = path_deviation(current_path_, new_path);
        if (dev > epsilon_) {
            events_.push_back({t, phase_name(),
                               current_path_.empty() ? Vec3{} : current_path_[0], new_path[0],
                               "deviation=" + std::to_string(dev) + "m"});
        }
    } else if (current_path_.empty() && !new_path.empty()) {
        events_.push_back({t, phase_name(),
                           Vec3{}, new_path[0], "initial_publish"});
    }
    current_path_ = new_path;
    return new_path;
}

inline std::vector<std::pair<std::array<int,3>, bool>> WindowReplanner::update_grid_from_sensor(
    const Vec3& pose, const std::array<double, 6>& readings) {
    static const std::array<Vec3, 6> dirs{Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,1,0},Vec3{0,-1,0},Vec3{0,0,1},Vec3{0,0,-1}};
    std::vector<std::pair<std::array<int,3>, bool>> changed;
    for (int di = 0; di < 6; ++di) {
        double r = std::min(readings[di], horizon_);
        int ns = std::max(1, static_cast<int>(r / mutable_grid_.resolution));
        for (int s = 0; s <= ns; ++s) {
            double ts = s * mutable_grid_.resolution;
            if (ts > r) break;
            Vec3 pt = pose + dirs[di] * ts;
            auto idx = mutable_grid_.world_to_index(pt);
            if (idx[0]<0||idx[0]>=mutable_grid_.nx||idx[1]<0||idx[1]>=mutable_grid_.ny||idx[2]<0||idx[2]>=mutable_grid_.nz) continue;
            int flat = flat_index(idx);
            if (flat < 0 || static_occupied_[flat]) continue;
            bool hit_cell = ts >= r - mutable_grid_.resolution;
            auto& cell = mutable_grid_.data[flat];
            if (hit_cell) {
                sensor_ttl_[flat] = sensor_obstacle_ttl_steps_;
                sensor_clear_hits_[flat] = 0;
                if (cell != 1 || !sensor_occupied_[flat]) {
                    cell = 1;
                    sensor_occupied_[flat] = 1;
                    changed.push_back({idx, true});
                }
            } else if (sensor_occupied_[flat] && cell == 1) {
                sensor_clear_hits_[flat] = std::min(sensor_clear_confirm_steps_, sensor_clear_hits_[flat] + 1);
                if (sensor_clear_hits_[flat] >= sensor_clear_confirm_steps_) {
                    sensor_ttl_[flat] = std::max(0, sensor_ttl_[flat] - 1);
                    if (sensor_ttl_[flat] <= 0) {
                        cell = 0;
                        sensor_occupied_[flat] = 0;
                        sensor_clear_hits_[flat] = 0;
                        changed.push_back({idx, false});
                    }
                }
            }
        }
    }
    return changed;
}

inline void WindowReplanner::decay_sensor_obstacles() {
    for (std::size_t flat = 0; flat < sensor_occupied_.size(); ++flat) {
        if (!sensor_occupied_[flat]) continue;
        if (static_occupied_[flat]) {
            sensor_occupied_[flat] = 0;
            sensor_ttl_[flat] = 0;
            sensor_clear_hits_[flat] = 0;
            continue;
        }
        sensor_ttl_[flat] = std::max(0, sensor_ttl_[flat] - 1);
        if (sensor_ttl_[flat] <= 0) {
            mutable_grid_.data[flat] = 0;
            sensor_occupied_[flat] = 0;
            sensor_clear_hits_[flat] = 0;
        }
    }
}

inline void WindowReplanner::update_risk(const Vec3& pose, const std::array<double, 6>* readings) {
    if (!adaptive_interval_enabled_) return;
    double min_sdf = obstacle_field_ ? obstacle_field_->signed_distance(pose)
                                     : std::numeric_limits<double>::infinity();
    double risk = 0.0;
    if (std::isfinite(min_sdf)) {
        risk = std::max(risk, 1.0 - std::min(min_sdf / 5.0, 1.0));
    }
    if (readings) {
        double sensor_min = *std::min_element(readings->begin(), readings->end());
        if (std::isfinite(sensor_min)) {
            risk = std::max(risk, 1.0 - std::min(sensor_min / 5.0, 1.0));
        }
    }
    last_risk_ = clamp(risk, 0.0, 1.0);
    risk_history_.push_back(last_risk_);
    if (risk_history_.size() > 20) risk_history_.erase(risk_history_.begin());
}

inline double WindowReplanner::path_deviation(const std::vector<Vec3>& a, const std::vector<Vec3>& b) const {
    if (a.empty() || b.empty()) return std::numeric_limits<double>::infinity();
    size_t K = std::min(a.size(), b.size());
    double md = 0;
    for (size_t i = 0; i < K; ++i) md = std::max(md, norm(a[i] - b[i]));
    return md;
}

inline int WindowReplanner::flat_index(const std::array<int,3>& idx) const {
    if (idx[0] < 0 || idx[0] >= mutable_grid_.nx ||
        idx[1] < 0 || idx[1] >= mutable_grid_.ny ||
        idx[2] < 0 || idx[2] >= mutable_grid_.nz) {
        return -1;
    }
    return (idx[2] * mutable_grid_.ny + idx[1]) * mutable_grid_.nx + idx[0];
}

}  // namespace sim
