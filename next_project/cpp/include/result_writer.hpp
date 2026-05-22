#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "config.hpp"
#include "formation_simulation.hpp"
#include "json_writer.hpp"
#include "math_utils.hpp"
#include "obstacles.hpp"

// ============================================================
// 公共工具
// ============================================================

inline std::string timestamp_dir_name() {
    auto n = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(n);
    std::tm local{};
    std::ostringstream oss;
#ifdef _WIN32
    if (localtime_s(&local, &t) == 0)
        oss << std::put_time(&local, "%Y%m%d-%H%M%S");
    else
        oss << "unknown";
#else
    if (std::localtime_r(&t, &local))
        oss << std::put_time(&local, "%Y%m%d-%H%M%S");
    else
        oss << "unknown";
#endif
    return oss.str();
}

inline bool run_report_pipeline(const std::filesystem::path& json_path, bool enabled) {
    if (!enabled) return false;
    std::string rel = json_path.string();
    // 转义双引号，避免命令注入
    std::string escaped_rel;
    for (char c : rel) {
        if (c == '"') escaped_rel += "\\\"";
        else escaped_rel += c;
    }
    std::cout << "Generating report: " << std::flush;
    int ret = -1;
    for (const char* script : {"../experiments/report_cpp_results.py",
                                "../../experiments/report_cpp_results.py"}) {
        std::string cmd = std::string("python \"") + script + "\" \"" + escaped_rel + "\"";
        ret = std::system(cmd.c_str());
        if (ret == 0) break;
    }
    if (ret == 0) {
        std::cout << (json_path.parent_path() / "cpp_report.md").string() << "\n";
    } else {
        std::cout << "skip report generation (python unavailable or report command failed, code=" << ret << ")\n";
    }
    return ret == 0;
}

namespace sim {

// ============================================================
// 辅助统计函数
// ============================================================

inline int accepted_replan_count(const SimulationResult& result) {
    int count = 0;
    for (const auto& event : result.planning_events)
        if (event.phase == "online_replan" && event.accepted) ++count;
    return count;
}

inline int collision_interval_count(const std::vector<CollisionEvent>& events, double dt) {
    if (events.empty()) return 0;
    std::vector<CollisionEvent> sorted = events;
    std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
        if (a.drone == b.drone) return a.t < b.t;
        return a.drone < b.drone;
    });
    int intervals = 0;
    std::string current_drone;
    double previous_t = -1.0;
    const double max_gap = std::max(dt * 1.5, 1e-6);
    for (const auto& event : sorted) {
        if (event.drone != current_drone || previous_t < 0.0 || event.t - previous_t > max_gap) {
            ++intervals;
            current_drone = event.drone;
        }
        previous_t = event.t;
    }
    return intervals;
}

inline int hard_collision_step_count(const std::vector<CollisionEvent>& events, const ObstacleField& obstacles) {
    int count = 0;
    for (const auto& event : events)
        if (obstacles.signed_distance(event.pos) < 0.0) ++count;
    return count;
}

inline int hard_collision_interval_count(const std::vector<CollisionEvent>& events,
                                          const ObstacleField& obstacles, double dt) {
    std::vector<CollisionEvent> hard;
    hard.reserve(events.size());
    for (const auto& e : events)
        if (obstacles.signed_distance(e.pos) < 0.0) hard.push_back(e);
    return collision_interval_count(hard, dt);
}

inline double min_airframe_signed_distance(const SimulationResult& result, const ObstacleField& obstacles) {
    if (obstacles.size() == 0) return 0.0;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& p : result.executed_path) best = std::min(best, obstacles.signed_distance(p));
    for (const auto& fp : result.followers)
        for (const auto& p : fp) best = std::min(best, obstacles.signed_distance(p));
    return std::isfinite(best) ? best : 0.0;
}

// ============================================================
// JSON 事件序列化
// ============================================================

inline void write_planning_events(JsonWriter& w, const std::vector<PlanningEvent>& events) {
    w.key("planning_events").begin_array();
    for (const auto& e : events) {
        w.begin_object();
        w.key("t").value(e.t); w.key("phase").value(e.phase);
        w.key("planner").value(e.planner); w.key("segment_index").value(e.segment_index);
        w.key("wall_time_s").value(e.wall_time_s); w.key("point_count").value(e.point_count);
        w.key("accepted").value(e.accepted); w.key("fallback_reason").value(e.fallback_reason);
        w.end_object();
    }
    w.end_array();
}

inline void write_waypoint_events(JsonWriter& w, const std::vector<WaypointEvent>& events) {
    w.key("waypoint_events").begin_array();
    for (const auto& e : events) {
        w.begin_object();
        w.key("t").value(e.t); w.key("type").value(e.type);
        w.key("index").value(e.index); w.key("distance").value(e.distance);
        w.end_object();
    }
    w.end_array();
}

inline void write_collision_log(JsonWriter& w, const std::vector<CollisionEvent>& events) {
    w.key("collision_log").begin_array();
    for (const auto& e : events) {
        w.begin_object();
        w.key("t").value(e.t); w.key("drone").value(e.drone); w.key("pos").vec3(e.pos);
        w.end_object();
    }
    w.end_array();
}

inline void write_formation_adaptation_events(JsonWriter& w,
                                               const std::vector<FormationAdaptationEvent>& events) {
    w.key("formation_adaptation_events").begin_array();
    for (const auto& e : events) {
        w.begin_object();
        w.key("t").value(e.t);
        if (!e.kind.empty()) w.key("kind").value(e.kind);
        w.key("from").value(e.from); w.key("to").value(e.to); w.key("reason").value(e.reason);
        if (e.has_channel_width) {
            w.key("channel_width").begin_array();
            w.value(e.channel_width[0]); w.value(e.channel_width[1]); w.value(e.channel_width[2]);
            w.end_array();
        }
        if (e.has_selected_envelope) {
            w.key("selected_envelope").begin_array();
            w.value(e.selected_envelope[0]); w.value(e.selected_envelope[1]); w.value(e.selected_envelope[2]);
            w.end_array();
        }
        if (e.has_clearance_margin) w.key("clearance_margin").value(e.clearance_margin);
        if (e.has_max_turn_angle) w.key("max_turn_angle_rad").value(e.max_turn_angle_rad);
        if (!e.planner.empty()) w.key("planner").value(e.planner);
        if (!e.goal_kind.empty()) w.key("goal_kind").value(e.goal_kind);
        if (e.goal_count > 0) w.key("goal_count").value(e.goal_count);
        if (e.point_count > 0) w.key("point_count").value(e.point_count);
        w.key("blocked_by_hold_time").value(e.blocked_by_hold_time);
        w.end_object();
    }
    w.end_array();
}

inline void write_obstacle_model(JsonWriter& w, const ObstacleField& obstacles,
                                  const std::array<Vec3, 2>& bounds) {
    const auto& variants = obstacles.obstacles();
    const auto& ids = obstacles.ids();
    w.key("obstacle_model").begin_object();
    w.key("bounds").begin_array(); w.vec3(bounds[0]); w.vec3(bounds[1]); w.end_array();
    w.key("primitives").begin_array();
    for (std::size_t i = 0; i < variants.size(); ++i) {
        const auto& p = variants[i];
        w.begin_object();
        w.key("id").value(i < ids.size() ? ids[i] : ("obs_" + std::to_string(i)));
        if (std::holds_alternative<AABB>(p)) {
            const auto& b = std::get<AABB>(p);
            w.key("type").value("aabb"); w.key("min").vec3(b.min_corner); w.key("max").vec3(b.max_corner);
        } else if (std::holds_alternative<Sphere>(p)) {
            const auto& s = std::get<Sphere>(p);
            w.key("type").value("sphere"); w.key("center").vec3(s.center); w.key("radius").value(s.radius);
        } else if (std::holds_alternative<Cylinder>(p)) {
            const auto& c = std::get<Cylinder>(p);
            w.key("type").value("cylinder"); w.key("center").vec3(c.center_xy);
            w.key("radius").value(c.radius); w.key("z_min").value(c.z_min); w.key("z_max").value(c.z_max);
        }
        w.end_object();
    }
    w.end_array();
    w.end_object();
}

// ============================================================
// 主入口
// ============================================================

inline void write_result_json(
    const std::filesystem::path& output_path,
    const SimulationResult& result,
    double planning_s,
    double sim_s,
    const std::string& preset,
    const ObstacleConfig& config,
    const ObstacleField& obstacles,
    const std::array<Vec3, 2>& bounds
) {
    namespace fs = std::filesystem;
    fs::create_directories(output_path.parent_path());
    std::ofstream out(output_path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write result json");

    double overall_mean = 0.0, overall_max = 0.0, overall_final = 0.0;
    if (!result.metrics.mean.empty()) {
        for (double v : result.metrics.mean) overall_mean += v;
        overall_mean /= static_cast<double>(result.metrics.mean.size());
    }
    for (double v : result.metrics.max) overall_max = std::max(overall_max, v);
    if (!result.metrics.final.empty()) {
        for (double v : result.metrics.final) overall_final += v;
        overall_final /= static_cast<double>(result.metrics.final.size());
    }

    const int collision_steps = static_cast<int>(result.collision_log.size());
    const int collision_intervals = collision_interval_count(result.collision_log, config.dt);
    const int hcs = hard_collision_step_count(result.collision_log, obstacles);
    const int hci = hard_collision_interval_count(result.collision_log, obstacles, config.dt);
    const double min_obs_sd = min_airframe_signed_distance(result, obstacles);

    int lookahead_blocked = 0, rrt_attempt = 0, rrt_accepted = 0, rrt_failed = 0;
    for (const auto& e : result.formation_adaptation_events) {
        if (e.kind == "lookahead_reference_blocked") ++lookahead_blocked;
        else if (e.kind == "rrt_escape_attempt") ++rrt_attempt;
        else if (e.kind == "rrt_escape_accepted") ++rrt_accepted;
        else if (e.kind == "rrt_escape_failed") ++rrt_failed;
    }

    JsonWriter w(out);
    w.begin_object();
    w.key("schema_version").value("1.0.0");
    w.key("preset").value(preset);
    w.key("runtime_engine").value("cpp");
    w.key("engine_version").value(engine_version());
    w.key("generated_at").value(iso8601_utc(std::chrono::system_clock::now()));

    w.key("config_snapshot").begin_object();
    w.key("dt").value(config.dt);
    w.key("num_followers").value(config.num_followers);
    w.key("initial_formation").value(config.initial_formation);
    w.key("leader_max_vel").value(config.leader_max_vel);
    w.key("leader_max_acc").value(config.leader_max_acc);
    w.key("planner_kind").value(config.planner_kind);
    w.key("planner_mode").value(config.planner_mode);
    w.key("planner_initial_map_unknown").value(config.planner_initial_map_unknown);
    w.key("planner_resolution").value(config.planner_resolution);
    w.key("planner_replan_interval").value(config.planner_replan_interval);
    w.key("planner_horizon").value(config.planner_horizon);
    w.key("safety_margin").value(config.safety_margin);
    w.key("detect_margin_scale").value(config.detect_margin_scale);
    w.key("plan_clearance_extra").value(config.plan_clearance_extra);
    w.key("planner_use_formation_envelope").value(config.planner_use_formation_envelope);
    w.key("sensor_enabled").value(config.sensor_enabled);
    w.key("danger_mode_enabled").value(config.danger_mode_enabled);
    w.key("apf_paper1_profile").value(config.apf_paper1_profile);
    w.key("apf_formation_centroid").value(config.apf_formation_centroid);
    w.key("formation_adaptation_enabled").value(config.formation_adaptation_enabled);
    w.key("formation_lookahead_enabled").value(config.formation_lookahead_enabled);
    w.key("formation_lookahead_rrt_enabled").value(config.formation_lookahead_rrt_enabled);
    w.key("formation_lookahead_distance").value(config.formation_lookahead_distance);
    w.key("formation_lookahead_turn_threshold_rad").value(config.formation_lookahead_turn_threshold_rad);
    w.key("trajectory_optimizer_enabled").value(config.trajectory_optimizer_enabled);
    w.key("trajectory_optimizer_method").value(config.trajectory_optimizer_method);
    w.end_object();

    w.key("metrics").begin_object();
    w.key("mean").array_double(result.metrics.mean);
    w.key("max").array_double(result.metrics.max);
    w.key("final").array_double(result.metrics.final);
    w.end_object();

    w.key("completed_waypoint_count").value(result.completed_waypoint_count);
    w.key("runtime_s").value(planning_s + sim_s);

    w.key("summary").begin_object();
    w.key("mean_error_overall").value(overall_mean);
    w.key("max_error_overall").value(overall_max);
    w.key("final_error_overall").value(overall_final);
    w.key("collision_count").value(hci);
    w.key("collision_step_count").value(hcs);
    w.key("hard_collision_count").value(hci);
    w.key("hard_collision_step_count").value(hcs);
    w.key("clearance_warning_count").value(collision_intervals);
    w.key("clearance_warning_step_count").value(collision_steps);
    w.key("min_obstacle_signed_distance").value(min_obs_sd);
    w.key("replan_count").value(accepted_replan_count(result));
    w.key("fault_count").value(static_cast<int>(result.fault_log.size()));
    w.key("formation_adaptation_count").value(static_cast<int>(result.formation_adaptation_events.size()));
    w.key("lookahead_reference_blocked_count").value(lookahead_blocked);
    w.key("rrt_escape_attempt_count").value(rrt_attempt);
    w.key("rrt_escape_accepted_count").value(rrt_accepted);
    w.key("rrt_escape_failed_count").value(rrt_failed);
    w.end_object();

    w.key("timing").begin_object();
    w.key("planning_s").value(planning_s);
    w.key("simulation_s").value(sim_s);
    w.key("total_s").value(planning_s + sim_s);
    w.end_object();

    if (config.planner_initial_map_unknown) {
        w.key("map_knowledge").begin_object();
        w.key("initial_map_unknown").value(true);
        w.key("truth_obstacle_count").value(static_cast<int>(obstacles.size()));
        w.key("planner_static_occupied_count").value(0);
        w.key("planner_sensor_occupied_count").value(0);
        w.end_object();
    }

    w.key("time").array_double(result.time);
    w.key("planned_path").array_vec3(result.planned_path);
    w.key("task_waypoints").array_vec3(result.task_waypoints);
    w.key("replanned_waypoints").array_vec3(result.replanned_waypoints);
    w.key("executed_path").array_vec3(result.executed_path);
    write_planning_events(w, result.planning_events);
    write_waypoint_events(w, result.waypoint_events);
    write_collision_log(w, result.collision_log);
    write_formation_adaptation_events(w, result.formation_adaptation_events);
    write_obstacle_model(w, obstacles, bounds);
    w.key("fault_log").array_string(result.fault_log);

    w.key("safety_metrics").begin_object();
    w.key("min_inter_drone_distance").value(result.safety_metrics.min_inter_drone_distance);
    w.key("downwash_hits").value(result.safety_metrics.downwash_hits);
    w.end_object();

    w.end_object();
    out << "\n";
}

}  // namespace sim
