#include <array>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <variant>

#include "config.hpp"
#include "json_writer.hpp"
#include "obstacles.hpp"
#include "obstacle_scenario.hpp"

namespace sim {

ObstacleField make_warehouse() {
    ObstacleField f;
    f.add_sphere({7,6,3.5}, 1.2);
    f.add_aabb({5,3,0}, {9,9,0.4});
    f.add_cylinder({10.5,2,0}, 0.15, 0, 6);
    f.add_cylinder({10.5,10,0}, 0.15, 0, 6);
    f.add_sphere({17,4,3.2}, 1.6);
    f.add_sphere({17,22,3.8}, 1.8);
    f.add_aabb({13,7.5,1.8}, {21,9,2.2});
    f.add_aabb({13,12,4}, {22,13.5,4.5});
    f.add_aabb({12,15.5,1.8}, {18,17,2.2});
    f.add_aabb({13,19,4}, {21,20.5,4.5});
    f.add_aabb({23,9,2.6}, {27,11.5,3.1});
    f.add_aabb({23,18,2.6}, {27,20.5,3.1});
    f.add_cylinder({15,5.5,0}, 0.22, 0, 9);
    f.add_cylinder({15,13,0}, 0.22, 0, 9);
    f.add_cylinder({15,21,0}, 0.22, 0, 9);
    f.add_cylinder({25,5.5,0}, 0.22, 0, 9);
    f.add_cylinder({25,13,0}, 0.22, 0, 9);
    f.add_cylinder({25,21,0}, 0.22, 0, 9);
    f.add_sphere({32,19,4.2}, 1.3);
    f.add_cylinder({35,5,0}, 0.55, 0, 5);
    f.add_cylinder({37,5,0}, 0.6, 0, 5.5);
    f.add_cylinder({35,12,0}, 0.5, 0, 4.5);
    f.add_cylinder({37,12,0}, 0.55, 0, 5.5);
    f.add_aabb({32,7,5.5}, {40,14,6});
    f.add_aabb({30,12,7.5}, {42,14,9});
    f.add_aabb({30,20,7.5}, {42,22,9});
    f.add_aabb({39,3,7}, {44,4,8});
    f.add_aabb({39,10,7}, {44,11,8});
    f.add_aabb({39,18,7}, {44,19,8});
    f.add_aabb({42,4,3}, {44,8,3.5});
    f.add_aabb({-1,-3,0}, {45,-1,10});
    f.add_aabb({-1,26,0}, {45,29,10});
    f.add_aabb({44,-3,0}, {45,29,10});
    f.add_aabb({-3,-3,0}, {-1,29,10});
    return f;
}

}  // namespace sim

namespace {

std::string timestamp_dir_name() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* local = std::localtime(&t);
    std::ostringstream oss;
    if (local != nullptr) {
        oss << std::put_time(local, "%Y%m%d-%H%M%S");
    } else {
        oss << "unknown";
    }
    return oss.str();
}

int accepted_replan_count(const sim::SimulationResult& result) {
    int count = 0;
    for (const auto& event : result.planning_events) {
        if (event.phase == "online_replan" && event.accepted) {
            ++count;
        }
    }
    return count;
}

int collision_interval_count(const std::vector<sim::CollisionEvent>& events, double dt) {
    if (events.empty()) return 0;
    std::vector<sim::CollisionEvent> sorted = events;
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

int hard_collision_interval_count(
    const std::vector<sim::CollisionEvent>& events,
    const sim::ObstacleField& obstacles,
    double dt
) {
    std::vector<sim::CollisionEvent> hard_events;
    hard_events.reserve(events.size());
    for (const auto& event : events) {
        if (obstacles.signed_distance(event.pos) < 0.0) {
            hard_events.push_back(event);
        }
    }
    return collision_interval_count(hard_events, dt);
}

int hard_collision_step_count(
    const std::vector<sim::CollisionEvent>& events,
    const sim::ObstacleField& obstacles
) {
    int count = 0;
    for (const auto& event : events) {
        if (obstacles.signed_distance(event.pos) < 0.0) {
            ++count;
        }
    }
    return count;
}

double min_airframe_signed_distance(const sim::SimulationResult& result, const sim::ObstacleField& obstacles) {
    if (obstacles.size() == 0) return 0.0;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& point : result.executed_path) {
        best = std::min(best, obstacles.signed_distance(point));
    }
    for (const auto& follower_path : result.followers) {
        for (const auto& point : follower_path) {
            best = std::min(best, obstacles.signed_distance(point));
        }
    }
    return std::isfinite(best) ? best : 0.0;
}

void write_planning_events(sim::JsonWriter& w, const std::vector<sim::PlanningEvent>& events) {
    w.key("planning_events").begin_array();
    for (const auto& event : events) {
        w.begin_object();
        w.key("t").value(event.t);
        w.key("phase").value(event.phase);
        w.key("planner").value(event.planner);
        w.key("segment_index").value(event.segment_index);
        w.key("wall_time_s").value(event.wall_time_s);
        w.key("point_count").value(event.point_count);
        w.key("accepted").value(event.accepted);
        w.key("fallback_reason").value(event.fallback_reason);
        w.end_object();
    }
    w.end_array();
}

void write_waypoint_events(sim::JsonWriter& w, const std::vector<sim::WaypointEvent>& events) {
    w.key("waypoint_events").begin_array();
    for (const auto& event : events) {
        w.begin_object();
        w.key("t").value(event.t);
        w.key("type").value(event.type);
        w.key("index").value(event.index);
        w.key("distance").value(event.distance);
        w.end_object();
    }
    w.end_array();
}

void write_collision_log(sim::JsonWriter& w, const std::vector<sim::CollisionEvent>& events) {
    w.key("collision_log").begin_array();
    for (const auto& event : events) {
        w.begin_object();
        w.key("t").value(event.t);
        w.key("drone").value(event.drone);
        w.key("pos").vec3(event.pos);
        w.end_object();
    }
    w.end_array();
}

void write_obstacle_model(
    sim::JsonWriter& w,
    const sim::ObstacleField& obstacles,
    const std::array<sim::Vec3, 2>& bounds
) {
    const auto& variants = obstacles.obstacles();
    const auto& ids = obstacles.ids();

    w.key("obstacle_model").begin_object();
    w.key("bounds").begin_array();
    w.vec3(bounds[0]);
    w.vec3(bounds[1]);
    w.end_array();

    w.key("primitives").begin_array();
    for (std::size_t i = 0; i < variants.size(); ++i) {
        const auto& primitive = variants[i];
        w.begin_object();
        w.key("id").value(i < ids.size() ? ids[i] : ("obs_" + std::to_string(i)));
        if (std::holds_alternative<sim::AABB>(primitive)) {
            const auto& box = std::get<sim::AABB>(primitive);
            w.key("type").value("aabb");
            w.key("min").vec3(box.min_corner);
            w.key("max").vec3(box.max_corner);
        } else if (std::holds_alternative<sim::Sphere>(primitive)) {
            const auto& sphere = std::get<sim::Sphere>(primitive);
            w.key("type").value("sphere");
            w.key("center").vec3(sphere.center);
            w.key("radius").value(sphere.radius);
        } else if (std::holds_alternative<sim::Cylinder>(primitive)) {
            const auto& cylinder = std::get<sim::Cylinder>(primitive);
            w.key("type").value("cylinder");
            w.key("center").vec3(cylinder.center_xy);
            w.key("radius").value(cylinder.radius);
            w.key("z_min").value(cylinder.z_min);
            w.key("z_max").value(cylinder.z_max);
        }
        w.end_object();
    }
    w.end_array();
    w.end_object();
}

void write_obstacle_result_json(
    const std::filesystem::path& output_path,
    const sim::SimulationResult& result,
    double planning_s,
    double sim_s,
    const std::string& preset,
    const sim::ObstacleConfig& config,
    const sim::ObstacleField& obstacles,
    const std::array<sim::Vec3, 2>& bounds
) {
    namespace fs = std::filesystem;
    fs::create_directories(output_path.parent_path());
    std::ofstream out(output_path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("cannot write obstacle result json");
    }

    double overall_mean = 0.0;
    double overall_max = 0.0;
    double overall_final = 0.0;
    if (!result.metrics.mean.empty()) {
        for (double v : result.metrics.mean) overall_mean += v;
        overall_mean /= static_cast<double>(result.metrics.mean.size());
    }
    if (!result.metrics.max.empty()) {
        for (double v : result.metrics.max) overall_max = std::max(overall_max, v);
    }
    if (!result.metrics.final.empty()) {
        for (double v : result.metrics.final) overall_final += v;
        overall_final /= static_cast<double>(result.metrics.final.size());
    }
    const int collision_steps = static_cast<int>(result.collision_log.size());
    const int collision_intervals = collision_interval_count(result.collision_log, config.dt);
    const int hard_collision_steps = hard_collision_step_count(result.collision_log, obstacles);
    const int hard_collision_intervals = hard_collision_interval_count(result.collision_log, obstacles, config.dt);
    const double min_obstacle_sd = min_airframe_signed_distance(result, obstacles);

    sim::JsonWriter w(out);

    w.begin_object();
    w.key("schema_version").value("1.0.0");
    w.key("preset").value(preset);
    w.key("runtime_engine").value("cpp");
    w.key("engine_version").value(sim::engine_version());
    w.key("generated_at").value(sim::iso8601_utc(std::chrono::system_clock::now()));

    w.key("config_snapshot").begin_object();
    w.key("dt").value(config.dt);
    w.key("num_followers").value(config.num_followers);
    w.key("initial_formation").value(config.initial_formation);
    w.key("leader_max_vel").value(config.leader_max_vel);
    w.key("leader_max_acc").value(config.leader_max_acc);
    w.key("planner_kind").value(config.planner_kind);
    w.key("planner_mode").value(config.planner_mode);
    w.key("planner_resolution").value(config.planner_resolution);
    w.key("planner_replan_interval").value(config.planner_replan_interval);
    w.key("planner_horizon").value(config.planner_horizon);
    w.key("safety_margin").value(config.safety_margin);
    w.key("detect_margin_scale").value(config.detect_margin_scale);
    w.key("plan_clearance_extra").value(config.plan_clearance_extra);
    w.key("planner_use_formation_envelope").value(config.planner_use_formation_envelope);
    w.key("sensor_enabled").value(config.sensor_enabled);
    w.key("danger_mode_enabled").value(config.danger_mode_enabled);
    w.key("apf_formation_centroid").value(config.apf_formation_centroid);
    w.key("trajectory_optimizer_enabled").value(false);
    w.key("trajectory_optimizer_method").value("");
    w.end_object();

    w.key("metrics").begin_object();
    w.key("mean").array_double(result.metrics.mean);
    w.key("max").array_double(result.metrics.max);
    w.key("final").array_double(result.metrics.final);
    w.end_object();  // metrics

    w.key("completed_waypoint_count").value(result.completed_waypoint_count);
    w.key("runtime_s").value(planning_s + sim_s);

    // summary
    w.key("summary").begin_object();
    w.key("mean_error_overall").value(overall_mean);
    w.key("max_error_overall").value(overall_max);
    w.key("final_error_overall").value(overall_final);
    w.key("collision_count").value(hard_collision_intervals);
    w.key("collision_step_count").value(hard_collision_steps);
    w.key("hard_collision_count").value(hard_collision_intervals);
    w.key("hard_collision_step_count").value(hard_collision_steps);
    w.key("clearance_warning_count").value(collision_intervals);
    w.key("clearance_warning_step_count").value(collision_steps);
    w.key("min_obstacle_signed_distance").value(min_obstacle_sd);
    w.key("replan_count").value(accepted_replan_count(result));
    w.key("fault_count").value(static_cast<int>(result.fault_log.size()));
    w.end_object();  // summary

    // timing snapshot (custom field, passthrough)
    w.key("timing").begin_object();
    w.key("planning_s").value(planning_s);
    w.key("simulation_s").value(sim_s);
    w.key("total_s").value(planning_s + sim_s);
    w.end_object();

    // optional passthrough fields
    w.key("time").array_double(result.time);
    w.key("planned_path").array_vec3(result.planned_path);
    w.key("task_waypoints").array_vec3(result.task_waypoints);
    w.key("replanned_waypoints").array_vec3(result.replanned_waypoints);
    w.key("executed_path").array_vec3(result.executed_path);
    write_planning_events(w, result.planning_events);
    write_waypoint_events(w, result.waypoint_events);
    write_collision_log(w, result.collision_log);
    write_obstacle_model(w, obstacles, bounds);
    w.key("fault_log").array_string(result.fault_log);

    w.key("safety_metrics").begin_object();
    w.key("min_inter_drone_distance").value(result.safety_metrics.min_inter_drone_distance);
    w.key("downwash_hits").value(result.safety_metrics.downwash_hits);
    w.end_object();  // safety_metrics

    w.end_object();  // root
    out << "\n";
}

}  // namespace

int main() {
    using sim::ObstacleConfig;
    using sim::ObstacleScenarioSimulation;
    using sim::Vec3;

    ObstacleConfig config = sim::config_warehouse();
    config.map_file = "";

    std::cout << "===== C++ warehouse scenario (Python-compatible subset) =====\n";

    auto t0 = std::chrono::high_resolution_clock::now();
    ObstacleScenarioSimulation sim(config);
    auto warehouse_obstacles = sim::make_warehouse();
    std::array<Vec3, 2> bounds{Vec3{-3,-3,0}, Vec3{45,28,10}};
    sim.set_obstacles(warehouse_obstacles, bounds);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto result = sim.run();
    auto t2 = std::chrono::high_resolution_clock::now();

    double tp = std::chrono::duration<double>(t1 - t0).count();
    double ts = std::chrono::duration<double>(t2 - t1).count();

    std::cout << "规划: " << tp << "s | 仿真: " << ts << "s | 总: " << (tp+ts) << "s\n";
    std::cout << "路点: " << result.completed_waypoint_count << "/" << result.waypoints.size() << "\n";
    for (size_t i = 0; i < result.metrics.mean.size(); ++i)
        std::cout << "F" << (i+1) << ": mean=" << result.metrics.mean[i]
                  << " max=" << result.metrics.max[i]
                  << " final=" << result.metrics.final[i] << "\n";
    std::cout << "Safety: min_inter=" << result.safety_metrics.min_inter_drone_distance
              << " downwash_hits=" << result.safety_metrics.downwash_hits << "\n";

    const std::string preset = "warehouse";
    const std::filesystem::path output_path =
        std::filesystem::path("outputs") / preset / timestamp_dir_name() / "sim_result.json";
    write_obstacle_result_json(output_path, result, tp, ts, preset, config, warehouse_obstacles, bounds);
    std::cout << "结果文件: " << output_path.string() << "\n";
    return 0;
}
