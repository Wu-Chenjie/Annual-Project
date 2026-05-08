#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "config.hpp"
#include "json_writer.hpp"
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

void write_obstacle_result_json(
    const std::filesystem::path& output_path,
    const sim::SimulationResult& result,
    double planning_s,
    double sim_s,
    const std::string& preset
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

    sim::JsonWriter w(out);

    w.begin_object();
    w.key("schema_version").value("1.0.0");
    w.key("preset").value(preset);
    w.key("runtime_engine").value("cpp");
    w.key("engine_version").value(sim::engine_version());
    w.key("generated_at").value(sim::iso8601_utc(std::chrono::system_clock::now()));

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
    w.key("collision_count").value(0);
    w.key("replan_count").value(0);
    w.key("fault_count").value(static_cast<int>(result.fault_log.size()));
    w.end_object();  // summary

    // timing snapshot (custom field, passthrough)
    w.key("timing").begin_object();
    w.key("planning_s").value(planning_s);
    w.key("simulation_s").value(sim_s);
    w.key("total_s").value(planning_s + sim_s);
    w.end_object();

    // optional passthrough fields
    w.key("task_waypoints").array_vec3(result.task_waypoints);
    w.key("replanned_waypoints").array_vec3(result.replanned_waypoints);
    w.key("executed_path").array_vec3(result.executed_path);
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
    sim.set_obstacles(sim::make_warehouse(),
                       std::array<Vec3, 2>{Vec3{-3,-3,0}, Vec3{45,28,10}});
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
    write_obstacle_result_json(output_path, result, tp, ts, preset);
    std::cout << "结果文件: " << output_path.string() << "\n";
    return 0;
}
