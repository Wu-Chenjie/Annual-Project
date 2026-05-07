#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "config.hpp"
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

void dump_vec3(std::ostream& out, const sim::Vec3& value) {
    out << "[" << value.x << "," << value.y << "," << value.z << "]";
}

void dump_vec3_array(std::ostream& out, const std::vector<sim::Vec3>& values) {
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << ",";
        dump_vec3(out, values[i]);
    }
    out << "]";
}

void dump_string(std::ostream& out, const std::string& value) {
    out << "\"";
    for (char c : value) {
        if (c == '"' || c == '\\') out << "\\";
        out << c;
    }
    out << "\"";
}

void dump_string_array(std::ostream& out, const std::vector<std::string>& values) {
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << ",";
        dump_string(out, values[i]);
    }
    out << "]";
}

void write_obstacle_result_json(
    const std::filesystem::path& output_path,
    const sim::SimulationResult& result,
    double planning_s,
    double sim_s
) {
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream out(output_path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("cannot write obstacle result json");
    }
    out << std::fixed << std::setprecision(8);
    out << "{";
    out << "\"timing\":{\"planning_s\":" << planning_s
        << ",\"simulation_s\":" << sim_s
        << ",\"total_s\":" << (planning_s + sim_s) << "},";
    out << "\"completed_waypoint_count\":" << result.completed_waypoint_count << ",";
    out << "\"task_waypoints\":";
    dump_vec3_array(out, result.task_waypoints);
    out << ",\"replanned_waypoints\":";
    dump_vec3_array(out, result.replanned_waypoints);
    out << ",\"executed_path\":";
    dump_vec3_array(out, result.executed_path);
    out << ",\"fault_log\":";
    dump_string_array(out, result.fault_log);
    out << ",\"safety_metrics\":{"
        << "\"min_inter_drone_distance\":" << result.safety_metrics.min_inter_drone_distance
        << ",\"downwash_hits\":" << result.safety_metrics.downwash_hits
        << "}";
    out << "}\n";
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

    const std::filesystem::path output_path = std::filesystem::path("outputs") / "warehouse_result.json";
    write_obstacle_result_json(output_path, result, tp, ts);
    std::cout << "结果文件: " << output_path.string() << "\n";
    return 0;
}
