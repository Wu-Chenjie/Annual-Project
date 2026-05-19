#include <chrono>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "model_importer.hpp"
#include "obstacle_scenario.hpp"
#include "json_writer.hpp"
#include "visualization.hpp"

namespace {
auto now() { return std::chrono::high_resolution_clock::now(); }
double sec(auto start, auto end) { return std::chrono::duration<double>(end - start).count(); }
std::string timestamp_dir_name() {
    auto n = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(n);
    std::tm* local = std::localtime(&t);
    std::ostringstream oss;
    if (local) oss << std::put_time(local, "%Y%m%d-%H%M%S");
    else oss << "unknown";
    return oss.str();
}
}

int main(int argc, char** argv) {
    using sim::Vec3;
    using sim::ObstacleConfig;
    using sim::ObstacleScenarioSimulation;
    using sim::SimulationVisualizer;
    using sim::import_model;

    if (argc < 2) {
        std::cerr << "usage: sim_scene_3dgs <model.ply|model.obj|model.stl> "
                  << "[voxel_size=0.3] [scale=1.0] [padding=0.5] [max_obstacles=10000]\n";
        return 2;
    }
    const std::string model_path = argv[1];
    const double voxel_size = (argc > 2) ? std::stod(argv[2]) : 0.3;
    const double model_scale = (argc > 3) ? std::stod(argv[3]) : 1.0;
    const double padding = (argc > 4) ? std::stod(argv[4]) : 0.5;
    const int max_obstacles = (argc > 5) ? std::stoi(argv[5]) : 10000;

    // [1] 导入 PLY
    std::cout << "[1] import_model... " << std::flush;
    auto t0 = now();
    auto [field, bounds] = import_model(model_path, voxel_size, model_scale, padding, max_obstacles);
    auto t1 = now();
    std::cout << field.size() << " obstacles, " << sec(t0,t1) << "s\n";

    // [2] 配置
    ObstacleConfig config;
    config.max_sim_time = 20.0;
    config.use_smc = true;
    config.use_backstepping = true;
    config.num_followers = 2;
    config.formation_spacing = 0.45;
    config.leader_max_vel = 1.0; config.leader_max_acc = 1.5;
    config.leader_gain_scale = 0.8; config.follower_gain_scale = 1.0;
    config.follower_max_vel = 5.0; config.follower_max_acc = 5.0;
    config.leader_acc_alpha = 0.3;
    config.enable_obstacles = true;
    config.planner_kind = "astar";
    config.planner_mode = "online";
    config.planner_resolution = 0.3;
    config.safety_margin = 0.3;
    config.sensor_enabled = true;
    config.planner_replan_interval = 2.0;
    config.planner_horizon = 5.0;
    config.danger_mode_enabled = true;
    config.apf_paper1_profile = "conservative";
    config.trajectory_optimizer_enabled = true;
    config.waypoints = {
        Vec3{bounds[0].x + 0.5, bounds[0].y + 0.5, 2.0},
        Vec3{(bounds[0].x + bounds[1].x) * 0.5, (bounds[0].y + bounds[1].y) * 0.5, 5.0},
        Vec3{bounds[1].x - 0.5, bounds[1].y - 0.5, 4.0},
    };

    // [3] set_obstacles
    std::cout << "[2] set_obstacles... " << std::flush;
    auto t2 = now();
    ObstacleScenarioSimulation sim(config);
    sim.set_obstacles(field, bounds);
    auto t3 = now();
    std::cout << sec(t2,t3) << "s\n";

    // [4] run
    std::cout << "[3] sim.run (max " << config.max_sim_time << "s)... " << std::flush;
    auto t4 = now();
    auto result = sim.run();
    auto t5 = now();
    std::cout << sec(t4,t5) << "s\n";

    // 结果
    std::cout << "\n航点: " << result.completed_waypoint_count << "/" << config.waypoints.size() << "\n";
    for (size_t i = 0; i < result.metrics.mean.size(); ++i)
        std::cout << "F" << (i+1) << ": mean=" << result.metrics.mean[i] << " max=" << result.metrics.max[i] << "\n";

    const auto run_dir = std::filesystem::path("outputs") / "scene_3dgs_cpp" / timestamp_dir_name();
    SimulationVisualizer vis(run_dir.string());
    auto figs = vis.plot_all(result);
    for (const auto& [k,v] : figs) std::cout << k << ": " << v << "\n";
    std::cout << "output: " << run_dir.string() << "\n";
    return 0;
}
