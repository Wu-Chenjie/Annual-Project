#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "model_importer.hpp"
#include "obstacle_scenario.hpp"
#include "result_writer.hpp"
#include "visualization.hpp"

namespace {
auto now() { return std::chrono::high_resolution_clock::now(); }
double sec(auto start, auto end) { return std::chrono::duration<double>(end - start).count(); }
}

int main(int argc, char** argv) {
    using sim::Vec3;
    using sim::ObstacleConfig;
    using sim::ObstacleScenarioSimulation;
    using sim::SimulationVisualizer;
    using sim::import_model;

    std::vector<std::string> positional_args;
    bool report_enabled = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--report") {
            report_enabled = true;
        } else if (arg.rfind("--", 0) == 0) {
            std::cerr << "unknown option: " << arg << "\n";
            return 2;
        } else {
            positional_args.push_back(arg);
        }
    }

    if (positional_args.empty() || positional_args.size() > 5) {
        std::cerr << "usage: sim_scene_3dgs <model.ply|model.obj|model.stl> "
                  << "[voxel_size=0.3] [scale=1.0] [padding=0.5] [max_obstacles=10000] [--report]\n";
        return 2;
    }
    const std::string model_path = positional_args[0];
    double voxel_size = 0.3;
    double model_scale = 1.0;
    double padding = 0.5;
    int max_obstacles = 10000;
    try {
        if (positional_args.size() > 1) voxel_size = std::stod(positional_args[1]);
        if (positional_args.size() > 2) model_scale = std::stod(positional_args[2]);
        if (positional_args.size() > 3) padding = std::stod(positional_args[3]);
        if (positional_args.size() > 4) max_obstacles = std::stoi(positional_args[4]);
    } catch (const std::exception& exc) {
        std::cerr << "invalid numeric argument: " << exc.what() << "\n";
        return 2;
    }

    // [1] import
    std::cout << "[1] import_model... " << std::flush;
    auto t0 = now();
    auto [field, bounds] = import_model(model_path, voxel_size, model_scale, padding, max_obstacles);
    auto t1 = now();
    std::cout << field.size() << " obstacles, " << sec(t0,t1) << "s\n";

    // [2] config
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

    // [3] set obstacles
    std::cout << "[2] set_obstacles... " << std::flush;
    auto t2 = now();
    ObstacleScenarioSimulation sim(config);
    sim.set_obstacles(field, bounds);
    auto t3 = now();
    double planning_s = sec(t2, t3);
    std::cout << planning_s << "s\n";

    // [4] run
    std::cout << "[3] sim.run (max " << config.max_sim_time << "s)... " << std::flush;
    auto t4 = now();
    auto result = sim.run();
    auto t5 = now();
    double sim_s = sec(t4, t5);
    std::cout << sim_s << "s\n";

    std::cout << "\n航点: " << result.completed_waypoint_count << "/" << config.waypoints.size() << "\n";
    for (size_t i = 0; i < result.metrics.mean.size(); ++i)
        std::cout << "F" << (i+1) << ": mean=" << result.metrics.mean[i] << " max=" << result.metrics.max[i] << " final=" << result.metrics.final[i] << "\n";
    std::cout << "碰撞: " << result.collision_log.size()
              << "  重规划: " << result.planning_events.size()
              << "  故障: " << result.fault_log.size() << "\n";
    std::cout << "规划路径: " << result.planned_path.size() << "pts"
              << "  执行路径: " << result.executed_path.size() << "pts"
              << "  安全间距: " << result.safety_metrics.min_inter_drone_distance << "m"
              << "  downwash: " << result.safety_metrics.downwash_hits << "\n";

    const auto run_dir = std::filesystem::path("outputs") / "scene_3dgs_cpp" / timestamp_dir_name();
    SimulationVisualizer vis(run_dir.string());
    auto figs = vis.plot_all(result);
    for (const auto& [k,v] : figs) std::cout << k << ": " << v << "\n";
    std::cout << "output: " << run_dir.string() << "\n";

    auto json_path = run_dir / "sim_result.json";
    sim::write_result_json(json_path, result, planning_s, sim_s, "scene_3dgs", config, field, bounds);
    std::cout << "结果文件: " << json_path.string() << "\n";
    run_report_pipeline(json_path, report_enabled);

    return 0;
}
