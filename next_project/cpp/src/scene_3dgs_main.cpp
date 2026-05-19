#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

    // [5] 写 sim_result.json + 调 Python 生成 report.md
    {
        namespace fs = std::filesystem;
        fs::create_directories(run_dir);
        auto json_path = run_dir / "sim_result.json";
        std::ofstream out(json_path);
        if (out) {
            using sim::JsonWriter;
            JsonWriter w(out);
            double overall_mean = 0.0, overall_max = 0.0, overall_final = 0.0;
            if (!result.metrics.mean.empty()) {
                for (double v : result.metrics.mean) overall_mean += v;
                overall_mean /= static_cast<double>(result.metrics.mean.size());
            }
            if (!result.metrics.max.empty()) {
                for (double v : result.metrics.max)
                    overall_max = std::max(overall_max, v);
            }
            if (!result.metrics.final.empty()) {
                for (double v : result.metrics.final) overall_final += v;
                overall_final /= static_cast<double>(result.metrics.final.size());
            }

            w.begin_object();
            w.key("schema_version").value("1.0.0");
            w.key("preset").value("scene_3dgs");
            w.key("runtime_engine").value("cpp");
            w.key("completed_waypoint_count").value(result.completed_waypoint_count);
            w.key("runtime_s").value(sec(t2,t3) + sec(t4,t5));

            w.key("config_snapshot").begin_object();
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
            w.key("sensor_enabled").value(config.sensor_enabled);
            w.key("danger_mode_enabled").value(config.danger_mode_enabled);
            w.key("apf_paper1_profile").value(config.apf_paper1_profile);
            w.key("trajectory_optimizer_enabled").value(config.trajectory_optimizer_enabled);
            w.end_object();

            w.key("task_waypoints").array_vec3(result.task_waypoints);

            w.key("metrics").begin_object();
            w.key("mean").array_double(result.metrics.mean);
            w.key("max").array_double(result.metrics.max);
            w.key("final").array_double(result.metrics.final);
            w.end_object();

            w.key("summary").begin_object();
            w.key("mean_error_overall").value(overall_mean);
            w.key("max_error_overall").value(overall_max);
            w.key("final_error_overall").value(overall_final);
            w.key("collision_count").value(static_cast<int>(result.collision_log.size()));
            w.key("replan_count").value(static_cast<int>(result.planning_events.size()));
            w.key("fault_count").value(static_cast<int>(result.fault_log.size()));
            w.key("planned_path_length").value(static_cast<int>(result.planned_path.size()));
            w.key("executed_path_length").value(static_cast<int>(result.executed_path.size()));
            w.end_object();

            w.key("safety_metrics").begin_object();
            w.key("min_inter_drone_distance").value(result.safety_metrics.min_inter_drone_distance);
            w.key("downwash_hits").value(result.safety_metrics.downwash_hits);
            w.end_object();

            w.end_object();
            out << "\n";
            out.close();
            std::cout << "结果文件: " << json_path.string() << "\n";

            // 调 Python 报告管道
            std::string rel = json_path.string();
            std::string cmd = "python \"../experiments/report_cpp_results.py\" \"" + rel + "\"";
            std::cout << "生成报告: " << std::flush;
            int ret = std::system(cmd.c_str());
            if (ret == 0) {
                std::cout << (run_dir / "cpp_report.md").string() << "\n";
            } else {
                std::cout << "跳过 (code=" << ret << ")\n";
            }
        }
    }

    return 0;
}
