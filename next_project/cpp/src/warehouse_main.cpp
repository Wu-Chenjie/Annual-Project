#include <array>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <variant>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#endif

#include "config.hpp"
#include "json_writer.hpp"
#include "obstacles.hpp"
#include "obstacle_scenario.hpp"
#include "result_writer.hpp"
#include "visualization.hpp"

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

bool path_exists(const std::filesystem::path& path) {
    std::error_code ec;
    return std::filesystem::exists(path, ec);
}

std::string resolve_map_file(const std::string& raw) {
    if (raw.empty()) return raw;
    const std::filesystem::path path(raw);
    if (path_exists(path)) return path.string();

    std::vector<std::filesystem::path> candidates;
    candidates.push_back(std::filesystem::path("..") / ".." / "maps" / path.filename());
    candidates.push_back(std::filesystem::path("maps") / path.filename());
    candidates.push_back(std::filesystem::path("..") / "maps" / path.filename());
    candidates.push_back(std::filesystem::path("next_project") / "maps" / path.filename());

    for (const auto& candidate : candidates) {
        if (path_exists(candidate)) return candidate.string();
    }
    return raw;
}

struct CliOptions {
    std::string preset = "warehouse";
    std::string map_file_override;
    double max_sim_time = -1.0;
};

CliOptions parse_cli(int argc, char* argv[]) {
    CliOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--preset" && i + 1 < argc) {
            options.preset = argv[++i];
        } else if (arg == "--map-file" && i + 1 < argc) {
            options.map_file_override = argv[++i];
        } else if (arg == "--max-sim-time" && i + 1 < argc) {
            options.max_sim_time = std::stod(argv[++i]);
        } else if (arg.rfind("--", 0) != 0) {
            options.preset = arg;
        }
    }
    return options;
}

}  // namespace


int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    using sim::ObstacleConfig;
    using sim::ObstacleScenarioSimulation;
    using sim::SimulationVisualizer;
    using sim::Vec3;

    const CliOptions cli = parse_cli(argc, argv);
    const std::string preset = cli.preset;
    ObstacleConfig config = sim::get_config(preset);
    if (cli.max_sim_time > 0.0) {
        config.max_sim_time = cli.max_sim_time;
    }

    if (!cli.map_file_override.empty()) {
        config.map_file = resolve_map_file(cli.map_file_override);
    }

    const bool use_manual_warehouse = preset == "warehouse" && cli.map_file_override.empty();
    if (use_manual_warehouse) {
        config.map_file = "";
    } else {
        config.map_file = resolve_map_file(config.map_file);
    }

    std::cout << "===== C++ obstacle scenario: " << preset << " (Python-compatible subset) =====\n";

    auto t0 = std::chrono::high_resolution_clock::now();
    ObstacleScenarioSimulation sim(config);
    sim::ObstacleField report_obstacles;
    std::array<Vec3, 2> report_bounds{Vec3{}, Vec3{}};
    if (use_manual_warehouse) {
        report_obstacles = sim::make_warehouse();
        report_bounds = {Vec3{-3,-3,0}, Vec3{45,28,10}};
        sim.set_obstacles(report_obstacles, report_bounds);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    auto result = sim.run();
    auto t2 = std::chrono::high_resolution_clock::now();
    if (!use_manual_warehouse) {
        report_obstacles = sim.obstacles_;
        report_bounds = sim.map_bounds_;
    }

    double tp = std::chrono::duration<double>(t1 - t0).count();
    double ts = std::chrono::duration<double>(t2 - t1).count();

    std::cout << "规划: " << tp << "s | 仿真: " << ts << "s | 总: " << (tp+ts) << "s\n";
    std::cout << "路点: " << result.completed_waypoint_count << "/" << result.task_waypoints.size() << "\n";
    for (size_t i = 0; i < result.metrics.mean.size(); ++i)
        std::cout << "F" << (i+1) << ": mean=" << result.metrics.mean[i]
                  << " max=" << result.metrics.max[i]
                  << " final=" << result.metrics.final[i] << "\n";
    std::cout << "Safety: min_inter=" << result.safety_metrics.min_inter_drone_distance
              << " downwash_hits=" << result.safety_metrics.downwash_hits << "\n";

    const std::filesystem::path output_path =
        std::filesystem::path("outputs") / preset / timestamp_dir_name() / "sim_result.json";
    sim::write_result_json(output_path, result, tp, ts, preset, config, report_obstacles, report_bounds);
    {
        SimulationVisualizer visualizer(output_path.parent_path().string());
        const auto figure_paths = visualizer.plot_all(result);
        const auto error_it = figure_paths.find("error_3d");
        if (error_it != figure_paths.end()) {
            std::cout << "Real-time error figure: " << error_it->second << "\n";
        }
    }
    std::cout << "结果文件: " << output_path.string() << "\n";

    // Auto-generate Chinese report.md via Python report pipeline
    {
        // 使用相对路径避免中文绝对路径的 _wsystem 编码问题
        std::string rel_output = output_path.string();
        std::cout << "生成报告: " << std::flush;
        int ret = -1;
#ifdef _WIN32
        std::string cmd = "python \"..\\..\\experiments\\report_cpp_results.py\" \""
                        + rel_output + "\"";
        ret = std::system(cmd.c_str());
#else
        std::string cmd = "python \"../../experiments/report_cpp_results.py\" \""
                        + rel_output + "\"";
        ret = std::system(cmd.c_str());
#endif
        if (ret == 0) {
            std::filesystem::path report_path = output_path.parent_path() / "cpp_report.md";
            std::cout << report_path.string() << "\n";
        } else {
            std::cout << "跳过 (python 不可用或脚本执行失败, code=" << ret << ")\n";
        }
    }
    return 0;
}
