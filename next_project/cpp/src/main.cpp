#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif

#include "formation_simulation.hpp"
#include "json_writer.hpp"
#include "visualization.hpp"

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

void write_sim_result_json(
    const std::filesystem::path& output_path,
    const sim::SimulationResult& result,
    double elapsed,
    const std::string& preset
) {
    namespace fs = std::filesystem;
    fs::create_directories(output_path.parent_path());
    std::ofstream out(output_path, std::ios::binary);
    if (!out) return;

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
    w.key("runtime_s").value(elapsed);

    w.key("summary").begin_object();
    w.key("mean_error_overall").value(overall_mean);
    w.key("max_error_overall").value(overall_max);
    w.key("final_error_overall").value(overall_final);
    w.key("collision_count").value(0);
    w.key("replan_count").value(0);
    w.key("fault_count").value(0);
    w.end_object();  // summary

    w.end_object();  // root
    out << "\n";
}

}  // namespace

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    using sim::FormationSimulation;
    using sim::SimulationConfig;
    using sim::SimulationVisualizer;
    using sim::Vec3;

    SimulationConfig config;
    config.max_sim_time = 30.0;
    config.use_smc = true;
    config.num_followers = 3;
    config.formation_spacing = 2.0;
    config.waypoints = {
        Vec3{0.0, 0.0, 0.0},
        Vec3{0.0, 0.0, 15.0},
        Vec3{20.0, 0.0, 15.0},
        Vec3{20.0, 20.0, 15.0},
        Vec3{0.0, 20.0, 15.0},
        Vec3{0.0, 0.0, 0.0},
    };

    FormationSimulation simulation(config);

    const auto start = std::chrono::high_resolution_clock::now();
    const sim::SimulationResult result = simulation.run();
    const auto end = std::chrono::high_resolution_clock::now();

    const double elapsed = std::chrono::duration<double>(end - start).count();

    const std::string preset = "basic";
    const std::filesystem::path run_dir =
        std::filesystem::path("outputs") / preset / timestamp_dir_name();
    SimulationVisualizer visualizer(run_dir.string());
    const auto figure_paths = visualizer.plot_all(result);

    const auto sim_result_path = run_dir / "sim_result.json";
    write_sim_result_json(sim_result_path, result, elapsed, preset);

    std::cout << "编队仿真完成\n";
    std::cout << "运行耗时: " << elapsed << " s\n";
    std::cout << "航点完成数: " << result.completed_waypoint_count << "/" << config.waypoints.size() << "\n";

    for (std::size_t idx = 0; idx < result.metrics.mean.size(); ++idx) {
        std::cout << "Follower " << (idx + 1)
                  << ": mean=" << result.metrics.mean[idx] << " m"
                  << ", max=" << result.metrics.max[idx] << " m"
                  << ", final=" << result.metrics.final[idx] << " m\n";
    }

    std::cout << "图表输出:\n";
    std::cout << "- 轨迹图: " << figure_paths.at("trajectory") << "\n";
    std::cout << "- 实时误差图: " << figure_paths.at("error_3d") << "\n";
    std::cout << "- 误差统计图: " << figure_paths.at("error_stats") << "\n";
    std::cout << "结果 JSON: " << sim_result_path.string() << "\n";

    return 0;
}
