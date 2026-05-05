#include <algorithm>
#include <chrono>
#include <iostream>

#include "formation_simulation.hpp"
#include "visualization.hpp"

int main() {
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

    SimulationVisualizer visualizer("outputs");
    const auto figure_paths = visualizer.plot_all(result);

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

    return 0;
}
