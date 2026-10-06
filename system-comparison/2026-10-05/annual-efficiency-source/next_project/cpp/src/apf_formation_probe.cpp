#include <array>
#include <cmath>
#include <iostream>

#include "obstacle_scenario.hpp"

namespace sim {

ObstacleField make_probe_obstacles() {
    ObstacleField f;
    // Two offset spheres create a shallow, asymmetric obstacle field so
    // centroid-level formation APF produces a measurable trajectory change.
    f.add_sphere({1.1, 0.0, 1.0}, 0.35);
    f.add_sphere({1.6, 0.35, 1.0}, 0.25);
    return f;
}

double path_length(const std::vector<Vec3>& path) {
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        total += norm(path[i] - path[i - 1]);
    }
    return total;
}

}  // namespace sim

int main(int argc, char* argv[]) {
    bool enable_centroid = argc >= 2 && std::string(argv[1]) == "on";

    sim::ObstacleConfig cfg;
    cfg.max_sim_time = 1.2;
    cfg.dt = 0.02;
    cfg.use_smc = true;
    cfg.use_backstepping = false;
    cfg.num_followers = 2;
    cfg.formation_spacing = 0.6;
    cfg.initial_formation = "diamond";
    cfg.wp_radius = 0.08;
    cfg.wp_radius_final = 0.05;
    cfg.leader_max_vel = 0.8;
    cfg.leader_max_acc = 1.2;
    cfg.leader_gain_scale = 0.8;
    cfg.follower_gain_scale = 1.0;
    cfg.follower_max_vel = 2.5;
    cfg.follower_max_acc = 3.0;
    cfg.leader_acc_alpha = 0.35;
    cfg.enable_obstacles = false;
    cfg.planner_mode = "offline";
    cfg.safety_margin = 0.25;
    cfg.apf_paper1_profile = "aggressive";
    cfg.apf_formation_centroid = enable_centroid;
    cfg.apf_centroid_alpha = 0.3;
    cfg.apf_centroid_beta = 0.8;
    cfg.waypoints = {
        sim::Vec3{0.0, 0.0, 1.0},
        sim::Vec3{2.4, 0.0, 1.0},
    };

    sim::ObstacleScenarioSimulation scenario(cfg);
    scenario.set_obstacles(
        sim::make_probe_obstacles(),
        std::array<sim::Vec3, 2>{sim::Vec3{-1.0, -1.5, 0.0}, sim::Vec3{4.0, 1.5, 2.5}});
    sim::SimulationResult result = scenario.run();

    const sim::Vec3 leader_final = result.executed_path.empty() ? sim::Vec3{} : result.executed_path.back();
    std::cout << "centroid=" << (enable_centroid ? "on" : "off")
              << " path_len=" << sim::path_length(result.executed_path)
              << " final_x=" << leader_final.x
              << " final_y=" << leader_final.y
              << " follower0_final_err=" << (result.metrics.final.empty() ? 0.0 : result.metrics.final[0])
              << "\n";
    return 0;
}
