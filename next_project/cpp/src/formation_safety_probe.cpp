#include <array>
#include <iostream>
#include <vector>

#include "formation_safety.hpp"
#include "obstacle_scenario.hpp"

int main() {
    using sim::DownwashZone;
    using sim::ObstacleConfig;
    using sim::ObstacleField;
    using sim::ObstacleScenarioSimulation;
    using sim::Vec3;

    const DownwashZone zone = sim::downwash_zone(0.45, 0.80);

    const bool downwash_hit = sim::is_in_downwash_zone(
        Vec3{0.05, 0.00, 1.55},
        Vec3{0.00, 0.00, 1.00},
        zone);
    const bool downwash_clear = sim::is_in_downwash_zone(
        Vec3{0.80, 0.00, 1.55},
        Vec3{0.00, 0.00, 1.00},
        zone);

    const std::vector<Vec3> compact_positions{
        Vec3{0.00, 0.00, 1.00},
        Vec3{0.20, 0.00, 1.00},
        Vec3{0.70, 0.00, 1.00},
    };
    const double min_dist = sim::min_inter_drone_distance(compact_positions);

    const std::vector<Vec3> reserved_positions{Vec3{0.00, 0.00, 1.00}};
    const bool recovery_ready = sim::nominal_target_ready_for_recovery(
        Vec3{1.00, 0.00, 1.00},
        reserved_positions,
        nullptr,
        0.80,
        true,
        0.30,
        0.35,
        &zone,
        0.10);
    const bool recovery_blocked = sim::nominal_target_ready_for_recovery(
        Vec3{0.05, 0.00, 1.55},
        reserved_positions,
        nullptr,
        0.80,
        true,
        0.30,
        0.35,
        &zone,
        0.10);

    ObstacleConfig cfg;
    cfg.max_sim_time = 0.1;
    cfg.dt = 0.02;
    cfg.num_followers = 1;
    cfg.formation_spacing = 0.2;
    cfg.initial_formation = "line";
    cfg.enable_obstacles = true;
    cfg.safety_margin = 0.2;
    cfg.formation_safety_enabled = true;
    cfg.formation_min_inter_drone_distance = 0.35;
    cfg.formation_downwash_radius = 0.25;
    cfg.formation_downwash_height = 0.6;
    cfg.waypoints = {
        Vec3{0.0, 0.0, 1.0},
        Vec3{0.3, 0.0, 1.0},
    };

    ObstacleField field;
    field.add_sphere(Vec3{0.05, 0.00, 1.55}, 0.18);

    ObstacleScenarioSimulation scenario(cfg);
    scenario.set_obstacles(
        field,
        std::array<Vec3, 2>{Vec3{-1.0, -1.0, 0.0}, Vec3{2.0, 2.0, 2.0}});

    const Vec3 leader_pos{0.0, 0.0, 1.0};
    const Vec3 nominal{0.45, 0.0, 1.0};
    const Vec3 blocked_raw{0.05, 0.0, 1.55};
    const Vec3 current{0.45, 0.2, 1.0};
    const std::vector<Vec3> reserved{leader_pos};

    const Vec3 safe_target = scenario.safe_follower_target(leader_pos, blocked_raw, &current);
    const Vec3 deconflicted = scenario.deconflict_follower_target(
        leader_pos, safe_target, nominal, reserved, 0, &current);
    scenario.formation_recovery_counts_[0] = cfg.formation_recovery_hold_steps - 1;
    const Vec3 recovered = scenario.deconflict_follower_target(
        leader_pos, current, nominal, reserved, 0, &current);
    const auto result = scenario.run();

    std::cout << "downwash_hit=" << (downwash_hit ? 1 : 0)
              << " downwash_clear=" << (downwash_clear ? 1 : 0)
              << " min_dist=" << min_dist
              << " recovery_ready=" << (recovery_ready ? 1 : 0)
              << " recovery_blocked=" << (recovery_blocked ? 1 : 0)
              << " safe_target_shift=" << sim::norm(safe_target - blocked_raw)
              << " deconflicted_y=" << deconflicted.y
              << " recovered_dx=" << sim::norm(recovered - nominal)
              << " run_min_inter=" << result.safety_metrics.min_inter_drone_distance
              << " run_downwash_hits=" << result.safety_metrics.downwash_hits
              << "\n";
    return 0;
}
