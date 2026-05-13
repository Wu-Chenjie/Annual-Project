#pragma once

#include <memory>
#include <string>
#include <vector>

#include "artificial_potential_field.hpp"
#include "astar_planner.hpp"
#include "fault_detector.hpp"
#include "firi.hpp"
#include "formation_safety.hpp"
#include "formation_apf.hpp"
#include "formation_simulation.hpp"
#include "hybrid_astar_planner.hpp"
#include "map_loader.hpp"
#include "occupancy_grid.hpp"
#include "replanner.hpp"
#include "sensor.hpp"

namespace sim {

struct ObstacleConfig : SimulationConfig {
    bool enable_obstacles = false;
    std::string map_file;
    double planner_resolution = 0.4;
    double safety_margin = 0.3;
    bool planner_sdf_aware = true;
    bool planner_esdf_aware = true;
    double detect_margin_scale = 0.5;
    std::string planner_kind = "astar";
    std::string planner_mode = "offline";  // "offline" | "online"
    bool planner_initial_map_unknown = false;
    double planner_replan_interval = 0.4;
    double planner_horizon = 6.0;
    bool planner_use_formation_envelope = false;
    bool formation_safety_enabled = false;
    double formation_min_inter_drone_distance = 0.35;
    double formation_downwash_radius = 0.45;
    double formation_downwash_height = 0.80;
    int formation_recovery_hold_steps = 4;
    double formation_recovery_clearance_margin = 0.10;
    bool planner_has_z_bounds = false;
    double planner_z_min = 0.0;
    double planner_z_max = 0.0;
    double plan_clearance_extra = 0.0;
    bool sensor_enabled = false;
    double sensor_max_range = 8.0;
    double sensor_noise_std = 0.02;
    std::string apf_paper1_profile = "off";  // "off" | "conservative" | "aggressive"
    double apf_comm_range = 10.0;
    double apf_centroid_alpha = 0.4;
    double apf_centroid_beta = 0.6;
    bool apf_dev_override = false;
    bool apf_adaptive_n_decay = false;
    bool apf_formation_centroid = false;
    bool apf_comm_constraint = false;
    bool apf_rotational_escape = false;
    bool replan_adaptive_interval = false;
    double replan_interval_min = 0.1;
    double replan_interval_max = 1.0;
    bool firi_enabled = true;
    double firi_sample_step = 0.2;
    int firi_max_projection_iter = 10;
    bool danger_mode_enabled = false;
    double danger_sensor_threshold = 2.0;
    double danger_sensor_safe_threshold = 4.0;
    double danger_sdf_threshold = 0.5;
    double danger_hysteresis_margin = 0.5;
    bool fault_injection_enabled = false;
    bool fault_detection_enabled = false;
    bool fault_reconfig_enabled = false;
    double fault_detector_max_acc = 10.0;
    double fault_detector_pos_dev = 5.0;
    int fault_detector_saturate_steps = 50;
    double fault_injection_time = -1.0;
    int fault_injection_follower = 0;
    int fault_injection_rotor = 1;
    double fault_injection_severity = 0.3;
};

class TelemetryObserver {
public:
    void clear_runtime();
    void clear_all();
    void record_planning(const PlanningEvent& event);
    void record_waypoint(const WaypointEvent& event);
    void record_collision(const CollisionEvent& event);

    const std::vector<PlanningEvent>& planning_events() const { return planning_events_; }
    const std::vector<WaypointEvent>& waypoint_events() const { return waypoint_events_; }
    const std::vector<CollisionEvent>& collision_events() const { return collision_events_; }

private:
    std::vector<PlanningEvent> planning_events_;
    std::vector<WaypointEvent> waypoint_events_;
    std::vector<CollisionEvent> collision_events_;
};

class ObstacleScenarioSimulation {
public:
    explicit ObstacleScenarioSimulation(const ObstacleConfig& config);

    SimulationResult run();

public:
    void setup_obstacles();
    void set_obstacles(ObstacleField field, const std::array<Vec3, 2>& bounds);
    void setup_online();
    std::vector<Vec3> plan_offline();
    double inflate_r() const;
    std::array<double, 3> inflate_margin_xyz() const;
    double compute_clearance() const;
    void apply_planning_z_bounds();
    void rebuild_planning_grid();
    std::tuple<double, double, double> channel_width_from_sensor(const std::array<double, 6>* sensor_reading) const;
    std::vector<Vec3> sanitize_waypoints(const std::vector<Vec3>& waypoints) const;
    Vec3 project_to_planning_free(const Vec3& point, const Vec3* prefer = nullptr,
                                  double max_radius_m = -1.0) const;
    Vec3 safe_follower_target(const Vec3& leader_pos, const Vec3& raw_target,
                              const Vec3* current_pos = nullptr) const;
    Vec3 deconflict_follower_target(const Vec3& leader_pos, const Vec3& candidate_target,
                                    const Vec3& nominal_target,
                                    const std::vector<Vec3>& reserved_positions,
                                    int follower_idx,
                                    const Vec3* current_pos = nullptr) const;
    ImprovedArtificialPotentialField build_apf() const;
    std::unique_ptr<FormationAPF> build_formation_apf() const;
    std::vector<Vec3> enforce_path_clearance(const std::vector<Vec3>& path, double min_clearance);
    // Returns the minimum SDF clearance sampled along every segment of a candidate path.
    // Stops early once the sampled clearance falls below min_clearance.
    double path_segment_clearance(const std::vector<Vec3>& path, double min_clearance) const;
    // Path-level acceptance gate: true only when all sampled path segments satisfy min_clearance.
    bool path_is_clearance_safe(const std::vector<Vec3>& path, double min_clearance) const;
    Vec3 obstacle_repulsion_acc(const Vec3& pos, const Vec3& goal,
                                const std::vector<Vec3>& other_positions = {});
    std::vector<Vec3> stitch_local_path_to_task_goal(const std::vector<Vec3>& local_path,
                                                     const Vec3& task_goal) const;
    bool apply_formation_adaptation(double time_now,
                                    const std::tuple<double, double, double>* channel_width,
                                    double clearance_margin,
                                    bool has_clearance_margin,
                                    const std::string& reason_override = "");
    std::vector<Vec3> maybe_lookahead_escape(double time_now,
                                             const Vec3& leader_pos,
                                             const std::vector<Vec3>& active_path,
                                             const Vec3& task_goal);
    std::vector<Vec3> path_window_from_position(const std::vector<Vec3>& path,
                                                const Vec3& position,
                                                double lookahead_distance) const;
    double path_max_turn_angle(const std::vector<Vec3>& path) const;

    ObstacleConfig config_;
    FormationSimulation formation_;
    ObstacleField obstacles_;
    std::array<Vec3, 2> map_bounds_{};
    OccupancyGrid grid_;
    std::unique_ptr<SDFAwareGrid> sdf_grid_;
    std::unique_ptr<WindowReplanner> replanner_;
    std::unique_ptr<RangeSensor6> sensor_;
    ImprovedArtificialPotentialField apf_;
    std::unique_ptr<FormationAPF> formation_apf_;
    double collision_margin_;
    int num_followers_;

    std::vector<CollisionEvent> collision_log_;
    std::vector<Vec3> planned_path_;
    std::vector<Vec3> waypoints_;
    std::vector<Vec3> task_waypoints_;
    std::vector<Vec3> replanned_waypoints_;
    std::vector<Vec3> executed_path_;
    std::vector<std::string> fault_log_;
    TelemetryObserver observer_;
    mutable std::vector<int> formation_recovery_counts_;
    std::vector<FormationAdaptationEvent> formation_adaptation_events_;
    double last_formation_adaptation_time_ = -1.0;
    double last_lookahead_escape_time_ = -1.0;
    DownwashZone downwash_zone_;
};

}  // namespace sim
