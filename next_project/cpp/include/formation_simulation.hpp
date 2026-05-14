#pragma once

#include <array>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "controller.hpp"
#include "drone.hpp"
#include "topology.hpp"
#include "wind_field.hpp"

namespace sim {

struct FormationSwitchEvent {
    double trigger_time;
    std::string target_formation;
    double transition_time;
};

struct SimulationConfig {
    double dt = 0.012;
    double max_sim_time = 30.0;
    bool use_smc = true;
    bool use_backstepping = false;

    int num_followers = 3;
    double formation_spacing = 2.0;
    std::string initial_formation = "v_shape";

    double leader_gain_scale = 0.9;
    double leader_max_vel = 4.0;
    double leader_max_acc = 5.0;

    double follower_gain_scale = 1.0;
    double follower_max_vel = 10.0;
    double follower_max_acc = 10.0;

    unsigned int leader_wind_seed = 42;
    unsigned int follower_wind_seed_start = 100;

    double wp_radius = 1.5;
    double wp_radius_final = 0.3;
    double leader_acc_alpha = 0.2;

    std::vector<FormationSwitchEvent> formation_schedule{};
    bool formation_adaptation_enabled = false;
    std::vector<std::string> formation_adaptation_candidates{"diamond", "v_shape", "triangle", "line"};
    double formation_adaptation_min_hold_time = 1.0;
    double formation_adaptation_recovery_margin = 0.15;
    double formation_adaptation_transition_time = 1.5;
    bool formation_lookahead_enabled = false;
    double formation_lookahead_distance = 4.0;
    double formation_lookahead_turn_threshold_rad = 1.05;
    double formation_lookahead_min_interval = 0.8;
    bool formation_lookahead_rrt_enabled = false;
    int formation_lookahead_rrt_max_iter = 800;
    double formation_lookahead_rrt_rewire_radius = 1.2;
    bool trajectory_optimizer_enabled = false;
    std::string trajectory_optimizer_method = "moving_average";
    std::vector<Vec3> custom_initial_offsets{};
    std::vector<Vec3> waypoints{
        Vec3{0.0, 0.0, 0.0},
        Vec3{0.0, 0.0, 15.0},
        Vec3{20.0, 0.0, 15.0},
        Vec3{20.0, 20.0, 15.0},
        Vec3{0.0, 20.0, 15.0},
        Vec3{0.0, 0.0, 0.0},
    };
};

struct SimulationMetrics {
    std::vector<double> mean;
    std::vector<double> max;
    std::vector<double> final;
};

struct FormationSafetyMetrics {
    double min_inter_drone_distance = 0.0;
    int downwash_hits = 0;
};

struct PlanningEvent {
    double t = 0.0;
    std::string phase;
    std::string planner;
    int segment_index = -1;
    double wall_time_s = 0.0;
    int point_count = 0;
    bool accepted = false;
    std::string fallback_reason;
};

struct WaypointEvent {
    double t = 0.0;
    std::string type = "waypoint_reached";
    int index = 0;
    double distance = 0.0;
};

struct CollisionEvent {
    double t = 0.0;
    std::string drone;
    Vec3 pos;
};

struct FormationAdaptationEvent {
    double t = 0.0;
    std::string kind;
    std::string from;
    std::string to;
    std::string reason;
    bool has_channel_width = false;
    std::array<double, 3> channel_width{0.0, 0.0, 0.0};
    bool has_selected_envelope = false;
    std::array<double, 3> selected_envelope{0.0, 0.0, 0.0};
    bool has_clearance_margin = false;
    double clearance_margin = 0.0;
    bool has_max_turn_angle = false;
    double max_turn_angle_rad = 0.0;
    std::string planner;
    std::string goal_kind;
    int goal_count = 0;
    int point_count = 0;
    bool blocked_by_hold_time = false;
};

struct SimulationResult {
    std::vector<double> time;
    std::vector<Vec3> leader;
    std::vector<std::vector<Vec3>> followers;
    std::vector<std::vector<Vec3>> targets;
    std::vector<std::vector<Vec3>> error_vectors;
    std::vector<std::vector<double>> errors;
    SimulationMetrics metrics;
    int completed_waypoint_count = 0;
    std::vector<Vec3> waypoints;
    std::vector<Vec3> task_waypoints;
    std::vector<Vec3> planned_path;
    std::vector<Vec3> replanned_waypoints;
    std::vector<Vec3> executed_path;
    std::vector<PlanningEvent> planning_events;
    std::vector<WaypointEvent> waypoint_events;
    std::vector<CollisionEvent> collision_log;
    std::vector<FormationAdaptationEvent> formation_adaptation_events;
    std::vector<std::string> fault_log;
    FormationSafetyMetrics safety_metrics;
};

class FormationSimulation {
public:
    explicit FormationSimulation(const SimulationConfig& config = SimulationConfig{});

    SimulationResult run();

private:
    void build_followers();
    void build_winds();
    void maybe_switch_formation(double time_now);

    static void apply_controller_profile(Controller* ctrl, double gain_scale, double max_vel, double max_acc);

public:
    SimulationConfig config_;
    double dt_;
    double max_sim_time_;
    bool use_smc_;

    Drone leader_;
    std::vector<Drone> followers_;

    std::unique_ptr<Controller> leader_ctrl_;
    std::vector<std::unique_ptr<Controller>> follower_ctrls_;

    WindField leader_wind_;
    std::vector<WindField> winds_;

    FormationTopology topology_;
    std::size_t next_switch_idx_ = 0;
};

}  // namespace sim
