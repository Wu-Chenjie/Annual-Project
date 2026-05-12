#pragma once

#include <stdexcept>
#include <string>

#include "formation_simulation.hpp"
#include "obstacle_scenario.hpp"

namespace sim {

inline ObstacleConfig config_basic();
inline ObstacleConfig config_obstacle();
inline ObstacleConfig config_warehouse();
inline ObstacleConfig config_warehouse_a();
inline ObstacleConfig config_warehouse_online();
inline ObstacleConfig config_warehouse_danger();
inline ObstacleConfig config_fault_tolerance();
inline ObstacleConfig config_fault_tolerance_online();
inline ObstacleConfig config_school_corridor();
inline ObstacleConfig config_school_corridor_online();
inline ObstacleConfig config_company_cubicles();
inline ObstacleConfig config_company_cubicles_online();
inline ObstacleConfig config_meeting_room();
inline ObstacleConfig config_meeting_room_online();
inline ObstacleConfig config_laboratory();
inline ObstacleConfig config_laboratory_online();
inline ObstacleConfig config_custom();

/// Return a named simulation preset.
/// C++ is still a runnable subset of the Python mainline, but now includes
/// engineering FIRI and GNN Danger variants.
inline ObstacleConfig get_config(const std::string& preset) {
    if (preset == "basic") return config_basic();
    if (preset == "obstacle") return config_obstacle();
    if (preset == "warehouse") return config_warehouse();
    if (preset == "warehouse_a") return config_warehouse_a();
    if (preset == "warehouse_online") return config_warehouse_online();
    if (preset == "warehouse_danger") return config_warehouse_danger();
    if (preset == "fault_tolerance") return config_fault_tolerance();
    if (preset == "fault_tolerance_online") return config_fault_tolerance_online();
    if (preset == "school_corridor") return config_school_corridor();
    if (preset == "school_corridor_online") return config_school_corridor_online();
    if (preset == "company_cubicles") return config_company_cubicles();
    if (preset == "company_cubicles_online") return config_company_cubicles_online();
    if (preset == "meeting_room") return config_meeting_room();
    if (preset == "meeting_room_online") return config_meeting_room_online();
    if (preset == "laboratory") return config_laboratory();
    if (preset == "laboratory_online") return config_laboratory_online();
    if (preset == "custom") return config_custom();
    throw std::invalid_argument("unknown preset: " + preset);
}

inline ObstacleConfig config_basic() {
    ObstacleConfig c;
    c.max_sim_time = 30.0; c.use_smc = true; c.use_backstepping = false;
    c.num_followers = 3; c.formation_spacing = 2.0; c.initial_formation = "v_shape";
    c.waypoints = {{0,0,0},{0,0,15},{20,0,15},{20,20,15},{0,20,15},{0,0,0}};
    return c;
}

inline ObstacleConfig config_obstacle() {
    ObstacleConfig c;
    c.max_sim_time = 40.0; c.use_smc = true; c.use_backstepping = false;
    c.num_followers = 2; c.formation_spacing = 0.3; c.initial_formation = "diamond";
    c.wp_radius = 0.06; c.wp_radius_final = 0.03;
    c.leader_max_vel = 0.4; c.leader_max_acc = 0.5; c.leader_gain_scale = 0.8;
    c.follower_gain_scale = 1.0; c.leader_acc_alpha = 0.35;
    c.enable_obstacles = true; c.map_file = "../maps/sample_simple.json";
    c.planner_kind = "astar"; c.planner_mode = "offline";
    c.planner_resolution = 0.3; c.safety_margin = 0.3;
    c.waypoints = {{2,10,2},{20,10,2}};
    return c;
}

inline ObstacleConfig config_warehouse() {
    ObstacleConfig c;
    c.max_sim_time = 100.0; c.use_smc = true; c.use_backstepping = true;
    c.num_followers = 3; c.formation_spacing = 0.25; c.initial_formation = "diamond";
    c.wp_radius = 0.8; c.wp_radius_final = 0.4;
    c.leader_max_vel = 1.1; c.leader_max_acc = 1.4; c.leader_gain_scale = 0.65;
    c.follower_gain_scale = 0.9; c.follower_max_vel = 4.5; c.follower_max_acc = 4.5;
    c.leader_acc_alpha = 0.30;
    c.enable_obstacles = true; c.map_file = "../maps/sample_warehouse.json";
    c.planner_kind = "astar"; c.planner_mode = "online";
    c.planner_resolution = 0.25; c.safety_margin = 0.3;
    c.planner_use_formation_envelope = true; c.plan_clearance_extra = 0.55;
    c.formation_safety_enabled = true; c.formation_min_inter_drone_distance = 0.25;
    c.sensor_enabled = true; c.planner_replan_interval = 0.6; c.planner_horizon = 9.0;
    c.replan_adaptive_interval = false; c.replan_interval_min = 0.1; c.replan_interval_max = 1.0;
    c.danger_mode_enabled = false;
    c.detect_margin_scale = 0.5;
    c.formation_schedule = {};
    c.waypoints = {{4.2,13.2,5.7},{11.4,10.2,2.4},{19.8,15.2,7},{29.2,22.4,3.7},{35.8,10,8.3},{42,4.8,1}};
    return c;
}

inline ObstacleConfig config_warehouse_a() {
    ObstacleConfig c = config_warehouse();
    c.planner_kind = "astar";
    c.planner_mode = "online";
    c.sensor_enabled = true;
    c.planner_replan_interval = 0.4;
    c.planner_horizon = 6.0;
    c.replan_adaptive_interval = false;
    c.danger_mode_enabled = true;
    return c;
}

inline ObstacleConfig config_warehouse_online() {
    ObstacleConfig c;
    c.max_sim_time = 30.0; c.use_smc = true; c.use_backstepping = true;
    c.num_followers = 3; c.formation_spacing = 0.5; c.initial_formation = "diamond";
    c.wp_radius = 0.8; c.wp_radius_final = 0.4;
    c.leader_max_vel = 2.0; c.leader_max_acc = 2.5; c.leader_gain_scale = 0.80;
    c.follower_gain_scale = 1.0; c.follower_max_vel = 8.0; c.follower_max_acc = 8.0;
    c.leader_acc_alpha = 0.30;
    c.enable_obstacles = true; c.map_file = "../maps/sample_warehouse.json";
    c.planner_kind = "astar"; c.planner_mode = "online";
    c.planner_resolution = 0.4; c.safety_margin = 0.3;
    c.sensor_enabled = true; c.planner_replan_interval = 2.0; c.planner_horizon = 6.0;
    c.replan_adaptive_interval = false;
    c.waypoints = {{4.2,13.2,5.7},{19.8,15.2,7},{42,4.8,1}};
    return c;
}

inline ObstacleConfig config_warehouse_danger() {
    ObstacleConfig c = config_warehouse_online();
    c.planner_replan_interval = 0.4;
    c.replan_adaptive_interval = false;
    c.replan_interval_min = 0.1;
    c.replan_interval_max = 1.0;
    c.danger_mode_enabled = true;
    return c;
}

inline ObstacleConfig config_fault_tolerance() {
    ObstacleConfig c = config_warehouse();
    c.max_sim_time = 30.0;
    c.planner_mode = "offline";
    c.sensor_enabled = false;
    c.danger_mode_enabled = false;
    c.replan_adaptive_interval = false;
    c.fault_injection_enabled = true;
    c.fault_detection_enabled = true;
    c.fault_reconfig_enabled = true;
    c.fault_detector_max_acc = 10.0;
    c.fault_detector_pos_dev = 5.0;
    c.fault_detector_saturate_steps = 50;
    c.waypoints = {{4.2,13.2,5.7},{19.8,15.2,7},{42,4.8,1}};
    return c;
}

inline ObstacleConfig config_fault_tolerance_online() {
    ObstacleConfig c = config_warehouse_danger();
    c.max_sim_time = 35.0;
    c.replan_adaptive_interval = true;
    c.fault_injection_enabled = true;
    c.fault_detection_enabled = true;
    c.fault_reconfig_enabled = true;
    c.fault_detector_max_acc = 10.0;
    c.fault_detector_pos_dev = 5.0;
    c.fault_detector_saturate_steps = 50;
    c.waypoints = {{4.2,13.2,5.7},{19.8,15.2,7},{42,4.8,1}};
    return c;
}

inline ObstacleConfig config_school_corridor() {
    ObstacleConfig c;
    c.max_sim_time = 500.0; c.use_smc = true; c.use_backstepping = true;
    c.num_followers = 3; c.formation_spacing = 0.4; c.initial_formation = "line";
    c.wp_radius = 0.5; c.wp_radius_final = 0.3;
    c.leader_max_vel = 1.5; c.leader_max_acc = 2.0; c.leader_gain_scale = 0.80;
    c.follower_gain_scale = 1.0; c.follower_max_vel = 6.0; c.follower_max_acc = 6.0;
    c.leader_acc_alpha = 0.30;
    c.enable_obstacles = true; c.map_file = "../maps/school_corridor.json";
    c.planner_kind = "astar"; c.planner_mode = "offline";
    c.planner_resolution = 0.3; c.safety_margin = 0.25;
    c.planner_has_z_bounds = true; c.planner_z_min = 1.5; c.planner_z_max = 3.0;
    c.sensor_enabled = true; c.planner_replan_interval = 0.4; c.planner_horizon = 6.0;
    c.danger_mode_enabled = true;
    c.formation_schedule = {{20,"diamond",2},{38,"line",3}};
    c.waypoints = {{1,2,2},{10,2,2},{23,2,2},{27.5,1,2},{34,5.5,2},{41,2,2},{47,2,2.5}};
    return c;
}

inline ObstacleConfig config_school_corridor_online() {
    ObstacleConfig c = config_school_corridor();
    c.max_sim_time = 40.0;
    c.planner_mode = "online";
    c.planner_replan_interval = 0.3; c.planner_horizon = 5.0;
    c.replan_adaptive_interval = true;
    c.formation_schedule = {{16,"diamond",2},{30,"line",3}};
    c.waypoints = {{1,2,2},{15,2,2},{27.5,1,2},{41,2,2},{47,2,2.5}};
    return c;
}

inline ObstacleConfig config_company_cubicles() {
    ObstacleConfig c;
    c.max_sim_time = 50.0; c.use_smc = true; c.use_backstepping = true;
    c.num_followers = 3; c.formation_spacing = 0.5; c.initial_formation = "diamond";
    c.wp_radius = 0.6; c.wp_radius_final = 0.3;
    c.leader_max_vel = 1.8; c.leader_max_acc = 2.0; c.leader_gain_scale = 0.80;
    c.follower_gain_scale = 1.0; c.follower_max_vel = 7.0; c.follower_max_acc = 7.0;
    c.leader_acc_alpha = 0.30;
    c.enable_obstacles = true; c.map_file = "../maps/company_cubicles.json";
    c.planner_kind = "hybrid_astar"; c.planner_mode = "offline";
    c.planner_resolution = 0.3; c.safety_margin = 0.3;
    c.sensor_enabled = false;
    c.formation_schedule = {{18,"line",4},{32,"diamond",4}};
    c.waypoints = {{3,4,2.5},{13,4,2.5},{22,4,2.5},{22,12,2.5},{13,12,2.5},{3,12,2.5},{3,21,2.5}};
    return c;
}

inline ObstacleConfig config_company_cubicles_online() {
    ObstacleConfig c = config_company_cubicles();
    c.max_sim_time = 400.0;
    c.planner_mode = "online";
    c.sensor_enabled = true; c.planner_replan_interval = 2.0; c.planner_horizon = 5.0;
    c.formation_schedule = {{14,"line",4},{26,"diamond",4}};
    c.waypoints = {{3,4,2.5},{13,4,2.5},{22,12,2.5},{13,12,2.5},{3,21,2.5}};
    return c;
}

inline ObstacleConfig config_meeting_room() {
    ObstacleConfig c;
    c.max_sim_time = 40.0; c.use_smc = true; c.use_backstepping = true;
    c.num_followers = 2; c.formation_spacing = 0.3; c.initial_formation = "diamond";
    c.wp_radius = 0.4; c.wp_radius_final = 0.2;
    c.leader_max_vel = 0.8; c.leader_max_acc = 1.0; c.leader_gain_scale = 0.80;
    c.follower_gain_scale = 1.0; c.follower_max_vel = 5.0; c.follower_max_acc = 5.0;
    c.leader_acc_alpha = 0.35;
    c.enable_obstacles = true; c.map_file = "../maps/meeting_room.json";
    c.planner_kind = "astar"; c.planner_mode = "offline";
    c.planner_resolution = 0.2; c.safety_margin = 0.2;
    c.planner_has_z_bounds = true; c.planner_z_min = 1.4; c.planner_z_max = 2.8;
    c.sensor_enabled = true; c.planner_replan_interval = 2.0; c.planner_horizon = 4.0;
    c.waypoints = {{1,5,2},{7,1.5,2},{13.5,5,2},{7,11,2},{1,8,2}};
    return c;
}

inline ObstacleConfig config_meeting_room_online() {
    ObstacleConfig c = config_meeting_room();
    c.max_sim_time = 35.0;
    c.wp_radius = 0.3; c.wp_radius_final = 0.15;
    c.planner_mode = "online";
    c.planner_horizon = 3.5;
    c.waypoints = {{1,5,2},{7,1.5,2},{13.5,5,2},{7,11,2}};
    return c;
}

inline ObstacleConfig config_laboratory() {
    ObstacleConfig c;
    c.max_sim_time = 55.0; c.use_smc = true; c.use_backstepping = true;
    c.num_followers = 3; c.formation_spacing = 0.5; c.initial_formation = "v_shape";
    c.wp_radius = 0.6; c.wp_radius_final = 0.3;
    c.leader_max_vel = 1.5; c.leader_max_acc = 2.0; c.leader_gain_scale = 0.80;
    c.follower_gain_scale = 1.0; c.follower_max_vel = 7.0; c.follower_max_acc = 7.0;
    c.leader_acc_alpha = 0.30;
    c.enable_obstacles = true; c.map_file = "../maps/laboratory.json";
    c.planner_kind = "astar"; c.planner_mode = "offline";
    c.planner_resolution = 0.3; c.safety_margin = 0.3;
    c.planner_has_z_bounds = true; c.planner_z_min = 2.0; c.planner_z_max = 3.2;
    c.sensor_enabled = true; c.planner_replan_interval = 2.0; c.planner_horizon = 6.0;
    c.formation_schedule = {{20,"diamond",3},{38,"line",5}};
    c.waypoints = {{1,4,2.5},{3,10,2.5},{8,4,2.5},{13,4,2.5},{16,10,2.5},{21,12,2.5},{13,16,2.5},{3,16,2.5}};
    return c;
}

inline ObstacleConfig config_laboratory_online() {
    ObstacleConfig c = config_laboratory();
    c.max_sim_time = 45.0;
    c.planner_kind = "hybrid_astar"; c.planner_mode = "online";
    c.planner_horizon = 5.0;
    c.formation_schedule = {{16,"diamond",3},{30,"line",5}};
    c.waypoints = {{1,4,2.5},{3,10,2.5},{8,4,2.5},{16,10,2.5},{21,12,2.5},{3,16,2.5}};
    return c;
}

inline ObstacleConfig config_custom() {
    ObstacleConfig c;
    c.max_sim_time = 65.0;
    c.use_smc = true; c.use_backstepping = true;
    c.num_followers = 3; c.formation_spacing = 0.5; c.initial_formation = "diamond";
    c.leader_max_vel = 2.0; c.leader_max_acc = 2.5; c.leader_gain_scale = 0.80;
    c.follower_gain_scale = 1.0; c.follower_max_vel = 8.0; c.follower_max_acc = 8.0;
    c.leader_acc_alpha = 0.30;
    c.enable_obstacles = true; c.safety_margin = 0.3; c.detect_margin_scale = 0.5;
    c.planner_kind = "hybrid_astar"; c.planner_mode = "offline";
    c.planner_resolution = 0.4; c.planner_sdf_aware = true; c.planner_esdf_aware = true;
    c.sensor_enabled = false; c.planner_replan_interval = 0.4; c.planner_horizon = 6.0;
    c.replan_adaptive_interval = false; c.replan_interval_min = 0.1; c.replan_interval_max = 1.0;
    c.firi_enabled = true;
    c.danger_mode_enabled = false;
    c.formation_schedule = {{15,"line",6},{28,"diamond",6}};
    c.waypoints = {{4.2,13.2,5.7},{11.4,10.2,2.4},{19.8,15.2,7},{29.2,22.4,3.7},{35.8,10,8.3},{42,4.8,1}};
    c.map_file = "../maps/sample_warehouse.json";
    return c;
}

}  // namespace sim
