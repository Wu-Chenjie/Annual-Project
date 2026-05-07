from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def test_cpp_online_replanner_preserves_task_waypoint_layer():
    source = read("cpp/src/obstacle_scenario.cpp")
    header = read("cpp/include/obstacle_scenario.hpp")

    assert "task_waypoints_" in header
    assert "replanned_waypoints_" in header
    assert "executed_path_" in header
    assert "stitch_local_path_to_task_goal" in source
    assert "task_wp_idx" in source
    assert "active_path" in source
    assert "waypoints_ = new_path" not in source


def test_cpp_online_replanner_has_path_acceptance_clearance_gate():
    scenario_source = read("cpp/src/obstacle_scenario.cpp")
    scenario_header = read("cpp/include/obstacle_scenario.hpp")
    dynamic_header = read("cpp/include/dynamic_scenario.hpp")

    assert "path_segment_clearance" in scenario_header
    assert "path_is_clearance_safe" in scenario_header
    candidate_block = re.search(
        r"auto\s+candidate_path\s*=.*?stitch_local_path_to_task_goal.*?"
        r"if\s*\(\s*candidate_safe\s*\)\s*\{(?P<body>.*?)\n\s*\}",
        scenario_source,
        flags=re.DOTALL,
    )
    assert candidate_block is not None
    assert re.search(
        r"const\s+double\s+clearance\s*=\s*compute_clearance\s*\(\s*\)",
        scenario_source,
    )
    assert re.search(
        r"candidate_safe\s*=\s*path_is_clearance_safe\s*\(\s*candidate_path\s*,\s*clearance\s*\)",
        scenario_source,
    )
    assert re.search(
        r"candidate_path\s*=\s*enforce_path_clearance\s*\(\s*candidate_path\s*,\s*clearance\s*\).*?"
        r"candidate_safe\s*=\s*path_is_clearance_safe\s*\(\s*candidate_path\s*,\s*clearance\s*\)",
        scenario_source,
        flags=re.DOTALL,
    )
    assert "active_path = std::move(candidate_path)" in candidate_block.group("body")
    assert "path_segment_clearance" in dynamic_header
    assert "path_is_clearance_safe" in dynamic_header
    dynamic_accept_block = re.search(
        r"if\s*\(\s*path_is_clearance_safe\s*\(\s*selected_path\s*,\s*accept_clearance\s*\)\s*\)\s*\{"
        r"(?P<body>.*?)\n\s*\}",
        dynamic_header,
        flags=re.DOTALL,
    )
    assert dynamic_accept_block is not None
    assert "active_path = std::move(selected_path)" in dynamic_accept_block.group("body")
    assert "ensure_reachable_replan_grid(leader_pose, goal)" in dynamic_header


def test_cpp_replanner_has_risk_adaptive_interval_and_sensor_ttl():
    source = read("cpp/include/replanner.hpp")

    assert "enable_adaptive_interval" in source
    assert "current_interval()" in source
    assert "update_risk(leader_pose, sensor_reading)" in source
    assert "double interval_now = current_interval()" in source
    assert "sensor_ttl_" in source
    assert "static_occupied_" in source
    assert "decay_sensor_obstacles()" in source


def test_cpp_config_exposes_adaptive_interval_fields():
    header = read("cpp/include/obstacle_scenario.hpp")

    assert "bool replan_adaptive_interval" in header
    assert "double replan_interval_min" in header
    assert "double replan_interval_max" in header


def test_cpp_firi_refiner_is_wired_into_planning_paths():
    header = read("cpp/include/firi.hpp")
    source = read("cpp/src/firi.cpp")
    scenario = read("cpp/src/obstacle_scenario.cpp")
    cmake = read("cpp/CMakeLists.txt")

    assert "struct FIRICorridor" in header
    assert "class FIRIRefiner" in header
    assert "FIRIRefiner::refine" in source
    assert "analytic_separating_plane" in source
    assert "compute_inscribed_ball" in source
    assert "nearest_sphere_point" in source
    assert "nearest_cylinder_point" in source
    assert "nearest_aabb_point" in source
    assert "src/firi.cpp" in cmake
    assert "FIRIRefiner refiner" in scenario
    assert "config_.firi_enabled" in scenario


def test_cpp_gnn_danger_mode_is_wired_into_replanner():
    visibility_header = read("cpp/include/visibility_graph.hpp")
    visibility_source = read("cpp/src/visibility_graph.cpp")
    gnn_header = read("cpp/include/gnn_planner.hpp")
    gnn_source = read("cpp/src/gnn_planner.cpp")
    dual_mode = read("cpp/include/dual_mode.hpp")
    replanner = read("cpp/include/replanner.hpp")
    scenario = read("cpp/src/obstacle_scenario.cpp")
    config = read("cpp/include/config.hpp")
    cmake = read("cpp/CMakeLists.txt")

    assert "class VisibilityGraph" in visibility_header
    assert "VisibilityGraph::is_visible" in visibility_source
    assert "class GNNPlanner" in gnn_header
    assert "GNNPlanner::integrate_gnn" in gnn_source
    assert "GNNPlanner::extract_path_from_activities" in gnn_source
    assert "class DualModeScheduler" in dual_mode
    assert "set_danger_planner" in replanner
    assert "replan_danger" in replanner
    assert "PHASE_DANGER" in replanner
    assert "set_dual_mode" in scenario
    assert "danger_mode_enabled = true" in config
    assert "src/visibility_graph.cpp" in cmake
    assert "src/gnn_planner.cpp" in cmake


def test_cpp_fault_tolerance_is_wired_into_drone_topology_and_scenario():
    drone_header = read("cpp/include/drone.hpp")
    drone_source = read("cpp/src/drone.cpp")
    detector = read("cpp/include/fault_detector.hpp")
    topology_header = read("cpp/include/topology.hpp")
    topology_source = read("cpp/src/topology.cpp")
    scenario_header = read("cpp/include/obstacle_scenario.hpp")
    scenario_source = read("cpp/src/obstacle_scenario.cpp")
    config = read("cpp/include/config.hpp")
    result_header = read("cpp/include/formation_simulation.hpp")

    assert "inject_fault" in drone_header
    assert "fault_mask_" in drone_header
    assert "thrusts_cmd[i] *= fault_mask_[i]" in drone_source
    assert "class FaultDetector" in detector
    assert "pos_dev_threshold_" in detector
    assert "saturate_count_" in detector
    assert "class TopologyGraph" in topology_header
    assert "fault_reconfigure" in topology_header
    assert "TopologyGraph::best_reconfig_topology" in topology_source
    assert "fault_injection_enabled" in scenario_header
    assert "fault_detection_enabled" in scenario_header
    assert "fault_reconfig_enabled" in scenario_header
    assert "FaultDetector" in scenario_source
    assert "inject:follower_" in scenario_source
    assert "detect:follower_" in scenario_source
    assert "reconfigure:" in scenario_source
    assert "config_fault_tolerance" in config
    assert "config_fault_tolerance_online" in config
    assert "std::vector<std::string> fault_log" in result_header


def test_cpp_topology_and_obstacle_scenario_use_axis_envelopes_and_true_lambda2():
    topology_header = read("cpp/include/topology.hpp")
    topology_source = read("cpp/src/topology.cpp")
    scenario_source = read("cpp/src/obstacle_scenario.cpp")
    scenario_header = read("cpp/include/obstacle_scenario.hpp")
    safety_header = read("cpp/include/formation_safety.hpp")
    safety_source = read("cpp/src/formation_safety.cpp")
    safety_probe = read("cpp/src/formation_safety_probe.cpp")
    result_header = read("cpp/include/formation_simulation.hpp")
    grid_header = read("cpp/include/occupancy_grid.hpp")
    grid_source = read("cpp/src/occupancy_grid.cpp")
    warehouse_main = read("cpp/src/warehouse_main.cpp")

    assert "envelope_per_axis" in topology_header
    assert "auto_shrink" in topology_header
    assert "constexpr double kSigmaSquared = 4.0;" in topology_source
    assert "std::exp(-d2 / kSigmaSquared)" in topology_source
    assert "std::sort(eigenvals.begin(), eigenvals.end())" in topology_source
    assert "eigenvals[1]" in topology_source
    assert "std::array<double, 3> inflate_margin_xyz() const;" in scenario_header
    assert "formation_.topology_.envelope_per_axis()" in scenario_source
    assert "grid_ = grid_.inflate(inflate_margin_xyz())" in scenario_source
    assert "channel_width_from_sensor" in scenario_source
    assert "rebuild_planning_grid" in scenario_source
    assert "bool formation_safety_enabled = false;" in scenario_header
    assert "double formation_min_inter_drone_distance = 0.35;" in scenario_header
    assert "double formation_downwash_radius = 0.45;" in scenario_header
    assert "double formation_downwash_height = 0.80;" in scenario_header
    assert "int formation_recovery_hold_steps = 4;" in scenario_header
    assert "double formation_recovery_clearance_margin = 0.10;" in scenario_header
    assert "DownwashZone downwash_zone_;" in scenario_header
    assert "struct FormationSafetyConfig" in safety_header
    assert "struct DownwashZone" in safety_header
    assert "nominal_target_ready_for_recovery" in safety_header
    assert "min_inter_drone_distance" in safety_header
    assert "struct FormationSafetyMetrics" in result_header
    assert "FormationSafetyMetrics safety_metrics;" in result_header
    assert "DownwashZone downwash_zone(" in safety_source
    assert "bool nominal_target_ready_for_recovery(" in safety_source
    assert "sim::is_in_downwash_zone(" in safety_probe
    assert "sim::min_inter_drone_distance(" in safety_probe
    assert "sim::nominal_target_ready_for_recovery(" in safety_probe
    assert "Vec3 safe_follower_target(" in scenario_header
    assert "Vec3 deconflict_follower_target(" in scenario_header
    assert "const Vec3& nominal_target" in scenario_header
    assert "int follower_idx" in scenario_header
    assert "Vec3 nominal_target = ls.position + offsets[i];" in scenario_source
    assert "safe_follower_target(ls.position, nominal_target, &follower_pos)" in scenario_source
    assert "deconflict_follower_target(" in scenario_source
    assert "nominal_target_ready_for_recovery(" in scenario_source
    assert "downwash_zone_ = downwash_zone(" in scenario_source
    assert "formation_recovery_counts_" in scenario_header
    assert "formation_recovery_counts_.assign" in scenario_source
    assert "std::fill(formation_recovery_counts_.begin(), formation_recovery_counts_.end(), 0);" in scenario_source
    assert "counter >= config_.formation_recovery_hold_steps" in scenario_source
    assert "formation_safety_enabled" in scenario_source
    assert "result.safety_metrics.min_inter_drone_distance" in scenario_source
    assert "result.safety_metrics.downwash_hits" in scenario_source
    assert 'std::cout << "Safety: min_inter="' in warehouse_main
    assert "result.safety_metrics.downwash_hits" in warehouse_main
    assert '\\"task_waypoints\\":' in warehouse_main
    assert '\\"executed_path\\":' in warehouse_main
    assert '\\"fault_log\\":' in warehouse_main
    assert '\\"safety_metrics\\":{' in warehouse_main
    assert 'warehouse_result.json' in warehouse_main
    assert "scenario.safe_follower_target(" in safety_probe
    assert "scenario.deconflict_follower_target(" in safety_probe
    assert "scenario.formation_recovery_counts_[0]" in safety_probe
    assert "recovered_dx=" in safety_probe
    assert "run_min_inter=" in safety_probe
    assert "run_downwash_hits=" in safety_probe
    assert "OccupancyGrid inflate(const std::array<double, 3>& radius_xyz) const;" in grid_header
    assert "OccupancyGrid::inflate(const std::array<double, 3>& radius_xyz) const" in grid_source
    assert "if (nx2 + ny2 + nz2 > 1.0 + 1e-9) continue;" in grid_source


def test_cpp_sensor_and_dynamic_replay_use_analytic_ranges():
    sensor_header = read("cpp/include/sensor.hpp")
    dynamic_header = read("cpp/include/dynamic_scenario.hpp")

    assert "ray_aabb" in sensor_header
    assert "ray_sphere" in sensor_header
    assert "ray_cylinder" in sensor_header
    assert "field.obstacles()" in sensor_header
    assert "field.is_collision(p)" not in sensor_header
    assert "frame.sensor_readings = sensor_.sense(leader_pose, obstacles_);" in dynamic_header
    assert "config_.sensor_max_range, config_.sensor_max_range" not in dynamic_header


def test_cpp_dynamic_replay_exports_scope_and_trace_fields():
    dynamic_header = read("cpp/include/dynamic_scenario.hpp")
    dynamic_main = read("cpp/src/dynamic_main.cpp")

    assert 'std::string execution_scope = "leader_centric"' in dynamic_header
    assert 'std::string follower_pose_mode = "illustrative_lateral_offsets"' in dynamic_header
    assert 'std::string summary_mode = "shared_leader_frame_eval"' in dynamic_header
    assert "std::vector<Vec3> task_waypoints;" in dynamic_header
    assert "std::vector<Vec3> replanned_waypoints;" in dynamic_header
    assert "std::vector<Vec3> executed_path;" in dynamic_header
    assert "struct ReplayReplanEvent" in dynamic_header
    assert "struct ReplayCollisionEvent" in dynamic_header
    assert "struct ReplayFaultEvent" in dynamic_header
    assert "std::vector<ReplayReplanEvent> replan_events;" in dynamic_header
    assert "std::vector<std::array<double, 6>> sensor_logs;" in dynamic_header
    assert "std::vector<ReplayCollisionEvent> collision_log;" in dynamic_header
    assert "std::vector<ReplayFaultEvent> fault_log;" in dynamic_header
    assert 'std::string apf_profile_scope = "cpp_subset_runtime"' in dynamic_header
    assert "std::vector<std::string> apf_runtime_fields;" in dynamic_header
    assert "std::vector<std::string> apf_python_only_fields;" in dynamic_header
    assert "std::vector<std::string> apf_cpp_pending_fields;" in dynamic_header
    assert '\\"execution_scope\\":' in dynamic_main
    assert '\\"apf_profile_scope\\":' in dynamic_main
    assert '\\"apf_runtime_fields\\":' in dynamic_main
    assert '\\"apf_python_only_fields\\":' in dynamic_main
    assert '\\"apf_cpp_pending_fields\\":' in dynamic_main
    assert '\\"task_waypoints\\":[' in dynamic_main
    assert '\\"replanned_waypoints\\":[' in dynamic_main
    assert '\\"executed_path\\":[' in dynamic_main
    assert '\\"replan_events\\":[' in dynamic_main
    assert '\\"sensor_logs\\":[' in dynamic_main
    assert '\\"collision_log\\":[' in dynamic_main
    assert '\\"fault_log\\":[' in dynamic_main
    assert "dump_replan_event" in dynamic_main
    assert "dump_collision_event" in dynamic_main
    assert "dump_fault_event" in dynamic_main


def test_cpp_apf_profile_config_surface_and_subset_runtime_are_explicit():
    obstacle_header = read("cpp/include/obstacle_scenario.hpp")
    obstacle_source = read("cpp/src/obstacle_scenario.cpp")
    dynamic_main = read("cpp/src/dynamic_main.cpp")
    dynamic_header = read("cpp/include/dynamic_scenario.hpp")
    apf_header = read("cpp/include/artificial_potential_field.hpp")
    apf_source = read("cpp/src/artificial_potential_field.cpp")
    formation_apf_header = read("cpp/include/formation_apf.hpp")
    formation_apf_source = read("cpp/src/formation_apf.cpp")
    formation_probe = read("cpp/src/apf_formation_probe.cpp")
    cmake = read("cpp/CMakeLists.txt")

    assert 'std::string apf_paper1_profile = "off"' in obstacle_header
    assert "double apf_comm_range = 10.0;" in obstacle_header
    assert "double apf_centroid_alpha = 0.4;" in obstacle_header
    assert "double apf_centroid_beta = 0.6;" in obstacle_header
    assert "bool apf_dev_override = false;" in obstacle_header
    assert "bool apf_adaptive_n_decay = false;" in obstacle_header
    assert "bool apf_formation_centroid = false;" in obstacle_header
    assert "bool apf_comm_constraint = false;" in obstacle_header
    assert "bool apf_rotational_escape = false;" in obstacle_header
    assert "ImprovedArtificialPotentialField build_apf() const;" in obstacle_header
    assert "std::unique_ptr<FormationAPF> build_formation_apf() const;" in obstacle_header
    assert "double k_comm = 0.0, double comm_range = 10.0," in apf_header
    assert "bool adaptive_n_decay = false" in apf_header
    assert "double k_rep, r_rep, k_inter, s_inter, mu_escape, max_acc, k_comm, comm_range;" in apf_header
    assert "bool adaptive_n_decay;" in apf_header
    assert "Vec3 communication_constraint_force" in apf_header
    assert "double adaptive_decay_exponent" in apf_header
    assert "double estimate_local_obstacle_density" in apf_header
    assert "mutable std::unordered_map<int, DensityCacheEntry> density_cache_" in apf_header
    assert "class FormationAPF" in formation_apf_header
    assert "compute_formation_avoidance(" in formation_apf_header
    assert "centroid_repulsion(" in formation_apf_header
    assert "relative_formation_forces(" in formation_apf_header
    assert "apf_(build_apf())" in obstacle_source
    assert "formation_apf_(build_formation_apf())" in obstacle_source
    assert 'if (config_.apf_paper1_profile == "conservative")' in obstacle_source
    assert 'else if (config_.apf_paper1_profile == "aggressive")' in obstacle_source
    assert "k_comm = 0.15;" in obstacle_source
    assert "k_comm = 0.30;" in obstacle_source
    assert "adaptive_n_decay = true;" in obstacle_source
    assert "adaptive_n_decay = config_.apf_adaptive_n_decay;" in obstacle_source
    assert "k_comm = config_.apf_comm_constraint ? 0.3 : 0.0;" in obstacle_source
    assert "config_.apf_dev_override && config_.apf_rotational_escape" in obstacle_source
    assert "double k_comm_, double comm_range_, bool adaptive_n_decay_)" in apf_source
    assert "max_acc(max_acc_), k_comm(k_comm_)," in apf_source
    assert "comm_range(std::max(comm_range_, 1e-6))" in apf_source
    assert "adaptive_n_decay(adaptive_n_decay_)" in apf_source
    assert "f_comm = communication_constraint_force(position, other_positions);" in apf_source
    assert "std::tanh((dist / comm_range - 0.8) / 0.05)" in apf_source
    assert "double limit = k_comm * static_cast<double>(others.size());" in apf_source
    assert "density_cache_.clear();" in apf_source
    assert "double decay_n = adaptive_n_decay ? adaptive_decay_exponent(pos, obs) : static_cast<double>(n_decay);" in apf_source
    assert "double ImprovedArtificialPotentialField::adaptive_decay_exponent(" in apf_source
    assert "double ImprovedArtificialPotentialField::estimate_local_obstacle_density(" in apf_source
    assert "if (norm(pos - cache.last_pos) < r_rep / 4.0 && cache.step < 10)" in apf_source
    assert "return 1.0 + 3.0 * std::min(local_density, 1.0);" in apf_source
    assert "std::pair<Vec3, std::vector<Vec3>> FormationAPF::compute_formation_avoidance(" in formation_apf_source
    assert "Vec3 FormationAPF::centroid_repulsion(" in formation_apf_source
    assert "std::vector<Vec3> FormationAPF::relative_formation_forces(" in formation_apf_source
    assert "src/formation_apf.cpp" in cmake
    assert "src/formation_safety.cpp" in cmake
    assert "sim_formation_safety_probe" in cmake
    assert "sim_apf_formation_probe" in cmake
    assert 'bool enable_centroid = argc >= 2 && std::string(argv[1]) == "on";' in formation_probe
    assert "cfg.apf_formation_centroid = enable_centroid;" in formation_probe
    assert 'std::cout << "centroid=" << (enable_centroid ? "on" : "off")' in formation_probe
    assert 'cfg.apf_paper1_profile = string_or(source.at("apf_paper1_profile"), cfg.apf_paper1_profile);' in dynamic_main
    assert 'cfg.apf_comm_range = number_or(source.at("apf_comm_range"), cfg.apf_comm_range);' in dynamic_main
    assert 'cfg.apf_centroid_alpha = number_or(source.at("apf_centroid_alpha"), cfg.apf_centroid_alpha);' in dynamic_main
    assert 'cfg.apf_centroid_beta = number_or(source.at("apf_centroid_beta"), cfg.apf_centroid_beta);' in dynamic_main
    assert 'cfg.apf_dev_override = bool_or(source.at("apf_dev_override"), cfg.apf_dev_override);' in dynamic_main
    assert 'cfg.apf_adaptive_n_decay = bool_or(source.at("apf_adaptive_n_decay"), cfg.apf_adaptive_n_decay);' in dynamic_main
    assert 'cfg.apf_formation_centroid = bool_or(source.at("apf_formation_centroid"), cfg.apf_formation_centroid);' in dynamic_main
    assert 'cfg.apf_comm_constraint = bool_or(source.at("apf_comm_constraint"), cfg.apf_comm_constraint);' in dynamic_main
    assert 'cfg.apf_rotational_escape = bool_or(source.at("apf_rotational_escape"), cfg.apf_rotational_escape);' in dynamic_main
    assert 'cfg.formation_safety_enabled = bool_or(' in dynamic_main
    assert 'cfg.formation_min_inter_drone_distance = number_or(' in dynamic_main
    assert 'cfg.formation_downwash_radius = number_or(' in dynamic_main
    assert 'cfg.formation_downwash_height = number_or(' in dynamic_main
    assert 'cfg.formation_recovery_hold_steps = static_cast<int>(number_or(' in dynamic_main
    assert 'cfg.formation_recovery_clearance_margin = number_or(' in dynamic_main
    assert '"apf_paper1_profile"' in dynamic_header
    assert '"apf_rotational_escape"' in dynamic_header
    assert '"apf_comm_range"' in dynamic_header
    assert '"apf_comm_constraint"' in dynamic_header
    assert '"apf_adaptive_n_decay"' in dynamic_header
    assert '"apf_formation_centroid"' in dynamic_header
    assert '"apf_centroid_alpha"' in dynamic_header
    assert '"apf_centroid_beta"' in dynamic_header
    assert '"k_comm"' in dynamic_header
    assert '"comm_range"' in dynamic_header
    assert '"adaptive_n_decay"' in dynamic_header
    assert '"formation_apf_runtime"' in dynamic_header
    assert "leader_other_positions = follower_positions_now;" in obstacle_source
    assert "other_positions.push_back(ls.position);" in obstacle_source
    assert "Vec3 rep_acc = obstacle_repulsion_acc(ls0.position, target, leader_other_positions);" in obstacle_source
    assert "Vec3 rep_acc_f = obstacle_repulsion_acc(follower_pos, target_pos, other_positions);" in obstacle_source
    assert "if (formation_apf_ && !follower_positions_now.empty())" in obstacle_source
    assert "formation_apf_->compute_formation_avoidance(" in obstacle_source
    assert "rep_acc += formation_leader_acc;" in obstacle_source
    assert "rep_acc_f += formation_follower_accs" in obstacle_source


def test_web_dynamic_replay_labels_leader_centric_scope_and_proxy_followers():
    web = read("web/dynamic_replay.html")

    assert "leader_centric" in web
    assert "Follower示意" in web
    assert "Follower示意当前" in web
    assert "Follower proxies" in web
    assert "共享 Leader 轨迹逐帧评估" in web
    assert "证据摘要：" in web
    assert "replan_events" in web
    assert "sensor_logs" in web


def test_python_obstacle_scenario_wires_formation_apf_switch():
    scenario = read("simulations/obstacle_scenario.py")

    assert "FormationAPF" in scenario
    assert "self.formation_apf = self._build_formation_apf()" in scenario
    assert "def _build_formation_apf" in scenario
    assert "compute_formation_avoidance(" in scenario
    assert "formation_leader_acc" in scenario
    assert "formation_follower_accs" in scenario
