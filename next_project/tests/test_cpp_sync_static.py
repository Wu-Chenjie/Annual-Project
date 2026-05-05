from pathlib import Path


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
