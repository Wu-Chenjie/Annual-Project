from __future__ import annotations

from experiments.metrics_extractor import extract_metrics


def test_extract_metrics_from_standard_sim_result():
    payload = {
        "preset": "demo",
        "runtime_engine": "python",
        "runtime_s": 1.25,
        "completed_waypoint_count": 2,
        "summary": {
            "mean_error_overall": 0.1,
            "max_error_overall": 0.2,
            "final_error_overall": 0.05,
            "collision_count": 0,
            "replan_count": 3,
            "fault_count": 1,
        },
        "planned_path": [[0, 0, 0], [3, 4, 0]],
        "executed_path": [[0, 0, 0], [0, 0, 1], [0, 0, 3]],
        "planned_trajectory": {
            "path_length": 5.5,
            "max_speed": 1.2,
            "max_acceleration": 0.8,
            "mean_jerk": 0.4,
            "max_jerk": 0.9,
            "jerk_squared_integral": 1.7,
            "snap_squared_integral": 2.3,
        },
        "safety_metrics": {
            "min_inter_drone_distance": 0.7,
            "downwash_hits": 2,
            "formation_clearance": {
                "required_clearance": 0.45,
                "min_leader_margin": 0.3,
                "min_follower_margin": -0.05,
                "min_formation_margin": -0.05,
                "violation_count": 4,
                "sample_count": 20,
            },
            "formation_clearance_posthoc": {
                "required_clearance": 0.9,
                "min_formation_margin": -0.2,
                "violation_count": 7,
                "follower_violation_count": 6,
            },
        },
        "formation_adaptation_events": [
            {"t": 0.0, "from": "diamond", "to": "line", "reason": "clearance_violation"},
            {"t": 2.5, "from": "line", "to": "diamond", "reason": "recover_default"},
        ],
    }

    metrics = extract_metrics(payload)

    assert metrics["preset"] == "demo"
    assert metrics["planned_path_length"] == 5.0
    assert metrics["executed_path_length"] == 3.0
    assert metrics["trajectory_max_speed"] == 1.2
    assert metrics["trajectory_max_jerk"] == 0.9
    assert metrics["trajectory_jerk_squared_integral"] == 1.7
    assert metrics["trajectory_snap_squared_integral"] == 2.3
    assert metrics["min_inter_drone_distance"] == 0.7
    assert metrics["downwash_hits"] == 2
    assert metrics["formation_clearance_required"] == 0.45
    assert metrics["formation_clearance_min_margin"] == -0.05
    assert metrics["formation_clearance_violation_count"] == 4
    assert metrics["formation_clearance_sample_count"] == 20
    assert metrics["formation_clearance_posthoc_min_margin"] == -0.2
    assert metrics["formation_clearance_posthoc_violation_count"] == 7
    assert metrics["formation_clearance_posthoc_follower_violation_count"] == 6
    assert metrics["formation_adaptation_count"] == 2
    assert metrics["formation_adaptation_last"] == "diamond"


def test_extract_metrics_from_raw_simulation_result():
    payload = {
        "metrics": {
            "mean": [0.1, 0.2],
            "max": [0.3, 0.4],
            "final": [0.05, 0.07],
        },
        "completed_waypoint_count": 1,
        "waypoints": [[0, 0, 0], [0, 3, 4]],
        "leader": [[0, 0, 0], [1, 0, 0], [1, 2, 0]],
    }

    metrics = extract_metrics(payload)

    assert metrics["mean_error_overall"] == 0.15000000000000002
    assert metrics["max_error_overall"] == 0.4
    assert metrics["final_error_overall"] == 0.060000000000000005
    assert metrics["planned_path_length"] == 5.0
    assert metrics["executed_path_length"] == 3.0


def test_extract_metrics_derives_trajectory_fields_without_planned_trajectory():
    payload = {
        "planned_path": [[0, 0, 0], [0, 7.5, 0]],
        "executed_path": [[0, 0, 0], [2, 0, 0], [4, 3, 0]],
        "time": [0.0, 1.0, 2.0],
    }

    metrics = extract_metrics(payload)

    assert metrics["trajectory_path_length"] == 7.5
    assert metrics["trajectory_max_speed"] == 3.605551275463989
    assert metrics["trajectory_max_acceleration"] is not None
    assert metrics["trajectory_jerk_squared_integral"] is not None
    assert metrics["trajectory_snap_squared_integral"] is not None
