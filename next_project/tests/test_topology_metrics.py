import math

import numpy as np

from core.topology import TopologyGraph
from core.topology_metrics import TopologyMetricAccumulator


def test_topology_metric_accumulator_tracks_connectivity_and_energy():
    offsets = [
        np.array([-2.0, -2.0, 0.0], dtype=float),
        np.array([-2.0, 2.0, 0.0], dtype=float),
        np.array([-4.0, 0.0, 0.0], dtype=float),
    ]
    expected_lambda2 = TopologyGraph(offsets).algebraic_connectivity

    acc = TopologyMetricAccumulator(dt=0.5)
    acc.add_sample(
        offsets,
        leader_control=np.array([1.0, 2.0, 0.0, 0.0], dtype=float),
        follower_controls=[
            np.array([0.5, 0.0, 0.0, 0.0], dtype=float),
            np.array([0.0, 0.5, 0.0, 0.0], dtype=float),
        ],
    )
    acc.add_sample(
        offsets,
        leader_control=np.array([0.0, 1.0, 0.0, 0.0], dtype=float),
        follower_controls=[np.array([0.0, 0.0, 1.0, 0.0], dtype=float)],
    )

    payload = acc.to_dict()

    assert payload["available"] is True
    assert payload["sample_count"] == 2
    assert math.isclose(payload["mean_algebraic_connectivity"], expected_lambda2, rel_tol=1e-9)
    assert math.isclose(payload["min_algebraic_connectivity"], expected_lambda2, rel_tol=1e-9)
    assert math.isclose(payload["final_algebraic_connectivity"], expected_lambda2, rel_tol=1e-9)
    assert math.isclose(payload["leader_control_energy_proxy"], 3.0, rel_tol=1e-9)
    assert math.isclose(payload["follower_control_energy_proxy"], 0.75, rel_tol=1e-9)


def test_topology_metric_accumulator_counts_fault_and_reconfigure_events():
    acc = TopologyMetricAccumulator(dt=0.1)
    fault_log = [
        {"type": "inject", "drone": "follower_0"},
        {"type": "detect", "drone": "follower_1"},
        {"type": "reconfigure", "topology": "line"},
        "reconfigure:diamond:t_2.0",
    ]

    payload = acc.to_dict(fault_log=fault_log)

    assert payload["fault_event_count"] == 2
    assert payload["reconfiguration_event_count"] == 2
