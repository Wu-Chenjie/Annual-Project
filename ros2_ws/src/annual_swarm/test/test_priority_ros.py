"""ROS message construction, configuration and service-feedback callback checks."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'next_project'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_visualization_node import Visualization
from exploration_palette import PRIORITY_HIGH
from decentralized_agent_node import ExplorationAgent
from core.exploration.priority import ExplorationPriority
from core.exploration.voxel_mapping import VoxelMap


def test_priority_markers_contain_unknown_voxels_size_and_score():
    m = VoxelMap([[0, 0, 0], [8, 8, 4]])
    m.state[:] = 0; m.state[20:, :, :] = -1
    priority = ExplorationPriority(); priority.update_map(m)
    priority.scores = {7: dict(region=7, components=[1], score=.7, deferred=False, wait_s=90.)}
    layer = priority.snapshot(m, 10., {7: 0}, 0)
    rclpy.init()
    node = Visualization()
    try:
        packets = []; node.pub = SimpleNamespace(publish=packets.append)
        node.graphs[0] = dict(exploration_priority=layer)
        node.tick()
        markers = packets[-1].markers
        heat = next(x for x in markers if x.ns == 'unknown_region_priority')
        assert len(heat.points) == np.count_nonzero(np.array(layer['slice_labels']) == 1)
        assert (heat.color.r, heat.color.g, heat.color.b) == pytest.approx(PRIORITY_HIGH)
        assert heat.color.a > 0
        text = next(x.text for x in markers if x.ns == 'unknown_region_labels')
        assert 'm3' in text and '0.700' in text
        # A peer observation changes the display category, never the private map.
        observed = m.state.copy(); observed[20:22, :, :] = 0
        node.map = dict(state=observed.ravel().tolist(), shape=list(m.shape),
                        resolution=m.resolution, origin=m.origin.tolist())
        node.tick(); markers = packets[-1].markers
        heat = next(x for x in markers if x.ns == 'unknown_region_priority')
        peer = next(x for x in markers if x.ns == 'peer_observed_local_unknown')
        assert len(peer.points) == 2*m.shape[1]
        assert len(heat.points)+len(peer.points) == np.count_nonzero(np.array(layer['slice_labels']) == 1)
        assert np.all(m.state[20:, :, :] == -1)
        # A completed component disappears on the next published DELETEALL frame.
        node.graphs[0] = {}
        node.tick()
        assert not any(x.ns == 'unknown_region_priority' for x in packets[-1].markers)
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_agent_feedback_persists_streak_and_success_across_restart(tmp_path):
    config = ROOT/'ros2_ws/src/annual_swarm/config/exploration_priority.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config),
                    '-p', 'bounds:="[[0,0,0],[8,8,4]]"',
                    '-p', 'fleet_starts:="[[2,2,1.5]]"', '-p', f'output_dir:={tmp_path}'])
    node = ExplorationAgent()
    try:
        assert node.fusion.priority.config.retry_base_s == 15.
        node.now = lambda: 100.
        node.position = np.array([2.25, 2.25, 1.65])
        node.bootstrapped = node.rejoin_ready = True
        node.replica.map.state[:] = 0
        node.replica.map.state.ravel()[100:200] = -1
        node.replica.map.rebuild()
        def observe(gain):
            node.intent = dict(committed=True, path=[node.position.tolist()], yaw=0., token='test')
            node.selection = dict(transit=False); node.active = 7
            node.view_start_cells = np.arange(100, 200)
            node.view_start_known = int(np.count_nonzero(node.replica.map.state != -1))
            node.replica.map.state.ravel()[100:100+gain] = 0
            node.execution = dict(epoch=node.epoch, arrived=True, ready=True)
            node.heartbeat()
        observe(0)
        assert node.service_feedback[7]['low_yield_streak'] == 1
        assert node.cooldown[7] == 115.
        node.now = lambda: 120.
        observe(40)
        assert node.service_feedback[7]['low_yield_streak'] == 0
        assert node.cooldown[7] == 120.
    finally:
        node.worker.shutdown(); node.log.close(); node.paths.close(); node.destroy_node()
    restored = ExplorationAgent()
    try:
        assert restored.service_feedback[7]['low_yield_streak'] == 0
        assert restored.service_feedback[7]['observed_new_cells'] == 40
    finally:
        restored.worker.shutdown(); restored.log.close(); restored.paths.close(); restored.destroy_node()
        rclpy.shutdown()
