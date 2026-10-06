"""Offline common mapping replay from the recorded real sensor stream."""
import argparse, base64, gzip, json, math, sys, zlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, '/workspace/ros2_ws/install/annual_swarm/lib/annual_swarm')
import planning_runtime
from core.exploration.voxel_mapping import VoxelMap, VoxelTruth
from common_mapper import PointCloudMapper
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


def stamp(message, t):
    ns = round(t*1e9)
    message.header.stamp.sec = ns//10**9
    message.header.stamp.nanosec = ns%10**9


def replay(root):
    root = Path(root)
    result = json.loads((root/'result.json').read_text())
    world = json.loads((root.parent/'map.json').read_text())
    truth = VoxelTruth(root.parent/'map.json')
    observed = VoxelMap(world['bounds'])
    counts = [0, 0]
    rclpy.init(args=['--ros-args', '-p', 'bounds:='+json.dumps(json.dumps(world['bounds']))])
    nodes = [PointCloudMapper(), PointCloudMapper()]
    def receive(i, message):
        packet = json.loads(message.data)
        cells = np.asarray(packet['indices'], int).reshape(-1, 3)
        if len(cells):
            observed.update(cells, np.asarray(packet['values'], np.int8))
        counts[i] += 1
    try:
        for i, node in enumerate(nodes):
            node.id = i
            node.pub = SimpleNamespace(publish=lambda m, i=i: receive(i, m))
        with gzip.open(root/'sensor-stream.jsonl.gz', 'rt') as stream:
            for line in stream:
                packet = json.loads(line)
                if packet['type'] not in ('odom', 'cloud') or packet['t'] > result['t']:
                    continue
                i = packet['i']
                if packet['type'] == 'odom':
                    m = Odometry(); stamp(m, packet['t'])
                    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = packet['p']
                    q = m.pose.pose.orientation
                    q.x, q.y, q.z, q.w = packet['q']
                    nodes[i].odom(m)
                else:
                    m = PointCloud2(); stamp(m, packet['t'])
                    m.height, m.width = packet['height'], packet['width']
                    m.point_step, m.row_step = packet['point_step'], packet['row_step']
                    m.fields = [PointField(name=n, offset=o, datatype=d, count=c)
                                for n, o, d, c in packet['fields']]
                    data = base64.b64decode(packet['data'])
                    m.data = zlib.decompress(data) if packet.get('compressed') else data
                    nodes[i].cloud(m)
        metrics = truth.coverage_metrics(observed)
        output = dict(recomputed=metrics, logged_coverage=result['coverage'],
                      difference=metrics['coverage']-result['coverage'], integrated_frames=counts,
                      logged_observation_frames=[result['counts'].get(f'observation_{i}') for i in range(2)],
                      note='Replay of recorded clock-stamped estimated pose and raw native LiDAR; independent of planner map and logged coverage. Callback receipt ordering and transport pose downsampling may change boundary voxels.')
        np.savez_compressed(root/'replayed-final-map.npz', state=observed.state,
                            bounds=observed.bounds, resolution=observed.resolution)
        (root/'coverage-replay.json').write_text(json.dumps(output, indent=2)+'\n')
        return output
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('directory')
    args = parser.parse_args(); print(json.dumps(replay(args.directory), indent=2))
