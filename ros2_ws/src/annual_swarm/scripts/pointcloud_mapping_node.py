#!/usr/bin/env python3
"""Integrate actual Gazebo GPU-lidar PointCloud2 into a private 3D occupancy map."""
import json
import uuid
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import planning_runtime
from core.exploration.voxel_mapping import VoxelMap, lidar_return_mask


class PointCloudMapper(Node):
    def __init__(self):
        super().__init__('pointcloud_mapper')
        self.session = uuid.uuid4().hex
        self.id = int(self.declare_parameter('drone_id', 0).value)
        bounds = json.loads(self.declare_parameter('bounds', '[]').value)
        self.map = VoxelMap(bounds); self.belief = np.zeros(self.map.shape, np.float32); self.ego_cells = set(); self.poses = deque(maxlen=300); self.sequence = 0; self.full_requested = False; self.last = -10.
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, f'/drone_{self.id}/observation', q)
        self.create_subscription(String, f'/drone_{self.id}/map_bootstrap', lambda m: setattr(self, 'full_requested', True), q)
        self.create_subscription(Odometry, f'/drone_{self.id}/estimated_odometry', self.odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, f'/drone_{self.id}/lidar/points', self.cloud, qos_profile_sensor_data)

    def odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        position = np.array([p.x, p.y, p.z]); rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self.poses.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9, position, rotation))
        # The physical collision hull has occupied this swept volume without
        # touching an external obstacle. Clear only voxel centers inside that
        # hull, not a fabricated sensor-radius free bubble.
        index = self.map.indices(position)
        cells = index+np.indices((5, 5, 5)).reshape(3, -1).T-2
        cells = cells[np.all((cells >= 0) & (cells < self.map.shape), axis=1)]
        body = (self.map.points(cells)-position)@rotation
        inside = np.all(np.abs(body) <= [.32, .32, .08], axis=1)
        self.ego_cells.update(map(tuple, cells[inside]))

    def cloud(self, m):
        stamp = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if not self.poses or stamp-self.last < .35:
            return
        t, position, rotation = min(self.poses, key=lambda p: abs(p[0]-stamp))
        if abs(t-stamp) > .15:
            return
        fields = {f.name: f for f in m.fields}
        if any(k not in fields or fields[k].datatype != 7 for k in ('x', 'y', 'z')):
            self.get_logger().error('Expected FLOAT32 XYZ point fields'); return
        dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=[('>' if m.is_bigendian else '<')+'f4']*3,
                              offsets=[fields[k].offset for k in ('x', 'y', 'z')], itemsize=m.point_step))
        rows = [np.frombuffer(m.data, dtype=dtype, count=m.width, offset=i*m.row_step) for i in range(m.height)]
        raw = np.concatenate(rows); points = np.column_stack([raw[k] for k in ('x', 'y', 'z')]).astype(float)
        n = len(points); finite = np.isfinite(points).all(axis=1)
        # Gazebo publishes an organized cloud. Missing returns retain ray direction
        # through this known sensor calibration, not through hidden world geometry.
        horizontal = np.linspace(-np.pi/3, np.pi/3, 181)
        vertical = np.linspace(-np.pi/3, np.pi/3, 31)
        az, el = np.meshgrid(horizontal, vertical)
        directions = np.column_stack([np.cos(el.ravel())*np.cos(az.ravel()),
                                      np.cos(el.ravel())*np.sin(az.ravel()), np.sin(el.ravel())])
        if n != len(directions):
            self.get_logger().error(f'Unexpected lidar calibration/cloud size: {n}'); return
        ranges = np.linalg.norm(np.nan_to_num(points, nan=0., posinf=0., neginf=0.), axis=1)
        hits = lidar_return_mask(points)
        points[~finite] = directions[~finite]*4.5
        # Put a noisy surface hit slightly inside the surface instead of letting
        # it alternate between the two adjacent voxels at a grid-aligned wall.
        points[hits] += .015*points[hits]/np.maximum(ranges[hits, None], 1e-6)
        # Self returns are integrated only up to their surface, never through it.
        origin = position+rotation@np.array([0., 0., .4])
        world = points@rotation.T+origin
        cells, values = self.map.integrate(origin, world, hits)
        key = tuple(cells.T)
        self.belief[key] = np.clip(self.belief[key]+np.where(values == 1, 1.2, -.45), -3., 3.)
        self.belief[key] = np.where(values == 1, np.maximum(self.belief[key], 1.2), self.belief[key])
        values = (self.belief[key] > .5).astype(np.int8)
        self.map.update(cells, values)
        if self.ego_cells:
            ego = np.array(sorted(self.ego_cells)); self.belief[tuple(ego.T)] = -3.; self.map.update(ego, np.zeros(len(ego), np.int8))
            merged = {tuple(c): int(v) for c, v in zip(cells, values)}
            merged.update({tuple(c): 0 for c in ego})
            cells = np.array(list(merged)); values = np.array(list(merged.values()), np.int8)
        if self.full_requested:
            cells = np.argwhere(self.map.state != -1); values = self.map.state[tuple(cells.T)]
            self.full_requested = False
        self.ego_cells.clear()
        self.sequence += 1; self.last = stamp
        packet = dict(schema='annual.observation/2', dimensions=3, resolution=self.map.resolution,
                      source=self.id, sensor_session=self.session, sequence=self.sequence, time=stamp, indices=cells.tolist(), values=values.tolist(),
                      sensor='gazebo_gpu_lidar', pose_source='estimated_odometry', point_count=n)
        self.pub.publish(String(data=json.dumps(packet, separators=(',', ':'))))


def main():
    rclpy.init(); node = PointCloudMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__ == '__main__':
    main()
