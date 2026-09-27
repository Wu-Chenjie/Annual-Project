#!/usr/bin/env python3
"""Place one real cylinder across an active curve with a distinct cached reserve.

Truth is used only by the external injector to avoid placing the cylinder in a
wall or on a UAV. Agents receive no truth, route choice or fabricated sensor
data. Actual Gazebo movement must be acknowledged; lidar triggers replanning.
"""
import json
from pathlib import Path
import time
import numpy as np


def select_point(active, progress, reserves, position, yaw, fleet_positions, occupied_xy, bounds):
    """A reproducible, visible, safe obstacle point and an unblocked reserve."""
    position = np.asarray(position, float)
    if active.duration-progress < 6. or not reserves:
        return None
    reserve_paths = [(name, curve.path(.2)) for name, curve in reserves]
    for future in np.arange(progress+5., min(active.duration, progress+20.), .5):
        point = active.sample(future)[0]; delta = point[:2]-position[:2]
        distance = float(np.linalg.norm(delta)); bearing = np.arctan2(delta[1], delta[0])
        if not 2.5 <= distance <= 3.8 or abs(np.arctan2(np.sin(bearing-yaw), np.cos(bearing-yaw))) > 1.0:
            continue
        if not .7 <= point[2] <= 3.1:
            continue
        for name, path in reserve_paths:
            nearest = path[np.argmin(np.linalg.norm(path[:, :2]-point[:2], axis=1)), :2]
            away = point[:2]-nearest
            offsets = [np.zeros(2)]
            if np.linalg.norm(away) > .1: offsets.append(.4*away/np.linalg.norm(away))
            for offset in offsets:
                center = point[:2]+offset; delta = center-position[:2]; distance = float(np.linalg.norm(delta))
                bearing = np.arctan2(delta[1], delta[0])
                if not 2.5 <= distance <= 3.8 or abs(np.arctan2(np.sin(bearing-yaw), np.cos(bearing-yaw))) > 1.0:
                    continue
                if np.any(center < bounds[0, :2]+.75) or np.any(center > bounds[1, :2]-.75):
                    continue
                if any(np.linalg.norm(center-np.asarray(p)[:2]) < 2.2 for p in fleet_positions):
                    continue
                if len(occupied_xy) and np.min(np.linalg.norm(occupied_xy-center, axis=1)) < .8:
                    continue
                clearance = float(np.min(np.linalg.norm(path[:, :2]-center, axis=1)))
                if clearance > 1.45:
                    return dict(center=center.tolist(), active_time=float(future),
                        visible_distance_m=distance, active_center_distance_m=float(np.linalg.norm(offset)),
                        reserve_candidate=name, reserve_center_clearance_m=clearance)
    return None


def last_pool(path, token):
    if not path.exists():
        return None
    with path.open('rb') as stream:
        size = stream.seek(0, 2); stream.seek(max(0, size-1048576)); lines = stream.read().splitlines()
    for line in reversed(lines):
        try:
            packet = json.loads(line)
            if packet.get('token') == token:
                return packet
        except (ValueError, UnicodeError):
            continue
    return None


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    import planning_runtime
    from core.planning.continuous_trajectory import ContinuousTrajectory
    from core.exploration.voxel_mapping import VoxelTruth

    class Probe(Node):
        def __init__(self):
            super().__init__('dynamic_backup_fault_probe')
            self.output = Path(self.declare_parameter('output_dir', '').value)
            world = VoxelTruth(self.declare_parameter('map_file', '').value)
            indices = (world.xyz[..., 2] >= .3) & (world.xyz[..., 2] <= 3.0)
            self.occupied_xy = np.unique(world.xyz[world.static_occupied & indices, :2], axis=0)
            self.bounds = world.bounds; self.positions = {}; self.yaws = {}; self.states = {}; self.executions = {}
            self.trial = None; self.ack = None; self.removed = False; self.next_command = 0.
            self.publisher = self.create_publisher(PoseStamped, '/swarm/move_obstacle', 1)
            q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for i in range(3):
                self.create_subscription(Odometry, f'/drone_{i}/odometry', lambda m, i=i: self.odom(i, m), qos_profile_sensor_data)
                self.create_subscription(String, f'/drone_{i}/peer_state', lambda m, i=i: self.states.update({i: json.loads(m.data)}), q)
                self.create_subscription(String, f'/drone_{i}/view_execution', lambda m, i=i: self.executions.update({i: json.loads(m.data)}), q)
            self.create_subscription(String, '/swarm/dynamic_obstacles', self.acknowledged, q)
            self.create_timer(.5, self.tick)

        def event(self, kind, **value):
            with (self.output/'dynamic-backup-fault.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(type=kind, time=self.get_clock().now().nanoseconds*1e-9,
                    wall_monotonic=time.monotonic(), **value))+'\n')

        def odom(self, i, message):
            p = message.pose.pose.position; q = message.pose.pose.orientation
            self.positions[i] = np.array([p.x, p.y, p.z])
            self.yaws[i] = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

        def acknowledged(self, message):
            packet = json.loads(message.data)
            if not self.trial or packet.get('source') != 'Gazebo SetEntityPose success':
                return
            center = packet['obstacles'][0]['center_xy']
            if np.linalg.norm(np.asarray(center)-self.trial['center']) < .05 and self.ack is None:
                self.ack = packet['stamp']; self.event('obstacle_acknowledged', snapshot=packet)
            elif np.linalg.norm(np.asarray(center)-[-20., -20.]) < .05 and self.removed:
                if not getattr(self, 'removal_logged', False):
                    self.event('obstacle_removal_acknowledged', snapshot=packet); self.removal_logged = True

        def command(self, center):
            message = PoseStamped(); message.header.frame_id = 'world'; message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x, message.pose.position.y = map(float, center)
            message.pose.position.z = 1.5; message.pose.orientation.w = 1.
            self.publisher.publish(message)

        def tick(self):
            now = self.get_clock().now().nanoseconds*1e-9
            if self.trial:
                if self.ack is None and now >= self.next_command:
                    self.command(self.trial['center']); self.next_command = now+1.
                if self.ack is not None and now-self.ack >= 24. and not self.removed:
                    self.command([-20., -20.]); self.removed = True; self.event('obstacle_removal_requested')
                return
            summary = self.output/'summary.json'
            if not summary.exists() or len(self.positions) != 3:
                return
            try:
                report = json.loads(summary.read_text())
            except (ValueError, OSError):
                return
            if report.get('start_time') is None or now-report['start_time'] < 120. or report['status'] != 'SEARCHING':
                return
            for i, state in sorted(self.states.items()):
                intent = state.get('intent') or {}; execution = self.executions.get(i, {})
                if (not intent.get('committed') or intent.get('retiring') or state.get('pending_intent')
                        or execution.get('token') != intent.get('token') or execution.get('reason') != 'tracking_view'
                        or now-state['time'] > 1. or intent.get('purpose') != 'explore'):
                    continue
                pool = last_pool(self.output/f'drone_{i}/candidates.jsonl', intent['token'])
                if not pool:
                    continue
                reserves = [(p['id'], ContinuousTrajectory.from_dict(p['trajectory'])) for p in pool['paths']
                    if p['id'] != pool['active_candidate'] and p.get('expires_at') is not None and p['expires_at'] > now]
                active_packet = next((p for p in pool['paths'] if p['id'] == pool['active_candidate']), None)
                if active_packet is None or pool['active_candidate'] != intent.get('candidate_id'):
                    continue
                active = ContinuousTrajectory.from_dict(active_packet['trajectory'])
                point = select_point(active, execution.get('trajectory_time', 0.), reserves,
                    self.positions[i], self.yaws[i], list(self.positions.values()), self.occupied_xy, self.bounds)
                if point:
                    self.trial = dict(**point, drone=i, token=intent['token'], candidate=intent['candidate_id'],
                        region=intent['region'], epoch=intent['epoch'], pool_time=pool['time'],
                        current_position=self.positions[i].tolist(), active_progress=execution.get('trajectory_time', 0.),
                        eligible_elapsed_s=now-report['start_time'])
                    self.event('obstacle_requested', trial=self.trial)
                    self.command(point['center']); self.next_command = now+1.; break

    rclpy.init(); node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
