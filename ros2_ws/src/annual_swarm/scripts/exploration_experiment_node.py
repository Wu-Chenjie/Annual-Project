#!/usr/bin/env python3
"""Read-only truth evaluator and reproducible fault/obstacle interventions.

Truth geometry never enters agent parameters/topics. Control only pauses one UAV
or freezes a completed/failed experiment; it assigns no task, owner, route or yaw.
"""
import csv
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts
import planning_runtime
from core.exploration.voxel_mapping import VoxelTruth, VoxelMap


class Experiment(Node):
    def __init__(self):
        super().__init__('exploration_experiment')
        file = self.declare_parameter('map_file', '').value
        self.fleet_size = len(json.loads(Path(file).read_text())['search_starts'])
        self.world = VoxelTruth(file); self.observed = VoxelMap(self.world.bounds)
        self.output = Path(self.declare_parameter('output_dir', '/tmp/decentralized').value); self.output.mkdir(parents=True, exist_ok=True)
        (self.output/'map.json').write_text(Path(file).read_text())
        self.threshold = self.declare_parameter('coverage_target', .95).value
        self.pause_after = self.declare_parameter('pause_after', 60.).value
        self.pause_duration = self.declare_parameter('pause_duration', 25.).value
        self.dynamic_enabled = self.declare_parameter('dynamic_enabled', False).value
        self.dynamic_trial = None; self.dynamic_finished = False
        self.obstacle_pub = self.create_publisher(PoseStamped, '/swarm/move_obstacle', 1)
        self.network_after = self.declare_parameter('network_after', 100.).value
        self.network_duration = self.declare_parameter('network_duration', 18.).value
        self.isolated = []; self.network_started = None; self.network_resumed = False
        self.restart_after = self.declare_parameter('restart_after', 210.).value
        self.restart_trial = None; self.restart_recovered = False
        self.session_counters = {}
        self.positions = {}; self.yaws = {}; self.airborne = set(); self.states = {}; self.executions = {}
        self.tracking = {i: [] for i in range(self.fleet_size)}; self.yaw_errors = {i: [] for i in range(self.fleet_size)}
        self.references = (self.output/'tracking.csv').open('w'); self.reference_writer = csv.writer(self.references)
        self.reference_writer.writerow(['time', 'drone', 'error_m', 'yaw_error_rad', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'method'])
        self.contacts = 0; self.minimum = None; self.started = None; self.finished = None; self.status = 'WAITING'
        self.failure_reason = None
        self.distances = {i: 0. for i in range(self.fleet_size)}; self.samples = {i: 0 for i in range(self.fleet_size)}; self.sequence = 0
        self.t90 = None; self.t95 = None; self.coverage = []; self.paused = []; self.pause_started = None; self.resumed = False
        self.coverage_details = {}
        self.csv = (self.output/'trajectory.csv').open('w'); self.writer = csv.writer(self.csv)
        self.writer.writerow(['time', 'drone', 'x', 'y', 'z', 'yaw'])
        self.telemetry = (self.output/'peer_states.jsonl').open('w')
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.observation_frames = {i: 0 for i in range(self.fleet_size)}
        for i in range(self.fleet_size):
            self.create_subscription(String, f'/drone_{i}/observation', lambda m, i=i: self.observation(i, m), q)
        self.control_pub = self.create_publisher(String, '/experiment/control', q)
        self.diag = self.create_publisher(String, '/experiment/diagnostics', q)
        self.map_pub = self.create_publisher(String, '/experiment/observed_map', q)
        for i in range(self.fleet_size):
            self.create_subscription(Odometry, f'/drone_{i}/odometry', lambda m, i=i: self.odom(i, m), qos_profile_sensor_data)
            self.create_subscription(Contacts, f'/drone_{i}/contacts', lambda m, i=i: self.contact(i, m), qos_profile_sensor_data)
            self.create_subscription(String, f'/drone_{i}/peer_state', lambda m, i=i: self.peer(i, m), q)
            self.create_subscription(String, f'/drone_{i}/view_execution', lambda m, i=i: self.execution(i, m), q)
        self.create_subscription(String, '/swarm/dynamic_obstacles', self.dynamic, q)
        self.create_timer(.5, self.tick)

    def observation(self, i, m):
        p = json.loads(m.data)
        cells = np.asarray(p['indices'], int).reshape(-1, 3)
        if len(cells):
            self.observed.update(cells, np.asarray(p['values'], np.int8))
            self.observation_frames[i] += 1

    def execution(self, i, m):
        p = json.loads(m.data); self.executions[i] = p
        if self.started is not None and self.finished is None and p.get('reason') == 'tracking_view':
            if i not in self.positions or 'reference' not in p:
                return
            error = float(np.linalg.norm(self.positions[i]-np.asarray(p['reference'])))
            yaw = abs(float(p.get('yaw_error', 0)))
            self.tracking[i].append(error); self.yaw_errors[i].append(yaw)
            self.reference_writer.writerow([self.get_clock().now().nanoseconds*1e-9, i, error, yaw,
                *p.get('reference_velocity', [0, 0, 0]), *p.get('reference_acceleration', [0, 0, 0]), p.get('trajectory_method')])

    def dynamic(self, m):
        data = json.loads(m.data)
        if data.get('frame_id') == 'world':
            self.world.dynamic_snapshot(int(data['version']), data['obstacles'])

    def peer(self, i, m):
        self.states[i] = json.loads(m.data)
        state = self.states[i]
        key = (i, state.get('session', 'legacy'))
        previous = self.session_counters.setdefault(key, {})
        for field in ('commits', 'observed_views', 'peer_payload_bytes', 'graph_payload_bytes', 'pair_commits'):
            previous[field] = max(previous.get(field, 0), state.get(field, 0))
        if self.restart_trial and i == self.restart_trial['drone']:
            if state.get('session') != self.restart_trial['session'] and state.get('rejoin_ready') and state.get('ready'):
                self.restart_recovered = True
                self.restart_trial['recovered_session'] = state['session']
        # Log public protocol state without duplicating every raw observation.
        p = dict(self.states[i]); p.pop('observation', None)
        self.telemetry.write(json.dumps(p)+'\n')

    def odom(self, i, m):
        p = m.pose.pose.position; value = np.array([p.x, p.y, p.z]); q = m.pose.pose.orientation
        self.yaws[i] = float(np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        t = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if i in self.positions:
            self.distances[i] += float(np.linalg.norm(value-self.positions[i]))
        self.positions[i] = value; self.samples[i] += 1
        if p.z > .5:
            self.airborne.add(i)
        for j, other in self.positions.items():
            if j != i:
                d = float(np.linalg.norm(value-other)); self.minimum = d if self.minimum is None else min(self.minimum, d)
        self.writer.writerow([t, i, *value, self.yaws[i]])

    def contact(self, i, m):
        for c in m.contacts:
            if 'ground' in c.collision1.name+c.collision2.name and i not in self.airborne:
                continue
            self.contacts += 1

    def obstacle_trial(self, t):
        if not self.dynamic_enabled or self.started is None or t-self.started < 150.:
            return
        if self.dynamic_trial is None:
            for i, state in sorted(self.states.items()):
                intent = state.get('intent') or {}
                if not intent.get('committed') or intent.get('retiring'):
                    continue
                path = np.asarray(intent['path']); position = self.positions[i]
                for point in path[len(path)//3::3]:
                    delta = point[:2]-position[:2]; distance = np.linalg.norm(delta)
                    if distance < 1.8 or distance > 4.2:
                        continue
                    heading = np.arctan2(delta[1], delta[0])
                    if abs(np.arctan2(np.sin(heading-self.yaws[i]), np.cos(heading-self.yaws[i]))) > .3:
                        continue
                    side = np.array([-delta[1], delta[0]])/distance*.8
                    centers = np.array([point[:2]+u*side for u in np.linspace(-1, 1, 11)])
                    if any(np.linalg.norm(c-p[:2]) < 1.6 for c in centers for p in self.positions.values()):
                        continue
                    xy = self.world.xyz[:, :, 5, :2]
                    occupied = self.world.static_occupied[:, :, 5]
                    if any(np.any(occupied & (np.linalg.norm(xy-c, axis=2) < .65)) for c in centers):
                        continue
                    self.dynamic_trial = dict(start=t, drone=i, center=point[:2].tolist(), side=side.tolist(), token=intent['token'])
                    break
                if self.dynamic_trial:
                    break
        if self.dynamic_trial and not self.dynamic_finished:
            elapsed = t-self.dynamic_trial['start']; center = np.array(self.dynamic_trial['center']); side = np.array(self.dynamic_trial['side'])
            if elapsed < 8.:
                point = center+(1-elapsed/8.)*side
            elif elapsed < 14.:
                point = center
            elif elapsed < 22.:
                point = center-(elapsed-14.)/8.*side
            else:
                point = np.array([-20., -20.]); self.dynamic_finished = True
            msg = PoseStamped(); msg.header.frame_id = 'world'; msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = map(float, point); msg.pose.position.z = 1.5; msg.pose.orientation.w = 1.
            self.obstacle_pub.publish(msg)

    def tick(self):
        t = self.get_clock().now().nanoseconds*1e-9; self.sequence += 1
        if len(self.positions) < self.fleet_size:
            return
        if self.started is None and len(self.executions) == self.fleet_size and all(p.get('ready') for p in self.executions.values()):
            self.started = t; self.status = 'SEARCHING'
        if self.started is not None and self.finished is None:
            elapsed = t-self.started
            if self.network_after > 0 and elapsed >= self.network_after and self.network_started is None:
                self.network_started = t; self.isolated = [2]
            if self.network_started is not None and t-self.network_started >= self.network_duration:
                self.isolated = []; self.network_resumed = True
            if self.restart_after > 0 and elapsed >= self.restart_after and self.restart_trial is None and self.states:
                i = min(self.states)
                self.restart_trial = dict(drone=i, session=self.states[i]['session'], start=t)
            if (self.pause_after > 0 and elapsed >= self.pause_after and self.pause_started is None
                    and (self.states.get(1, {}).get('intent') or {}).get('committed', False)):
                self.pause_started = t; self.paused = [1]
            if self.pause_started is not None and not self.resumed and t-self.pause_started >= self.pause_duration:
                self.paused = []; self.resumed = True
        if self.finished is None:
            self.coverage_details = self.world.coverage_metrics(self.observed)
            coverage = self.coverage_details['coverage']; self.coverage.append([t, coverage])
            if self.started is not None:
                if coverage >= .9 and self.t90 is None:
                    self.t90 = t-self.started
                if coverage >= .95 and self.t95 is None:
                    self.t95 = t-self.started
                if coverage >= self.threshold:
                    self.status = 'COMPLETE'; self.finished = t
            if self.contacts:
                self.status = 'FAILED'; self.finished = t; self.failure_reason = 'contact'
            if any(i in self.airborne and (np.any(p < self.world.bounds[0]-.5) or np.any(p > self.world.bounds[1]+.5))
                   for i, p in self.positions.items()):
                self.status = 'FAILED'; self.finished = t; self.failure_reason = 'flight_bounds'
            if self.finished is not None:
                np.savez_compressed(self.output/'observed_final.npz', state=self.observed.state,
                                    bounds=self.observed.bounds, resolution=self.observed.resolution)
        self.obstacle_trial(t)
        restart = [self.restart_trial['drone']] if self.restart_trial and not self.restart_recovered else []
        self.control_pub.publish(String(data=json.dumps(dict(paused=self.paused, isolated=self.isolated,
            restart=restart, restart_session=self.restart_trial['session'] if restart else None, done=self.finished is not None))))
        self.save(t)
        if self.sequence % 2 == 0:
            self.map_pub.publish(String(data=json.dumps(dict(shape=[int(v) for v in self.observed.shape], origin=self.observed.origin.tolist(),
                resolution=self.observed.resolution, state=self.observed.state.flatten().tolist()))))

    def save(self, t):
        def totals(field):
            return {i: sum(c.get(field, 0) for (drone, _), c in self.session_counters.items() if drone == i)
                    for i in range(self.fleet_size)}
        report = dict(fleet_size=self.fleet_size, agent_ages={i: t-p['time'] for i, p in self.states.items()}, status=self.status, failure_reason=self.failure_reason, simulation_time=t, start_time=self.started, finish_time=self.finished,
            architecture='fused adaptive Hgrid, MR-DTG deltas, two-level graph Voronoi, bilateral capacity routing, continuous quintic flight',
            coverage=self.coverage[-1][1] if self.coverage else 0., t90=self.t90, t95=self.t95, coverage_target=self.threshold,
            contacts_after_takeoff=self.contacts, min_separation_m=self.minimum, distances_m=self.distances,
            odometry_samples=self.samples, pause_resumed=self.resumed, paused=self.paused,
            graph={i: p.get('graph', {}) for i, p in self.states.items()},
            commits=totals('commits'), views=totals('observed_views'),
            peer_payload_bytes=totals('peer_payload_bytes'), graph_payload_bytes=totals('graph_payload_bytes'),
            fusion={i: p.get('fusion', {}) for i, p in self.states.items()},
            pair_commits=totals('pair_commits'),
            tracking={i: dict(samples=len(e), rms_m=float(np.sqrt(np.mean(np.square(e)))), p95_m=float(np.quantile(e, .95)),
                              max_m=max(e)) for i, e in self.tracking.items() if e},
            observation_frames=self.observation_frames, mapping_dimensions=3, isolated=self.isolated,
            network_started=self.network_started, network_resumed=self.network_resumed,
            dynamic_trial=self.dynamic_trial, dynamic_finished=self.dynamic_finished,
            restart_trial=self.restart_trial, restart_recovered=self.restart_recovered,
            tracking_reference='Gazebo truth position versus executed reference at telemetry receipt',
            assumptions='3D occupancy; native GPU lidar 120x120 degrees / 4.5m with range noise; IMU and noisy simulated localization EKF (not SLAM); planning-channel partition with independent local safety sensing; fixed fleet spatial reservations')
        report.update(self.coverage_details)
        tmp = self.output/'summary.tmp'; tmp.write_text(json.dumps(report, indent=2)); tmp.replace(self.output/'summary.json')
        if rclpy.ok():
            self.diag.publish(String(data=json.dumps(report)))
        (self.output/'coverage.json').write_text(json.dumps(self.coverage))
        self.csv.flush(); self.telemetry.flush(); self.references.flush()


def main():
    rclpy.init(); node = Experiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save(node.get_clock().now().nanoseconds*1e-9); node.csv.close(); node.telemetry.close(); node.references.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
