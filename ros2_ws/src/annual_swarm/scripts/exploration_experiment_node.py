#!/usr/bin/env python3
"""Simulation-only finite-FOV sensor and read-only benchmark observer.

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
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts
import planning_runtime
from core.exploration.mapping import RaySensorWorld, ObservedMap


class Experiment(Node):
    def __init__(self):
        super().__init__('exploration_experiment')
        file = self.declare_parameter('map_file', '').value
        self.world = RaySensorWorld(file); self.observed = ObservedMap(self.world.bounds)
        self.output = Path(self.declare_parameter('output_dir', '/tmp/decentralized').value); self.output.mkdir(parents=True, exist_ok=True)
        (self.output/'map.json').write_text(Path(file).read_text())
        self.threshold = self.declare_parameter('coverage_target', .95).value
        self.pause_after = self.declare_parameter('pause_after', 60.).value
        self.pause_duration = self.declare_parameter('pause_duration', 25.).value
        self.positions = {}; self.yaws = {}; self.airborne = set(); self.states = {}; self.executions = {}
        self.contacts = 0; self.minimum = None; self.started = None; self.finished = None; self.status = 'WAITING'
        self.distances = {i: 0. for i in range(3)}; self.samples = {i: 0 for i in range(3)}; self.sequence = 0
        self.t90 = None; self.t95 = None; self.coverage = []; self.paused = []; self.pause_started = None; self.resumed = False
        self.csv = (self.output/'trajectory.csv').open('w'); self.writer = csv.writer(self.csv)
        self.writer.writerow(['time', 'drone', 'x', 'y', 'z', 'yaw'])
        self.telemetry = (self.output/'peer_states.jsonl').open('w')
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {i: self.create_publisher(String, f'/drone_{i}/observation', q) for i in range(3)}
        self.control_pub = self.create_publisher(String, '/experiment/control', q)
        self.diag = self.create_publisher(String, '/experiment/diagnostics', q)
        self.map_pub = self.create_publisher(String, '/experiment/observed_map', q)
        for i in range(3):
            self.create_subscription(Odometry, f'/drone_{i}/odometry', lambda m, i=i: self.odom(i, m), qos_profile_sensor_data)
            self.create_subscription(Contacts, f'/drone_{i}/contacts', lambda m, i=i: self.contact(i, m), qos_profile_sensor_data)
            self.create_subscription(String, f'/drone_{i}/peer_state', lambda m, i=i: self.peer(i, m), q)
            self.create_subscription(String, f'/drone_{i}/view_execution', lambda m, i=i: self.executions.update({i: json.loads(m.data)}), q)
        self.create_subscription(String, '/swarm/dynamic_obstacles', self.dynamic, q)
        self.create_timer(.5, self.tick)

    def dynamic(self, m):
        data = json.loads(m.data)
        if data.get('frame_id') == 'world':
            self.world.dynamic_snapshot(int(data['version']), data['obstacles'])

    def peer(self, i, m):
        self.states[i] = json.loads(m.data)
        # Log public protocol state without duplicating every raw observation.
        p = dict(self.states[i]); p.pop('observation', None)
        self.telemetry.write(json.dumps(p)+'\n')

    def odom(self, i, m):
        p = m.pose.pose.position; value = np.array([p.x, p.y, p.z]); q = m.pose.pose.orientation
        self.yaws[i] = float(np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        t = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if i in self.positions:
            self.distances[i] += float(np.linalg.norm(value[:2]-self.positions[i][:2]))
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

    def tick(self):
        t = self.get_clock().now().nanoseconds*1e-9; self.sequence += 1
        if len(self.positions) < 3:
            return
        if self.started is None and len(self.executions) == 3 and all(p.get('ready') for p in self.executions.values()):
            self.started = t; self.status = 'SEARCHING'
        if self.started is not None and self.finished is None:
            elapsed = t-self.started
            if (self.pause_after > 0 and elapsed >= self.pause_after and self.pause_started is None
                    and (self.states.get(1, {}).get('intent') or {}).get('committed', False)):
                self.pause_started = t; self.paused = [1]
            if self.pause_started is not None and not self.resumed and t-self.pause_started >= self.pause_duration:
                self.paused = []; self.resumed = True
        if self.finished is None:
            for i, p in self.positions.items():
                if p[2] < 1.2 or i in self.paused:
                    continue
                indices, values = self.world.observe(p, self.yaws[i])
                self.observed.update(indices, values)
                self.pubs[i].publish(String(data=json.dumps(dict(source=i, sequence=self.sequence, time=t,
                    indices=indices.tolist(), values=values.tolist(), position=p.tolist(), yaw=self.yaws[i], fov_rad=2*np.pi/3, radius_m=3.5))))
            coverage = self.world.coverage(self.observed); self.coverage.append([t, coverage])
            if self.started is not None:
                if coverage >= .9 and self.t90 is None:
                    self.t90 = t-self.started
                if coverage >= .95 and self.t95 is None:
                    self.t95 = t-self.started
                if coverage >= self.threshold:
                    self.status = 'COMPLETE'; self.finished = t
            if self.contacts:
                self.status = 'FAILED'; self.finished = t
        self.control_pub.publish(String(data=json.dumps(dict(paused=self.paused, done=self.finished is not None))))
        self.save(t)
        if self.sequence % 2 == 0:
            self.map_pub.publish(String(data=json.dumps(dict(shape=[int(v) for v in self.observed.shape], origin=self.observed.origin.tolist(),
                resolution=self.observed.resolution, state=self.observed.state.flatten().tolist()))))

    def save(self, t):
        report = dict(status=self.status, simulation_time=t, start_time=self.started, finish_time=self.finished,
            architecture='three independent agents, peer insertion auctions and unanimous path leases',
            coverage=self.coverage[-1][1] if self.coverage else 0., t90=self.t90, t95=self.t95, coverage_target=self.threshold,
            contacts_after_takeoff=self.contacts, min_separation_m=self.minimum, distances_m=self.distances,
            odometry_samples=self.samples, pause_resumed=self.resumed, paused=self.paused,
            graph={i: p.get('graph', {}) for i, p in self.states.items()},
            commits={i: p.get('commits', 0) for i, p in self.states.items()},
            views={i: p.get('observed_views', 0) for i, p in self.states.items()},
            peer_payload_bytes={i: p.get('payload_bytes', 0) for i, p in self.states.items()},
            assumptions='Planar 1.5 m altitude; ideal 120 degree 3.5 m occluded ray sensor; Gazebo truth odometry; fixed fleet quorum freezes on partition')
        tmp = self.output/'summary.tmp'; tmp.write_text(json.dumps(report, indent=2)); tmp.replace(self.output/'summary.json')
        self.diag.publish(String(data=json.dumps(report)))
        (self.output/'coverage.json').write_text(json.dumps(self.coverage))
        self.csv.flush(); self.telemetry.flush()


def main():
    rclpy.init(); node = Experiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save(node.get_clock().now().nanoseconds*1e-9); node.csv.close(); node.telemetry.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
