#!/usr/bin/env python3
"""One aircraft's position/yaw executor with peer and odometry watchdogs."""
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import planning_runtime
from core.exploration.decentralized import execution_lease


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


class ViewExecutor(Node):
    def __init__(self):
        super().__init__('view_executor')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        self.ref = np.array(json.loads(self.declare_parameter('start', '[]').value), float)
        self.heading = 0.; self.actual_yaw = 0.; self.positions = {}; self.stamps = {}; self.peers = {}
        self.path = None; self.token = None; self.index = 0; self.epoch = 0; self.goal_yaw = 0.
        self.ready = False; self.arrived = True; self.scanned = 0.; self.hold_since = None
        self.done = False; self.paused = False; self.last = None; self.waits = 0; self.reason = 'takeoff'
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseStamped, f'/drone_{self.id}/target', 1)
        self.report_pub = self.create_publisher(String, f'/drone_{self.id}/view_execution', q)
        self.create_subscription(String, f'/drone_{self.id}/view_command', self.command, q)
        self.create_subscription(String, '/experiment/control', self.control, q)
        for i in range(3):
            self.create_subscription(Odometry, f'/drone_{i}/odometry', lambda m, i=i: self.odom(i, m), qos_profile_sensor_data)
            self.create_subscription(String, f'/drone_{i}/peer_state', lambda m, i=i: self.peers.update({i: json.loads(m.data)}), q)
        self.create_timer(.02, self.tick); self.create_timer(.2, self.report)

    def odom(self, i, m):
        p = m.pose.pose.position; self.positions[i] = np.array([p.x, p.y, p.z])
        self.stamps[i] = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if i == self.id:
            q = m.pose.pose.orientation
            self.actual_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def stop(self):
        self.path = None; self.arrived = True
        if self.id in self.positions:
            self.ref = self.positions[self.id].copy(); self.ref[2] = 1.5

    def command(self, m):
        p = json.loads(m.data)
        if p['epoch'] <= self.epoch:
            return
        if p.get('cancel'):
            self.epoch = p['epoch']; self.stop(); return
        path = np.asarray(p['path'], float)
        if path.ndim != 2 or path.shape[1] != 3 or not len(path) or not np.isfinite(path).all() or not np.isfinite(p['yaw']):
            return
        if self.id not in self.positions or np.linalg.norm(path[0]-self.positions[self.id]) > .6:
            return
        self.epoch = p['epoch']; self.path = path; self.index = 0; self.arrived = False
        self.goal_yaw = p['yaw']; self.token = p['token']; self.hold_since = None

    def control(self, m):
        p = json.loads(m.data); self.done = p.get('done', False); self.paused = self.id in p.get('paused', [])
        if self.done or self.paused:
            self.stop()

    def report(self):
        self.report_pub.publish(String(data=json.dumps(dict(drone=self.id, ready=self.ready, arrived=self.arrived,
            epoch=self.epoch, reason=self.reason, traffic_wait_samples=self.waits, yaw_error=wrap(self.goal_yaw-self.actual_yaw)))))

    def tick(self):
        now = self.get_clock().now(); t = now.nanoseconds*1e-9
        dt = 0 if self.last is None else min(max(t-self.last, 0), .04); self.last = t
        if self.id not in self.positions:
            return
        error = np.linalg.norm(self.positions[self.id]-self.ref)
        stale = any(t-stamp > .5 or stamp > t+.1 for stamp in self.stamps.values()) or len(self.positions) != 3
        if not self.ready and error < .15 and not stale:
            # Physical yaw scan, actual finite-FOV observations populate the map.
            if abs(wrap(self.heading-self.actual_yaw)) < .2:
                delta = min(.65*dt, 2*math.pi-self.scanned)
                self.heading = wrap(self.heading+delta); self.scanned += delta
            self.reason = 'initial_yaw_scan'
            if self.scanned >= 2*math.pi-1e-6 and abs(wrap(self.heading-self.actual_yaw)) < .08:
                self.ready = True
        elif self.ready and not self.done and not self.paused:
            peer_stale = len(self.peers) != 3 or any(t-p['time'] > 3. for p in self.peers.values())
            self.reason = 'odometry_timeout' if stale else 'peer_timeout' if peer_stale else 'idle'
            lease_valid = execution_lease(self.peers, self.id, self.token, t)
            if self.path is not None and not self.arrived and not lease_valid:
                self.reason = 'lease_confirmation_hold'
            if self.path is not None and not self.arrived and not stale and not peer_stale and lease_valid:
                # Bounded yaw rate is optimized as a cost by the observation planner.
                self.heading = wrap(self.heading+np.clip(wrap(self.goal_yaw-self.heading), -.65*dt, .65*dt))
                r = self.ref.copy(); index = self.index; remaining = .6*dt if error < .25 else 0.
                while index < len(self.path) and remaining > 0:
                    d = self.path[index]-r; norm = np.linalg.norm(d)
                    if norm <= remaining:
                        r = self.path[index].copy(); index += 1; remaining -= norm
                    else:
                        r += d/norm*remaining; remaining = 0
                conflict = any(np.linalg.norm(r[:2]-p[:2]) < 1.05 for i, p in self.positions.items() if i != self.id)
                if conflict:
                    self.waits += 1; self.reason = 'separation_hold'
                else:
                    self.ref = r; self.index = index; self.reason = 'tracking_view'
                if index >= len(self.path) and error < .07 and abs(wrap(self.goal_yaw-self.actual_yaw)) < .12:
                    if self.hold_since is None:
                        self.hold_since = t
                    # Dwell long enough for a post-arrival sensor frame.
                    if t-self.hold_since >= .65:
                        self.arrived = True; self.reason = 'view_observed'
        m = PoseStamped(); m.header.frame_id = 'world'; m.header.stamp = now.to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, self.ref)
        m.pose.orientation.w = math.cos(self.heading/2); m.pose.orientation.z = math.sin(self.heading/2)
        self.pub.publish(m)


def main():
    rclpy.init(); node = ViewExecutor()
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
