#!/usr/bin/env python3
"""Live RViz markers from actual Gazebo odometry and public agent messages."""
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

COLORS = [(1., .32, .18), (.1, .85, .5), (.25, .6, 1.)]


class Visualization(Node):
    def __init__(self):
        super().__init__('exploration_visualization')
        self.fleet_size = int(self.declare_parameter('fleet_size', 3).value)
        self.states = {}; self.graphs = {}; self.poses = {}; self.trails = {i: [] for i in range(self.fleet_size)}; self.map = None; self.diag = {}
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/exploration/markers', q)
        for i in range(self.fleet_size):
            self.create_subscription(Odometry, f'/drone_{i}/odometry', lambda m, i=i: self.odom(i, m), qos_profile_sensor_data)
            self.create_subscription(String, f'/drone_{i}/peer_state', lambda m, i=i: self.states.update({i: json.loads(m.data)}), q)
            self.create_subscription(String, f'/drone_{i}/topology', lambda m, i=i: self.graphs.update({i: json.loads(m.data)}), q)
        self.create_subscription(String, '/experiment/observed_map', lambda m: setattr(self, 'map', json.loads(m.data)), q)
        self.create_subscription(String, '/experiment/diagnostics', lambda m: setattr(self, 'diag', json.loads(m.data)), q)
        self.create_timer(.25, self.tick)

    def odom(self, i, m):
        self.poses[i] = m.pose.pose; p = m.pose.pose.position; value = [p.x, p.y, p.z]
        if not self.trails[i] or np.linalg.norm(np.array(self.trails[i][-1])-value) > .08:
            self.trails[i].append(value)

    def marker(self, ns, ident, kind, color, alpha=1.):
        m = Marker(); m.header.frame_id = 'world'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns; m.id = ident; m.type = kind; m.action = Marker.ADD; m.pose.orientation.w = 1.
        m.color.r, m.color.g, m.color.b = color; m.color.a = alpha
        return m

    @staticmethod
    def points(m, points):
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        return m

    def tick(self):
        out = MarkerArray(); clear = Marker(); clear.action = Marker.DELETEALL; out.markers.append(clear)
        if self.map:
            state = np.array(self.map['state']).reshape(self.map['shape']); res = self.map['resolution']; origin = np.array(self.map['origin'])
            if state.ndim == 3:
                # Show the 1.5m slice and retain actual 3D trajectories / graph.
                state = state[:, :, min(state.shape[2]-1, int(1.5/res))]; origin = origin[:2]
            for value, color, height, z in [(-1, (.25, .28, .34), .025, 0.), (0, (.58, .64, .7), .025, .01), (1, (.66, .75, .86), 1.1, .55)]:
                cells = np.argwhere(state == value); xy = origin+(cells+.5)*res
                m = self.marker('observed_map', value+1, Marker.CUBE_LIST, color)
                m.scale.x = m.scale.y = res*.98; m.scale.z = height
                self.points(m, np.column_stack([xy, np.full(len(xy), z)])); out.markers.append(m)
        # Display one replica's sparse graph, not a misleading union of three copies.
        graph = self.graphs.get(0, {})
        nodes = self.marker('sparse_junctions', 0, Marker.SPHERE_LIST, (1., .85, .3)); nodes.scale.x = nodes.scale.y = nodes.scale.z = .14
        self.points(nodes, [n['position'] for n in graph.get('nodes', [])]); out.markers.append(nodes)
        edges = self.marker('sparse_corridors', 0, Marker.LINE_LIST, (.15, .95, .9)); edges.scale.x = .05
        points = []
        for edge in graph.get('edges', []):
            for a, b in zip(edge['points'][:-1], edge['points'][1:]):
                points.extend([a, b])
        self.points(edges, points); out.markers.append(edges)
        tasks = {int(t['id']): t for g in self.graphs.values() for t in g.get('regions', []) if t.get('status') == 'activeR'}; owners = {}
        for state in self.states.values():
            tasks.update({t['id']: t for t in state.get('tasks', [])}); owners.update(state.get('owners', {}))
        for rid, task in tasks.items():
            if len(task['bounds'][0]) == 3 and not task['bounds'][0][2] <= 1.5 < task['bounds'][1][2]:
                continue
            owner = int(owners.get(str(rid), -1)); color = COLORS[owner % len(COLORS)] if owner >= 0 else (.6, .6, .6)
            low, high = np.asarray(task['bounds'])[:, :2]; x, y = (low+high)/2
            border = self.marker('regional_tasks', rid, Marker.LINE_STRIP, color, .7); border.scale.x = .035
            self.points(border, [[low[0], low[1], .04], [high[0], low[1], .04], [high[0], high[1], .04], [low[0], high[1], .04], [low[0], low[1], .04]])
            out.markers.append(border)
            label = self.marker('region_labels', rid, Marker.TEXT_VIEW_FACING, color); label.scale.z = .3
            label.pose.position.x = float(x); label.pose.position.y = float(y); label.pose.position.z = .35
            label.text = f'R{rid} / '+(f'U{owner}' if owner >= 0 else 'auction'); out.markers.append(label)
        for i, pose in self.poses.items():
            color = COLORS[i % len(COLORS)]; p = pose.position
            body = self.marker('actual_vehicle', i, Marker.CUBE, color); body.pose = pose
            body.scale.x = body.scale.y = .5; body.scale.z = .15; out.markers.append(body)
            trail = self.marker('flown_trajectory', i, Marker.LINE_STRIP, color, .85); trail.scale.x = .055
            self.points(trail, self.trails[i]); out.markers.append(trail)
            state = self.states.get(i, {}); yaw = state.get('yaw', 0.)
            rays = self.marker('camera_fov_120deg', i, Marker.LINE_LIST, color, .6); rays.scale.x = .025
            arc = [[p.x+4.5*math.cos(a), p.y+4.5*math.sin(a), p.z] for a in np.linspace(yaw-math.pi/3, yaw+math.pi/3, 20)]
            points = [[p.x, p.y, p.z], arc[0], [p.x, p.y, p.z], arc[-1]]
            for a, b in zip(arc[:-1], arc[1:]):
                points.extend([a, b])
            self.points(rays, points); out.markers.append(rays)
            intent = state.get('intent')
            if intent:
                path = self.marker('committed_view_route', i, Marker.LINE_STRIP, color, 1. if intent.get('committed') else .35); path.scale.x = .11
                self.points(path, intent['path']); out.markers.append(path)
            label = self.marker('vehicle_labels', i, Marker.TEXT_VIEW_FACING, color); label.scale.z = .45
            label.pose.position.x = p.x; label.pose.position.y = p.y; label.pose.position.z = p.z+.6
            label.text = f'UAV {i} / '+('PAUSED' if not state.get('available', True) else f"R{state.get('active', '-')}" )
            out.markers.append(label)
        text = self.marker('live_status', 0, Marker.TEXT_VIEW_FACING, (.94, .96, 1.)); text.scale.z = .48
        text.pose.position.x = 12.; text.pose.position.y = 22.5; text.pose.position.z = 1.
        d = self.diag; elapsed = d.get('simulation_time', 0)-(d.get('start_time') or 0)
        text.text = (f'FUSED EXPLORATION | {self.fleet_size} UAVs\n'
            f"Coverage {d.get('coverage', 0)*100:.1f}%  |  t={elapsed:.0f}s  |  {d.get('status', 'WAITING')}\n"
            f"Sparse graph: {len(graph.get('nodes', []))} nodes / {graph.get('free_cells', 0)} free cells\n"
            'Hgrid > MR-DTG > GVP + pair CVRP > continuous flight\n'
            f"Network {'RECOVERED' if d.get('network_resumed') else 'PARTITION' if d.get('isolated') else 'OK'} | Restart {'RECOVERED' if d.get('restart_recovered') else 'PENDING' if d.get('restart_trial') else '-'} | Obstacle {'CLEARED' if d.get('dynamic_finished') else 'ACTIVE' if d.get('dynamic_trial') else '-'}")
        out.markers.append(text); self.pub.publish(out)


def main():
    rclpy.init(); node = Visualization()
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
