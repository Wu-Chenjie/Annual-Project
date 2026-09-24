#!/usr/bin/env python3
"""Per-UAV transport endpoint for the old centralized planner; not a planner."""
import json
import os
import uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from contracts import view_finished


class Client(Node):
    def __init__(self):
        super().__init__('early_policy_client')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        self.session = uuid.uuid4().hex; self.execution = {}; self.position = None; self.speed = 0.; self.yaw = 0.
        self.intent = None; self.epoch = 0; self.core_epoch = 0; self.completed_core_epoch = -1
        self.commits = 0; self.views = 0; self.seen = set(); self.control_state = {}; self.central_time = -1e9
        self.restarting = False; self.stop_since = None; self.central_stats = {}; self.rejoin_ready = False
        out = Path(self.declare_parameter('output_dir', '').value)/f'drone_{self.id}'; out.mkdir(parents=True, exist_ok=True)
        self.log = (out/'events.jsonl').open('a')
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, f'/drone_{self.id}/peer_state', q)
        self.command_pub = self.create_publisher(String, f'/drone_{self.id}/view_command', q)
        self.create_subscription(String, f'/baseline/command_{self.id}', self.command, q)
        self.create_subscription(String, f'/drone_{self.id}/view_execution', self.feedback, q)
        self.create_subscription(Odometry, f'/drone_{self.id}/estimated_odometry', self.odom, qos_profile_sensor_data)
        self.create_subscription(String, '/experiment/control', self.control, q)
        self.create_timer(.2, self.tick)
        self.event('client_started', role='centralized_command_transport_endpoint')

    def now(self): return self.get_clock().now().nanoseconds*1e-9
    def event(self, kind, **data):
        self.log.write(json.dumps(dict(type=kind, drone=self.id, time=self.now(), session=self.session, **data))+'\n'); self.log.flush()

    def odom(self, m):
        p=m.pose.pose.position; q=m.pose.pose.orientation; v=m.twist.twist.linear
        self.position=np.array([p.x,p.y,p.z]); self.speed=float(np.linalg.norm([v.x,v.y,v.z]))
        self.yaw=float(np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def stop(self, reason):
        if self.intent is None:
            return
        self.epoch=max(self.epoch,int(self.execution.get('epoch',0)))+1
        self.command_pub.publish(String(data=json.dumps(dict(epoch=self.epoch,cancel=True))))
        self.event('route_cancelled',reason=reason,token=self.intent['token'])
        self.intent=None

    def control(self, m):
        p=json.loads(m.data); self.control_state=p
        if self.id in p.get('restart',[]) and p.get('restart_session')==self.session and not self.restarting:
            self.restarting=True; self.event('client_restart_requested'); self.stop('process_restart')
        if self.id in p.get('paused',[]) or self.id in p.get('isolated',[]) or p.get('done'):
            self.stop('experiment_control')

    def available(self):
        return not (self.restarting or self.control_state.get('done') or self.id in self.control_state.get('paused',[]) or self.id in self.control_state.get('isolated',[]))

    def command(self,m):
        p=json.loads(m.data)
        if p['session']!=self.session or self.id in self.control_state.get('isolated',[]): return
        self.central_time=p['time']; self.central_stats=p.get('stats',{})
        self.rejoin_ready=True
        if p.get('cancel'):
            self.stop(p.get('reason','central_cancel')); return
        packet=p.get('packet')
        if packet is None or not self.available() or not self.execution.get('ready'): return
        if packet['token'] in self.seen: return
        self.seen.add(packet['token']); self.epoch=max(self.epoch,int(self.execution.get('epoch',0)))+1
        self.core_epoch=packet['core_epoch']; packet['epoch']=self.epoch
        self.intent=dict(token=packet['token'],region=packet['task'],path=packet['reservation_path'],committed=True,
                         recovery=packet['recovery'],retiring=False,duration=packet['trajectory']['duration'])
        self.commits+=1; self.tick()
        self.command_pub.publish(String(data=json.dumps(packet)))
        self.event('path_committed',token=packet['token'],core_epoch=self.core_epoch,task=packet['task'],trajectory_limits=packet['trajectory']['limits'],recovery=packet['recovery'])

    def feedback(self,m):
        self.execution=json.loads(m.data)
        if self.intent and self.available() and view_finished(self.execution,self.epoch,self.intent['duration']):
            if not self.intent['recovery']:
                self.views+=1; self.completed_core_epoch=self.core_epoch
            self.event('view_observed',token=self.intent['token'],recovery=self.intent['recovery'])
            self.intent=None

    def tick(self):
        t=self.now()
        if t-self.central_time>3.: self.stop('central_watchdog')
        if self.restarting:
            if self.speed<.08 and self.execution.get('arrived'):
                if self.stop_since is None:self.stop_since=t
                if t-self.stop_since>=.5:
                    self.event('client_process_exit'); self.log.close(); os._exit(77)
            else:self.stop_since=None
        p=dict(drone=self.id,session=self.session,time=t,central_time=self.central_time,position=self.position.tolist() if self.position is not None else [0.,0.,0.],
               yaw=self.yaw,speed=self.speed,available=self.available(),ready=self.execution.get('ready',False),rejoin_ready=self.rejoin_ready,
               intent=self.intent,active=self.intent['region'] if self.intent else None,commits=self.commits,observed_views=self.views,
               completed_core_epoch=self.completed_core_epoch,core_epoch=self.core_epoch,execution=self.execution,
               stopped=self.speed<.08 and self.execution.get('arrived',False),graph=self.central_stats,
               role='centralized_command_transport_endpoint',restarting=self.restarting)
        self.pub.publish(String(data=json.dumps(p)))


def main():
    rclpy.init(); node=Client()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
