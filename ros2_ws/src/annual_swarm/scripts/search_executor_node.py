#!/usr/bin/env python3
"""Independent Gazebo reference execution with fleet separation interlock."""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class SearchExecutor(Node):
    def __init__(self):
        super().__init__('search_executor');starts=json.loads(self.declare_parameter('starts','[]').value)
        self.refs={i:np.array(p,float) for i,p in enumerate(starts)};self.positions={};self.stamps={}
        self.paths={};self.indices={};self.epochs={};self.arrived={i:True for i in self.refs};self.available=set(self.refs)
        self.ready=False;self.done=False;self.failure=None;self.last=None;self.speed=.6;self.waits=0
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs={i:self.create_publisher(PoseStamped,f'/drone_{i}/target',1) for i in self.refs}
        for i in self.refs:
            self.create_subscription(Odometry,f'/drone_{i}/odometry',lambda m,i=i:self.odom(i,m),qos_profile_sensor_data)
            self.create_subscription(String,f'/drone_{i}/search_path',lambda m,i=i:self.path(i,m),q)
        self.create_subscription(String,'/search/control',self.control,q)
        self.pub=self.create_publisher(String,'/search/execution',q)
        self.create_timer(.02,self.tick);self.create_timer(.2,self.report)
    def odom(self,i,m):
        p=m.pose.pose.position;self.positions[i]=np.array([p.x,p.y,p.z]);self.stamps[i]=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def path(self,i,m):
        packet=json.loads(m.data)
        if packet is None:
            self.paths.pop(i,None);self.arrived[i]=True
            if i in self.positions:self.refs[i]=self.positions[i].copy();self.refs[i][2]=1.5
            return
        if packet['epoch']<=self.epochs.get(i,0):return
        points=np.array(packet['path'],float)
        if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():return
        self.paths[i]=points;self.indices[i]=0;self.epochs[i]=packet['epoch'];self.arrived[i]=False
    def control(self,m):
        packet=json.loads(m.data);new=set(packet.get('available',self.available));self.done=packet.get('done',False)
        for i in self.available-new:
            self.paths.pop(i,None);self.arrived[i]=True
            if i in self.positions:self.refs[i]=self.positions[i].copy();self.refs[i][2]=1.5
        self.available=new
    def report(self):
        self.pub.publish(String(data=json.dumps(dict(ready=self.ready,arrived=self.arrived,epochs=self.epochs,
            failure=self.failure,traffic_wait_samples=self.waits,done=self.done))))
    def tick(self):
        now=self.get_clock().now();t=now.nanoseconds*1e-9;dt=0 if self.last is None else min(max(t-self.last,0),.04);self.last=t
        if len(self.positions)!=len(self.refs):return
        if any(t-stamp>.5 or stamp>t+.1 for stamp in self.stamps.values()):self.failure='stale odometry'
        if not self.ready:self.ready=all(np.linalg.norm(self.positions[i]-r)<.15 for i,r in self.refs.items())
        if self.ready and not self.done and self.failure is None:
            proposed={i:r.copy() for i,r in self.refs.items()};new_indices=dict(self.indices)
            for i,path in self.paths.items():
                if i not in self.available or self.arrived[i]:continue
                if np.linalg.norm(self.positions[i]-self.refs[i])>.25:continue
                remaining=self.speed*dt;index=self.indices[i];r=self.refs[i].copy()
                while index<len(path) and remaining>0:
                    delta=path[index]-r;length=np.linalg.norm(delta)
                    if length<=remaining:r=path[index].copy();index+=1;remaining-=length
                    else:r+=delta/length*remaining;remaining=0
                proposed[i]=r;new_indices[i]=index
            for i,r in proposed.items():
                # Full-route reservations provide prevention; this interlock catches execution drift.
                conflict=any(np.linalg.norm(r[:2]-self.positions[j][:2])<1.0 or np.linalg.norm(r[:2]-proposed[j][:2])<1.0 for j in self.refs if j!=i)
                if conflict:self.waits+=1;continue
                self.refs[i]=r
                if i in new_indices:self.indices[i]=new_indices[i]
                if i in self.paths and self.indices[i]>=len(self.paths[i]) and np.linalg.norm(self.positions[i]-r)<.15:self.arrived[i]=True
        for i,r in self.refs.items():
            m=PoseStamped();m.header.frame_id='world';m.header.stamp=now.to_msg();m.pose.orientation.w=1.
            m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,r);self.pubs[i].publish(m)

def main():
    rclpy.init();node=SearchExecutor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
