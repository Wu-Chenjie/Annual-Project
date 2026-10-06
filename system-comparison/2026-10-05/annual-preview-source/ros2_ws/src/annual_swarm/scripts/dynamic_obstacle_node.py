#!/usr/bin/env python3
"""Move a real Gazebo cylinder and publish acknowledged geometry snapshots.

This is a simulation truth adapter, not lidar detection. Manual world-frame
PoseStamped commands on /swarm/move_obstacle can move it repeatedly.
"""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros_gz_interfaces.srv import SetEntityPose

class DynamicObstacle(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle')
        self.demo=self.declare_parameter('demo',False).value
        self.demo_mode=self.declare_parameter('demo_mode','backup_switch').value
        self.client=self.create_client(SetEntityPose,'/world/indoor/set_pose')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(String,'/swarm/dynamic_obstacles',qos)
        self.create_subscription(PoseStamped,'/swarm/move_obstacle',self.command,1)
        self.create_subscription(String,'/swarm/candidate_paths',self.candidates,qos)
        self.create_subscription(Odometry,'/drone_0/odometry',self.odom,qos_profile_sensor_data)
        self.pool=None;self.position=None;self.pending=None;self.future=None;self.version=0;self.demo_sent=False
        self.create_timer(.1,self.tick)
    def candidates(self,msg):self.pool=json.loads(msg.data)
    def odom(self,msg):
        p=msg.pose.pose.position;self.position=np.array([p.x,p.y,p.z])
    def command(self,msg):
        p=msg.pose.position
        if msg.header.frame_id=='world' and np.isfinite([p.x,p.y,p.z]).all():self.pending=np.array([p.x,p.y,1.5])
    def tick(self):
        if self.future is not None and self.future.done():
            result=self.future.result();self.future=None
            if result.success:
                self.version+=1
                snapshot=dict(frame_id='world',version=self.version,stamp=self.get_clock().now().nanoseconds*1e-9,
                    source='Gazebo SetEntityPose success',trigger_leader_position=self.position.tolist() if self.position is not None else None,obstacles=[dict(id='dynamic_obstacle',center_xy=self.requested[:2].tolist(),radius=.55,z_range=[0.,3.])])
                self.pub.publish(String(data=json.dumps(snapshot)));self.get_logger().info(json.dumps(snapshot))
            else:self.get_logger().error('Gazebo rejected obstacle movement; geometry was not published')
        if self.demo and not self.demo_sent and self.pool and self.position is not None and self.position[2]>1.3:
            active=np.array(self.pool['active_path']);backups=[np.array(x['points']) for x in self.pool['backup_paths']]
            if len(active)>1 and len(backups)>=5 and self.demo_mode=='benchmark':
                if self.position[0]>=3.0:
                    self.pending=np.array([9.5,5.,1.5]);self.demo_sent=True
            elif len(active)>1 and len(backups)>=5 and self.demo_mode=='block_goal':
                self.pending=active[-1].copy();self.demo_sent=True
            elif len(active)>1 and len(backups)>=5:
                # Select a reproducible ahead-of-fleet obstruction with a genuine reserve route.
                for point in active[len(active)//3:2*len(active)//3]:
                    if np.linalg.norm(point-self.position)<6.:continue
                    if any(np.min(np.linalg.norm(route-point,axis=1))>3.0 for route in backups):
                        self.pending=point.copy();self.demo_sent=True;break
        if self.pending is not None and self.future is None and self.client.service_is_ready():
            req=SetEntityPose.Request();req.entity.name='dynamic_obstacle';req.entity.type=2
            req.pose.position.x,req.pose.position.y,req.pose.position.z=map(float,self.pending);req.pose.orientation.w=1.
            self.requested=self.pending.copy();self.pending=None;self.future=self.client.call_async(req)

def main():
    rclpy.init();node=DynamicObstacle()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
