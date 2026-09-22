#!/usr/bin/env python3
"""Selectable legacy planners with nonblocking, odometry-based ROS replanning."""
import json
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from planning_runtime import Runtime

class Planning(Node):
    def __init__(self):
        super().__init__('planner')
        def param(name,value): return self.declare_parameter(name,value).value
        self.start=np.array(param('start',[2.,3.,1.5])); self.goal=np.array(param('goal',[18.,16.,1.5]))
        self.runtime=Runtime(param('map_file',''),param('planner_type','astar'),self.start[2],
            param('clearance',1.9),param('esdf',False),param('firi',False),param('trajectory','none'))
        self.interval=param('replan_interval',0.0)
        if self.runtime.algorithm=='window' and self.interval==0: self.interval=2.0
        if self.interval<0: raise ValueError('replan_interval must be nonnegative')
        self.position=None; self.airborne=False; self.last_plan=-1e9; self.revision=0; self.ready=False
        self.executor_pool=ThreadPoolExecutor(max_workers=1); self.future=None
        self.last_status='WAITING'; self.pending=True
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(Path,'/swarm/path',qos)
        self.status_pub=self.create_publisher(String,'/swarm/planner_status',qos)
        self.diag_pub=self.create_publisher(String,'/swarm/planner_diagnostics',qos)
        self.create_subscription(Odometry,'/drone_0/odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/swarm/goal',self.new_goal,1)
        self.create_timer(.1,self.tick)
    def odom(self,msg):
        p=msg.pose.pose.position; self.position=np.array([p.x,p.y,p.z])
        self.airborne=abs(p.z-self.start[2])<.15
    def new_goal(self,msg):
        if msg.header.frame_id!='world': return
        p=msg.pose.position; goal=np.array([p.x,p.y,p.z])
        if not self.runtime.safe_point(goal) or abs(goal[2]-self.start[2])>1e-6:
            self.get_logger().error('Rejected unsafe mission goal'); return
        self.goal=goal; self.revision+=1; self.pending=True
    def tick(self):
        t=self.get_clock().now().nanoseconds*1e-9
        if self.future is not None and self.future.done():
            future=self.future; self.future=None
            if self.job_revision!=self.revision: self.pending=True
            else:
                try:
                    path,report=future.result()
                    # A slow planner can finish after the vehicle has moved. Validate the joining segment.
                    if self.ready and self.position is not None:
                        p=self.position.copy(); p[2]=self.start[2]
                        closest=int(np.argmin(np.linalg.norm(path-p,axis=1)))
                        path=np.vstack([p,path[closest:]])
                        if not self.runtime.safe_path(path): raise ValueError('Unsafe join to replanned route')
                    msg=Path(); msg.header.frame_id='world'; msg.header.stamp=self.get_clock().now().to_msg()
                    for point in path:
                        pose=PoseStamped(); pose.header=msg.header
                        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,point)
                        pose.pose.orientation.w=1.; msg.poses.append(pose)
                    self.path_pub.publish(msg); self.ready=True; self.last_status='READY'
                    report['goal']=self.goal.tolist(); self.diag_pub.publish(String(data=json.dumps(report)))
                    self.get_logger().info(json.dumps(report))
                except Exception as exc:
                    # Keep the previously validated route on a periodic-planning failure.
                    self.last_status=('DEGRADED: ' if self.ready else 'FAILED: ')+str(exc)
                    self.diag_pub.publish(String(data=json.dumps({'algorithm':self.runtime.algorithm,'error':str(exc)})))
                    self.get_logger().error(self.last_status)
            self.last_plan=t
        due=self.interval>0 and self.airborne and np.linalg.norm(self.position-self.goal)>.3 and t-self.last_plan>=self.interval
        if self.future is None and (self.pending or due) and self.position is not None:
            self.pending=False; self.job_revision=self.revision
            start=self.position.copy() if self.ready else self.start.copy(); start[2]=self.start[2]
            goal=self.goal.copy()
            def work():
                if not self.ready: self.runtime.check_takeoff(start)
                return self.runtime.plan(start,goal,t)
            self.future=self.executor_pool.submit(work)
        self.status_pub.publish(String(data=self.last_status))

def main():
    rclpy.init(); node=Planning()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.executor_pool.shutdown(wait=False,cancel_futures=True); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
