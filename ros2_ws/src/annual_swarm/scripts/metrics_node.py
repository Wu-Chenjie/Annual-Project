#!/usr/bin/env python3
"""Record Gazebo truth and flight metrics; atomic summary survives interruption."""
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts

class Metrics(Node):
    def __init__(self):
        super().__init__('metrics')
        root=Path(self.declare_parameter('output_dir',str(Path.home()/'annual_swarm_results')).value)
        self.output=root/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.output.mkdir(parents=True,exist_ok=False)
        self.candidate_file=(self.output/'candidate_paths.jsonl').open('w')
        self.file=(self.output/'trajectory.csv').open('w',newline='')
        self.writer=csv.writer(self.file); self.writer.writerow(['time','drone','x','y','z','target_x','target_y','target_z','error'])
        self.transitions=[]
        self.targets={}; self.positions={}; self.airborne=set(); self.samples=0; self.squared_error=0.; self.max_error=0.
        self.min_separation=None; self.contacts=0; self.status='WAITING'; self.planner_diagnostics={}; self.dynamic_obstacles={}; self.formation_diagnostics={}; self.controllers={}; self.latest_t=0.
        self.odom_counts={str(i):0 for i in range(3)}
        self.imu_counts={str(i):0 for i in range(3)}
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String,'/swarm/status',self.on_status,qos)
        self.create_subscription(String,'/swarm/candidate_paths',self.on_candidates,qos)
        self.create_subscription(String,'/swarm/dynamic_obstacles',lambda m:setattr(self,'dynamic_obstacles',json.loads(m.data)),qos)
        self.create_subscription(String,'/swarm/formation_diagnostics',lambda m:setattr(self,'formation_diagnostics',json.loads(m.data)),qos)
        self.create_subscription(String,'/swarm/planner_diagnostics',lambda m:setattr(self,'planner_diagnostics',json.loads(m.data)),qos)
        for i in range(3):
            self.create_subscription(String,f'/drone_{i}/controller_status',lambda m,i=i:self.controllers.update({str(i):m.data}),qos)
            self.create_subscription(PoseStamped,f'/drone_{i}/target',lambda m,i=i:self.targets.update({i:m.pose.position}),1)
            self.create_subscription(Odometry,f'/drone_{i}/odometry',lambda m,i=i:self.on_odom(i,m),qos_profile_sensor_data)
            self.create_subscription(Imu,f'/drone_{i}/imu',lambda m,i=i:self.on_imu(i,m),qos_profile_sensor_data)
            self.create_subscription(Contacts,f'/drone_{i}/contacts',lambda m,i=i:self.on_contacts(i,m),qos_profile_sensor_data)
        self.create_timer(1.,self.save)
        self.get_logger().info(f'Results: {self.output}')

    def on_candidates(self,msg):
        snapshot=json.loads(msg.data);snapshot['recorded_simulation_time']=self.get_clock().now().nanoseconds*1e-9
        self.candidate_file.write(json.dumps(snapshot,allow_nan=False)+'\n');self.candidate_file.flush()

    def on_imu(self,i,msg): self.imu_counts[str(i)]+=1
    def on_status(self,msg):
        if msg.data!=self.status:
            self.transitions.append(dict(status=msg.data,simulation_time=self.get_clock().now().nanoseconds*1e-9))
        self.status=msg.data
    def on_contacts(self,i,msg):
        for c in msg.contacts:
            names=c.collision1.name+' '+c.collision2.name
            # Ground contact while sitting on the floor is expected before takeoff.
            if 'ground' in names and i not in self.airborne: continue
            self.contacts+=1

    def on_odom(self,i,msg):
        p=msg.pose.pose.position; self.positions[i]=p; self.odom_counts[str(i)]+=1
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9; self.latest_t=t
        if p.z>0.5: self.airborne.add(i)
        target=self.targets.get(i)
        if target is None: return
        error=math.sqrt((p.x-target.x)**2+(p.y-target.y)**2+(p.z-target.z)**2)
        self.samples+=1; self.squared_error+=error**2; self.max_error=max(self.max_error,error)
        self.writer.writerow([t,i,p.x,p.y,p.z,target.x,target.y,target.z,error])
        for j,other in self.positions.items():
            if j==i: continue
            d=math.sqrt((p.x-other.x)**2+(p.y-other.y)**2+(p.z-other.z)**2)
            self.min_separation=d if self.min_separation is None else min(self.min_separation,d)

    def save(self):
        summary={'state_transitions':self.transitions,'dynamic_obstacles':self.dynamic_obstacles,'formation':self.formation_diagnostics,'planner':self.planner_diagnostics,'controllers':self.controllers,'simulation_time':self.latest_t,'mission_status':self.status,'samples':self.samples,
            'odometry_samples':self.odom_counts,'imu_samples':self.imu_counts,'tracking_rmse_m':math.sqrt(self.squared_error/self.samples) if self.samples else None,
            'max_tracking_error_m':self.max_error,'min_pairwise_distance_m':self.min_separation,
            'contact_samples_after_takeoff':self.contacts,
            'positions':{str(i):[p.x,p.y,p.z] for i,p in self.positions.items()}}
        self.file.flush()
        tmp=self.output/'summary.json.tmp'; tmp.write_text(json.dumps(summary,indent=2,allow_nan=False)); tmp.replace(self.output/'summary.json')

def main():
    rclpy.init(); node=Metrics()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.save(); node.file.close(); node.candidate_file.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
