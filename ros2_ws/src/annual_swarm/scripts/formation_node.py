#!/usr/bin/env python3
"""Three-vehicle world-fixed formation, with feedback-gated path progression."""
import copy
import json
import math
import numpy as np
from planning_runtime import load_from_json
from core.obstacles import Cylinder
from core.artificial_potential_field import ImprovedArtificialPotentialField
from core.planning import FormationAPF
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import String

OFFSETS = [(0., 0., 0.), (0., -1., 0.), (0., 1., 0.)]

def xyz(p): return (p.x, p.y, p.z)
def distance(a,b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

class Formation(Node):
    def __init__(self):
        super().__init__('formation')
        self.path = []; self.index = 0; self.reference = None
        self.states = {}; self.timestamps = {}; self.last_time = None
        self.speed = self.declare_parameter('speed',0.5).value
        if not 0 < self.speed <= 1.0: raise ValueError('speed must be in (0, 1] m/s')
        self.use_apf=self.declare_parameter('apf',False).value
        self.use_formation_apf=self.declare_parameter('formation_apf',False).value
        self.feedforward=self.declare_parameter('velocity_feedforward',False).value
        self.field=load_from_json(self.declare_parameter('map_file','').value)[0] if self.use_apf or self.use_formation_apf else None
        self.static_field=copy.deepcopy(self.field); self.obstacle_version=-1
        self.apfs=[ImprovedArtificialPotentialField(k_inter=.2,s_inter=1.0,k_comm=0.,max_acc=.5) for _ in range(3)]
        self.formation_apf=FormationAPF(r_rep=2.)
        self.state = 'WAITING'; self.terminal_failure = False
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.apf_active_samples=0; self.max_apf=0.; self.max_feedforward=0.
        self.diagnostics_pub=self.create_publisher(String,'/swarm/formation_diagnostics',qos)
        self.create_timer(1.,self.diagnostics)
        self.create_subscription(String,'/swarm/dynamic_obstacles',self.obstacles,qos)
        self.status_pub = self.create_publisher(String,'/swarm/status',qos)
        self.create_subscription(Path,'/swarm/path',self.on_path,qos)
        self.create_subscription(String,'/swarm/planner_status',self.on_planner,qos)
        self.pubs=[]; self.trajectory_pubs=[]
        for i in range(3):
            self.trajectory_pubs.append(self.create_publisher(MultiDOFJointTrajectory,f'/drone_{i}/trajectory_target',1))
            self.pubs.append(self.create_publisher(PoseStamped,f'/drone_{i}/target',1))
            self.create_subscription(Odometry,f'/drone_{i}/odometry',lambda m,i=i:self.on_odom(i,m),qos_profile_sensor_data)
        self.create_timer(0.02,self.tick)

    def obstacles(self,msg):
        if self.static_field is None:return
        data=json.loads(msg.data)
        if data.get('frame_id')!='world' or int(data['version'])<=self.obstacle_version:return
        field=copy.deepcopy(self.static_field)
        for o in data['obstacles']:
            field.add(Cylinder(np.array(o['center_xy'],float),float(o['radius']),tuple(o['z_range'])))
        self.field=field;self.obstacle_version=int(data['version'])

    def diagnostics(self):
        self.diagnostics_pub.publish(String(data=json.dumps({'apf':self.use_apf,
            'formation_apf':self.use_formation_apf,'velocity_feedforward':self.feedforward,
            'apf_active_samples':self.apf_active_samples,'max_apf_acceleration':self.max_apf,
            'max_reference_velocity':self.max_feedforward})))

    def on_path(self,msg):
        if not msg.poses or msg.header.frame_id!='world': return
        self.path = [xyz(p.pose.position) for p in msg.poses]
        initial = self.reference is None
        self.reference = self.path[0]; self.index = 1
        self.state='TAKEOFF' if initial else 'FLYING'
        self.terminal_failure = False

    def on_planner(self,msg):
        if msg.data.startswith('PAUSED') and self.reference is not None:
            self.state='PAUSED'; self.status_pub.publish(String(data=self.state))
        elif msg.data=='READY' and self.state=='PAUSED': self.state='FLYING'
        if msg.data.startswith('FAILED'):
            self.state=msg.data; self.terminal_failure=True
            self.status_pub.publish(String(data=self.state))

    def on_odom(self,i,msg):
        self.states[i] = xyz(msg.pose.pose.position)
        self.timestamps[i] = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def tick(self):
        now = self.get_clock().now(); t=now.nanoseconds*1e-9
        dt=0.0 if self.last_time is None else t-self.last_time; self.last_time=t
        if dt<0:
            self.states.clear(); self.timestamps.clear(); self.index=1
            self.reference=self.path[0] if self.path else None
            self.state='TAKEOFF' if self.path else 'WAITING'
            return
        if self.terminal_failure or self.reference is None or len(self.states)!=3:
            self.status_pub.publish(String(data=self.state)); return
        if any(t-s>0.3 or t-s < -0.1 for s in self.timestamps.values()):
            self.state='FAILED: stale fleet odometry'; self.terminal_failure=True
            self.status_pub.publish(String(data=self.state)); return
        targets=[tuple(a+b for a,b in zip(self.reference,off)) for off in OFFSETS]
        error=max(distance(self.states[i],targets[i]) for i in range(3))
        previous=np.array(self.reference)
        if self.state=='TAKEOFF' and error<0.15: self.state='FLYING'
        if self.state in ('FLYING','HOLDING') and error<0.35:
            remaining=self.speed*min(dt,0.04)
            while self.index<len(self.path) and remaining>0:
                goal=self.path[self.index]; length=distance(goal,self.reference)
                if length<=remaining:
                    self.reference=goal; self.index+=1; remaining-=length
                else:
                    self.reference=tuple(a+(b-a)*remaining/length for a,b in zip(self.reference,goal)); remaining=0
            if self.index==len(self.path) and error<0.15: self.state='HOLDING'
        velocity=(np.array(self.reference)-previous)/dt if 0<dt<=.04 and self.feedforward else np.zeros(3)
        self.max_feedforward=max(self.max_feedforward,float(np.linalg.norm(velocity)))
        accelerations=[np.zeros(3) for _ in range(3)]
        if self.state!='TAKEOFF' and self.field is not None:
            positions=[np.array(self.states[i]) for i in range(3)]
            if self.use_apf or self.use_formation_apf:
                accelerations=[self.apfs[i].compute_avoidance_acceleration(positions[i],
                    np.array(self.reference)+OFFSETS[i],self.field,[positions[j] for j in range(3) if j!=i]) for i in range(3)]
            if self.use_formation_apf:
                lead,followers=self.formation_apf.compute_formation_avoidance(positions[0],positions[1:],
                    np.array(self.reference),self.field,[np.array(x) for x in OFFSETS[1:]])
                accelerations=[a+b for a,b in zip(accelerations,[lead,*followers])]
        for i,(pub,off) in enumerate(zip(self.pubs,OFFSETS)):
            msg=PoseStamped(); msg.header.stamp=now.to_msg(); msg.header.frame_id='world'
            msg.pose.position.x=self.reference[0]+off[0]; msg.pose.position.y=self.reference[1]+off[1]; msg.pose.position.z=self.reference[2]+off[2]
            msg.pose.orientation.w=1.; pub.publish(msg)
            trajectory=MultiDOFJointTrajectory(); trajectory.header=msg.header; trajectory.joint_names=['base_link']
            point=MultiDOFJointTrajectoryPoint(); transform=Transform()
            transform.translation.x=msg.pose.position.x; transform.translation.y=msg.pose.position.y; transform.translation.z=msg.pose.position.z
            transform.rotation.w=1.; v=Twist(); a=Twist()
            v.linear.x,v.linear.y,v.linear.z=map(float,velocity)
            acceleration=np.clip(accelerations[i],-.5,.5)
            magnitude=float(np.linalg.norm(acceleration))
            self.max_apf=max(self.max_apf,magnitude)
            if magnitude>1e-8: self.apf_active_samples+=1
            a.linear.x,a.linear.y,a.linear.z=map(float,acceleration)
            point.transforms=[transform]; point.velocities=[v]; point.accelerations=[a]
            trajectory.points=[point]; self.trajectory_pubs[i].publish(trajectory)
        self.status_pub.publish(String(data=self.state))

def main():
    rclpy.init(); node=Formation()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
