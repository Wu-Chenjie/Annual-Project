import sys,os,json,threading,queue,base64,time,math,csv,zlib,gzip
from pathlib import Path
from collections import defaultdict
from motion_envelope import Envelope
from geometry_check import intersects
import numpy as np
sys.path.insert(0,'/workspace/next_project')
from core.exploration.voxel_mapping import VoxelMap,VoxelTruth
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,Transform,Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from ros_gz_interfaces.msg import Contacts
mode=os.environ['COMPARISON_MODE'];out=Path(os.environ['COMPARISON_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
limit=float(os.environ.get('COMPARISON_LIMIT','300'))
q_in=queue.Queue()
def reader():
 for line in sys.stdin:
  try:q_in.put(json.loads(line))
  except ValueError:pass
threading.Thread(target=reader,daemon=True).start()
def vec(v):return [v.x,v.y,v.z]
def quat(q):return [q.x,q.y,q.z,q.w]
class Bridge(Node):
 def __init__(self):
  super().__init__('comparison_observer',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
  self.geometry=json.loads(Path('/comparison/map.json').read_text())['obstacles'];self.geometric_hits=[];self.bad_tracking={};self.failure=None
  self.t=0.;self.last_report=-1.;self.begin=time.monotonic();self.truth=VoxelTruth('/comparison/map.json');self.observed=VoxelMap(self.truth.bounds);self.per_observed=[VoxelMap(self.truth.bounds) for _ in range(2)]
  self.starts=json.loads(Path('/comparison/map.json').read_text())['search_starts'];self.cmd={};self.last_pos={};self.distance=defaultdict(float);self.count=defaultdict(int);self.maxspeed=0.;self.minimum=None;self.contacts=0;self.air=set();self.t95=None;self.t90=None;self.done=False;self.delays=[];self.est_last={};self.subs=[];self.pubs=[];self.posepub=[];self.native_seen=set();self.max_command_speed=0.;self.max_command_acc=0.;self.tracking=[]
  self.stream=gzip.open(out/'sensor-stream.jsonl.gz','wt',compresslevel=1);self.executed=(out/'executed-commands.jsonl').open('w');self.contact_log=(out/'contacts.jsonl').open('w')
  self.log=(out/'trajectory.csv').open('w');self.csv=csv.writer(self.log);self.csv.writerow(['t','i','x','y','z','speed'])
  self.cover=(out/'coverage.jsonl').open('w');self.commands=(out/'commands.jsonl').open('w')
  self.subs.append(self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data))
  for i in range(2):
   for topic,typ,cb,qos in [('estimated_odometry',Odometry,self.estimated,qos_profile_sensor_data),('odometry',Odometry,self.odom,qos_profile_sensor_data),('lidar/points',PointCloud2,self.cloud,qos_profile_sensor_data),('contacts',Contacts,self.contact,qos_profile_sensor_data),('observation',String,self.obs,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)),('trajectory_target',MultiDOFJointTrajectory,self.target,10)]:
    self.subs.append(self.create_subscription(typ,f'/drone_{i}/{topic}',lambda m,i=i,cb=cb:cb(i,m),qos))
   self.pubs.append(self.create_publisher(MultiDOFJointTrajectory,f'/drone_{i}/trajectory_target',1));self.posepub.append(self.create_publisher(PoseStamped,f'/drone_{i}/target',1))
  self.native_yaw={}
  if mode=='annual':
   for i in range(2):
    self.subs.append(self.create_subscription(PoseStamped,f'/drone_{i}/native_target',lambda m,i=i:self.annual_pose(i,m),1))
    self.subs.append(self.create_subscription(MultiDOFJointTrajectory,f'/drone_{i}/native_trajectory_target',lambda m,i=i:self.annual_traj(i,m),1))
  self.envelopes=[Envelope([p[0],p[1],.1]) for p in self.starts];self.last_control=0.;self.raw_maxspeed=0.;self.raw_maxacc=0.
  self.create_timer(.033,self.tick)
 def annual_pose(self,i,m):
  q=m.pose.orientation;self.native_yaw[i]=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 def annual_traj(self,i,m):
  if not m.points:return
  p=m.points[0]
  q_in.put(dict(type='command',i=i,t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p=vec(p.transforms[0].translation),v=vec(p.velocities[0].linear),a=vec(p.accelerations[0].linear),yaw=self.native_yaw.get(i,0.)))
 def emit(self,m):
  line=json.dumps(m,separators=(',',':'));self.stream.write(line+'\n')
  if mode!='annual':print(line,flush=True)
 def clock(self,m):
  self.t=m.clock.sec+m.clock.nanosec*1e-9
  
  if self.t-getattr(self,'last_clock_sent',-1)>=.009:
   self.emit({'type':'clock','t':self.t});self.last_clock_sent=self.t
 def estimated(self,i,m):
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if t-self.est_last.get(i,-1)<.018:return
  self.est_last[i]=t
  self.emit(dict(type='odom',i=i,t=t,p=vec(m.pose.pose.position),q=quat(m.pose.pose.orientation),v=vec(m.twist.twist.linear),w=vec(m.twist.twist.angular)))
  self.count[f'estimated_{i}']+=1
 def cloud(self,i,m):
  self.count[f'cloud_{i}']+=1
  self.emit(dict(type='cloud',i=i,t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,height=m.height,width=m.width,point_step=m.point_step,row_step=m.row_step,fields=[[f.name,f.offset,f.datatype,f.count] for f in m.fields],data=base64.b64encode(zlib.compress(bytes(m.data),1)).decode(),compressed=True))
 def obs(self,i,m):
  p=json.loads(m.data);cells=np.array(p['indices'],int).reshape(-1,3)
  if len(cells):
   self.observed.update(cells,np.asarray(p['values'],np.int8));self.per_observed[i].update(cells,np.asarray(p['values'],np.int8))
  self.count[f'observation_{i}']+=1
 def odom(self,i,m):
  p=np.array(vec(m.pose.pose.position));speed=float(np.linalg.norm(vec(m.twist.twist.linear)));self.maxspeed=max(self.maxspeed,speed)
  if i in self.last_pos:self.distance[i]+=float(np.linalg.norm(p-self.last_pos[i]))
  self.last_pos[i]=p
  if p[2]>.5:self.air.add(i)
  if len(self.last_pos)==2:
   d=float(np.linalg.norm(self.last_pos[0]-self.last_pos[1]));self.minimum=d if self.minimum is None else min(self.minimum,d)
  if self.t>=20 and self.count[f'odom_{i}']%5==0:
   for k,ob in enumerate(self.geometry):
    if intersects(p,quat(m.pose.pose.orientation),ob['min'],ob['max']):
     self.geometric_hits.append(dict(t=self.t,i=i,obstacle=k));self.failure='physical_hull_intersects_obstacle';break
  self.csv.writerow([self.t,i,*p,speed]);self.count[f'odom_{i}']+=1
 def contact(self,i,m):
  for c in m.contacts:
   if 'ground' in c.collision1.name+c.collision2.name and i not in self.air:continue
   self.contacts+=1;self.contact_log.write(json.dumps(dict(t=self.t,i=i,a=c.collision1.name,b=c.collision2.name))+'\n')
 def target(self,i,m):
  if not m.points:return
  p=m.points[0]
  self.executed.write(json.dumps(dict(t=self.t,i=i,p=vec(p.transforms[0].translation),v=vec(p.velocities[0].linear),a=vec(p.accelerations[0].linear)))+'\n')
  if p.velocities:self.max_command_speed=max(self.max_command_speed,float(np.linalg.norm(vec(p.velocities[0].linear))))
  if p.accelerations:self.max_command_acc=max(self.max_command_acc,float(np.linalg.norm(vec(p.accelerations[0].linear))))
  if p.transforms and i in self.last_pos:
   err=float(np.linalg.norm(np.array(vec(p.transforms[0].translation))-self.last_pos[i]));self.tracking.append(err)
   if self.t>=20 and err>.8:
    self.bad_tracking.setdefault(i,self.t)
    if self.t-self.bad_tracking[i]>1.:self.failure='tracking_error_exceeds_0.8m_for_1s'
   else:self.bad_tracking.pop(i,None)
 def tick(self):
  while not q_in.empty():
   c=q_in.get()
   if c.get('type')=='command':
    self.raw_maxspeed=max(self.raw_maxspeed,float(np.linalg.norm(c['v'])));self.raw_maxacc=max(self.raw_maxacc,float(np.linalg.norm(c['a'])))
    self.cmd[c['i']]=c;self.native_seen.add(c['i']);self.delays.append(self.t-c['t']);self.commands.write(json.dumps(dict(c,received_sim_t=self.t))+'\n');self.count['native_commands']+=1
  if True:
   for i,start in enumerate(self.starts):
    # Public startup/hold only; native planner owns every exploration motion.
    c=self.cmd.get(i);valid=c is not None and self.t>=20 and -.15<=self.t-c['t']<=.5
    if valid:p,v,a,yaw=c['p'],c['v'],c['a'],c['yaw']
    else:
     p=list(start);p[2]=min(start[2],.10+.3*self.t);v=[0.,0.,0.];a=v;yaw=min(2*math.pi,max(0,self.t-5)*.6)
     if self.t>=20 and i in self.native_seen:p=c['p'];yaw=c['yaw']
    p,v,a,yaw=self.envelopes[i].step(p,v,yaw,self.t-self.last_control)
    stamp=self.get_clock().now().to_msg();m=MultiDOFJointTrajectory();m.header.frame_id='world';m.header.stamp=stamp
    pt=MultiDOFJointTrajectoryPoint();tr=Transform();tr.translation.x,tr.translation.y,tr.translation.z=map(float,p);tr.rotation.w=math.cos(yaw/2);tr.rotation.z=math.sin(yaw/2)
    vel=Twist();vel.linear.x,vel.linear.y,vel.linear.z=map(float,v);acc=Twist();acc.linear.x,acc.linear.y,acc.linear.z=map(float,a);pt.transforms=[tr];pt.velocities=[vel];pt.accelerations=[acc];m.points=[pt]
    pm=PoseStamped();pm.header=m.header;pm.pose.position.x,pm.pose.position.y,pm.pose.position.z=map(float,p);pm.pose.orientation=tr.rotation;self.posepub[i].publish(pm);self.pubs[i].publish(m)
  self.last_control=self.t
  if self.t-self.last_report<1:return
  self.last_report=self.t;cov=self.truth.coverage(self.observed)
  if cov>=.9 and self.t90 is None:self.t90=self.t
  if cov>=.95 and self.t95 is None:self.t95=self.t
  result=dict(mode=mode,t=self.t,wall_seconds=time.monotonic()-self.begin,coverage=cov,t90=self.t90,t95=self.t95,distance_m=dict(self.distance),contacts=self.contacts,min_separation_m=self.minimum,max_measured_speed_mps=self.maxspeed,max_command_speed_mps=self.max_command_speed,max_command_acc_mps2=self.max_command_acc,tracking_error_p95_m=float(np.percentile(self.tracking,95)) if self.tracking else None,counts=dict(self.count),positions={k:v.tolist() for k,v in self.last_pos.items()},native_seen=sorted(self.native_seen),adapter_command_age_p95_s=float(np.percentile(self.delays,95)) if self.delays else None,truth_free_voxels=int((~self.truth.occupied).sum()),per_drone_coverage=[self.truth.coverage(m) for m in self.per_observed],overlap_free_voxels=int(np.count_nonzero((self.per_observed[0].state==0)&(self.per_observed[1].state==0)&~self.truth.occupied)),raw_planner_max_speed_mps=self.raw_maxspeed,raw_planner_max_acc_mps2=self.raw_maxacc,status='RUNNING')
  if self.t>=limit:result['status']='TIME_LIMIT'
  if self.t95 is not None:result['status']='COVERAGE_TARGET'
  result['geometric_collision_events']=self.geometric_hits;result['failure_reason']=self.failure
  if self.failure:result['status']='SAFETY_FAILURE'
  self.cover.write(json.dumps(result)+'\n');self.cover.flush();self.log.flush();self.commands.flush();self.stream.flush();self.executed.flush();self.contact_log.flush()
  tmp=out/'progress.tmp';tmp.write_text(json.dumps(result,indent=2));tmp.replace(out/'progress.json')
  if result['status']!='RUNNING':(out/'result.json').write_text(json.dumps(result,indent=2));self.done=True
rclpy.init();b=Bridge()
try:
 while rclpy.ok() and not b.done:rclpy.spin_once(b,timeout_sec=.1)
finally:
 b.stream.close();b.executed.close();b.contact_log.close();b.log.close();b.commands.close();b.cover.close();b.destroy_node();rclpy.shutdown()
