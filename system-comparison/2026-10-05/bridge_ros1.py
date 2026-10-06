import sys,json,base64,math,threading,time,zlib,os
from pathlib import Path
from collections import deque
import numpy as np
from tf.transformations import quaternion_matrix
class Rotation:
 @staticmethod
 def from_quat(q):return Rotation(q)
 def __init__(self,q):self.r=quaternion_matrix(q)[:3,:3]
 def apply(self,p):return np.asarray(p)@self.r.T
import rospy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Empty
from trajectory_msgs.msg import MultiDOFJointTrajectory
mode=sys.argv[1];lock=threading.Lock()
rospy.init_node('comparison_transport',disable_signals=True)
clock=rospy.Publisher('/clock',Clock,queue_size=5)
odom=[rospy.Publisher(f'/common/odom_{i+1}',Odometry,queue_size=10) for i in range(2)]
sensor_odom=[rospy.Publisher(f'/common/sensor_odom_{i+1}',Odometry,queue_size=10) for i in range(2)]
poses=[rospy.Publisher(f'/common/sensor_pose_{i+1}',PoseStamped,queue_size=10) for i in range(2)]
clouds=[rospy.Publisher(f'/pcl_render_node/cloud_{i+1}',PointCloud2,queue_size=10) for i in range(2)]
trigger=rospy.Publisher('/start_trigger',Empty,queue_size=1,latch=True)
rc_trigger=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1,latch=True)
buffer=[deque(maxlen=300) for i in range(2)];started=False;stats={'cloud':0,'dropped_cloud':0,'odom':0,'commands':0}
def vec(v):return [v.x,v.y,v.z]
def emit(c):
 stats['commands']+=1
 with lock:print(json.dumps(c,separators=(',',':')),flush=True)
def cmd(i,m):
 emit(dict(type='command',i=i,t=rospy.Time.now().to_sec(),p=vec(m.position),v=vec(m.velocity),a=vec(m.acceleration),yaw=m.yaw))
def traj(i,m):
 if not m.points or not m.points[0].transforms:return
 p=m.points[0];q=p.transforms[0].rotation
 emit(dict(type='command',i=i,t=rospy.Time.now().to_sec(),p=vec(p.transforms[0].translation),v=vec(p.velocities[0].linear) if p.velocities else [0,0,0],a=vec(p.accelerations[0].linear) if p.accelerations else [0,0,0],yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
subs=[]
if mode=='racer':
 from sensor_msgs import point_cloud2
 def occupancy(m):
  if not buffer[0]:return
  t=rospy.Time.now().to_sec()
  if t<15 or t>16:return
  p=np.array(list(point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True)))
  if len(p):
   d=np.linalg.norm(p-np.array(buffer[0][-1]['p']),axis=1)
   Path(os.environ['COMPARISON_OUTPUT']+'/occupied-near-start.json').write_text(json.dumps(dict(t=t,n=len(p),nearest=p[np.argsort(d)[:30]].tolist())))
 subs.append(rospy.Subscriber('/sdf_map/occupancy_all_1',PointCloud2,occupancy,queue_size=1))
if mode=='racer':
 from quadrotor_msgs.msg import PositionCommand
 for i in range(2):subs.append(rospy.Subscriber(f'/planning/pos_cmd_{i+1}',PositionCommand,lambda m,i=i:cmd(i,m),queue_size=1))
else:
 for i in range(2):subs.append(rospy.Subscriber(f'/common/command_{i+1}',MultiDOFJointTrajectory,lambda m,i=i:traj(i,m),queue_size=1))
h,v=np.meshgrid(np.linspace(-math.pi/3,math.pi/3,181),np.linspace(-math.pi/3,math.pi/3,31));directions=np.stack([np.cos(v)*np.cos(h),np.cos(v)*np.sin(h),np.sin(v)],axis=-1).reshape(-1,3)
def make_odom(d,t=None):
 m=Odometry();m.header.stamp=rospy.Time.from_sec(d['t'] if t is None else t);m.header.frame_id='world';m.child_frame_id='world_velocity'
 m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=d['p'];m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=d['q']
 velocity=Rotation.from_quat(d['q']).apply(d['v']);m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=velocity;m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z=d['w'];return m
for line in sys.stdin:
 try:
  d=json.loads(line);kind=d['type']
  if kind=='clock':
   t=d['t'];clock.publish(Clock(rospy.Time.from_sec(t)))
   if t>=20 and not started:
    trigger.publish(Empty());m=PoseStamped();m.header.frame_id='world';m.header.stamp=rospy.Time.from_sec(t);m.pose.position.z=1.2;m.pose.orientation.w=1;rc_trigger.publish(m);started=True
  elif kind=='odom':
   i=d['i'];buffer[i].append(d);odom[i].publish(make_odom(d));stats['odom']+=1
  elif kind=='cloud':
   i=d['i'];stats['cloud']+=1
   if not buffer[i]:stats['dropped_cloud']+=1;continue
   pose=min(buffer[i],key=lambda p:abs(p['t']-d['t']))
   if abs(pose['t']-d['t'])>.15:stats['dropped_cloud']+=1;continue
   raw=base64.b64decode(d['data']);raw=zlib.decompress(raw) if d.get('compressed') else raw;offsets={f[0]:f[1] for f in d['fields']};n=d['width']*d['height'];pts=np.column_stack([np.ndarray((n,),dtype='<f4',buffer=raw,offset=offsets[k],strides=(d['point_step'],)) for k in ('x','y','z')]).astype(float)
   if 9<d['t']<10 and i==0:
    near=pts[np.isfinite(pts).all(axis=1)&(np.linalg.norm(pts+[0,0,.4],axis=1)<.8)]
    Path(os.environ['COMPARISON_OUTPUT']+'/near-body-returns.json').write_text(json.dumps(dict(t=d['t'],count=len(near),points=near[::max(1,len(near)//30)].tolist())))
   valid=np.isfinite(pts).all(axis=1)&(np.linalg.norm(pts,axis=1)<4.495)
   if n!=len(directions):raise RuntimeError('Unexpected lidar organization')
   self_return=np.isfinite(pts).all(axis=1)&np.all(np.abs(pts+[0,0,.4])<=[.34,.34,.10],axis=1)
   pts[~valid]=directions[~valid]*4.51
   pts=pts[~self_return];n=len(pts)
   rot=Rotation.from_quat(pose['q']);origin=np.array(pose['p'])+rot.apply([0,0,.4]);pts=rot.apply(pts)+origin
   m=PointCloud2();m.header.frame_id='world';m.header.stamp=rospy.Time.from_sec(d['t']);m.height=1;m.width=n;m.point_step=12;m.row_step=n*12;m.is_dense=True;m.fields=[PointField(k,j*4,PointField.FLOAT32,1) for j,k in enumerate(('x','y','z'))];m.data=pts.astype('<f4').tobytes()
   p=PoseStamped();p.header=m.header;p.pose.position.x,p.pose.position.y,p.pose.position.z=origin;p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=pose['q'];poses[i].publish(p);sensor_odom[i].publish(make_odom(pose,d['t']));clouds[i].publish(m)
 except Exception as e:print(repr(e),file=sys.stderr,flush=True);raise
print(json.dumps(stats),file=sys.stderr)
