from pathlib import Path
import json, shutil, difflib, xml.etree.ElementTree as ET, math, hashlib
root=Path(__file__).resolve().parents[2]; out=Path(__file__).resolve().parent
obs=[]
def box(a,b):obs.append(dict(type='aabb',min=a,max=b))
box([-6,-6,0],[-5.85,6,3]);box([5.85,-6,0],[6,6,3]);box([-6,-6,0],[6,-5.85,3]);box([-6,5.85,0],[6,6,3])
box([-.15,-6,0],[.15,-1.5,3]);box([-.15,1.5,0],[.15,6,3]);box([-6,-.15,0],[-2,.15,3]);box([2,-4,0],[2.6,-.5,1.8])
world=dict(bounds=[[-6.,-6.,0.],[6.,6.,3.]],search_starts=[[-4.,-3.,1.2],[-4.,3.,1.2]],obstacles=obs)
(out/'map.json').write_text(json.dumps(world,indent=2))
protocol=dict(status='preflight_not_scored',systems=['Annual-Project','RACER','GVP-MREP'],trial_seed=900,repetitions=1,map_sha256=hashlib.sha256((out/'map.json').read_bytes()).hexdigest(),fleet_size=2,max_velocity_mps=.6,max_acceleration_mps2=.8,max_yaw_rate_radps=.65,lidar=dict(range_m=[.1,4.5],horizontal_fov_deg=120,vertical_fov_deg=120,samples=[181,31],hz=5,noise_std_m=.005,body_offset_m=[0,0,.4]),coverage=dict(target=.95,voxel_m=.3,definition='observed free / geometrically fully free voxels, common external mapper; truth only for scoring'),limit_sim_seconds=300,limit_wall_seconds=2400,shared='Gazebo Harmonic quadrotor physics + lidar + noisy localization/IMU estimator + PID motor controller',adapted='Native mapping, allocation, planning and trajectory sampling retained; ROS1 transport adapter; GVP simulation clock and finite-FOV pointcloud ingress port',limitations=['One paired trial cannot establish statistical superiority','ROS1/ROS2 transport and runtimes differ; record adapter delay','Internal map representation and initialization remain system-specific','A smoke failure is an integration result, never an allocation quality result'])
(out/'protocol.json').write_text(json.dumps(protocol,indent=2))
src=root/'ros1-sim/sources/GVP-MREP'; dst=out/'gvp_ws/src/GVP-MREP'
if not dst.exists():shutil.copytree(src,dst)
diffs=[]
for p in dst.rglob('*'):
 if p.suffix not in ('.cpp','.h','.hpp'):continue
 rel=p.relative_to(dst); original=(src/rel).read_text(); s=original.replace('ros::WallTime','ros::Time')
 if str(rel)=='Exploration/murder_swarm/src/murder.cpp':
  s=s.replace('if(FG_.sensor_type_ == SensorType::CAMERA){','if(false){',1) # finite camera FOV with common PCL ingress
 if str(rel)=='Trajectory/traj_exc/src/traj_exc_node.cpp':
  s=s.replace('std_msgs::EmptyPtr e;\n        ros::Duration(3.0).sleep();\n        Takeoff(e);','ready_ = true; // common simulator handles takeoff; /start_trigger starts planner')
 if s!=original:
  p.write_text(s);diffs.extend(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile='original/'+str(rel),tofile='adapted/'+str(rel)))
(out/'gvp-adaptation.patch').write_text(''.join(diffs))
# Native RACER launch copied, only interface and common physical condition parameters overridden.
src=root/'ros1-sim/sources/RACER/swarm_exploration/exploration_manager/launch'
pt=ET.parse(src/'single_drone_planner.xml'); pn=pt.getroot().find('node')
updates={'sdf_map/ground_height':-.1,'partitioning/use_swarm_tf':'false','sdf_map/box_min_x':-6,'sdf_map/box_min_y':-6,'sdf_map/box_min_z':0,'sdf_map/box_max_x':6,'sdf_map/box_max_y':6,'sdf_map/box_max_z':3,'exploration/yd':.65,'exploration/ydd':.8,'sdf_map/min_ray_length':.1,'perception_utils/top_angle':math.pi/3,'perception_utils/left_angle':math.pi/3,'perception_utils/right_angle':math.pi/3}
for p in pn.findall('param'):
 if p.get('name') in updates:p.set('value',str(updates[p.get('name')]))
pt.write(out/'racer-planner.xml')
st=ET.parse(src/'single_drone_exploration.xml'); sr=st.getroot()
for x in list(sr):
 if x.tag=='include' and x.get('file')!='$(find exploration_manager)/launch/single_drone_planner.xml':sr.remove(x)
sr.find('include').set('file','/comparison/racer-planner.xml')
for a in sr.find('include').findall('arg'):
 if a.get('name')=='max_vel':a.set('value','.6')
 if a.get('name')=='max_acc':a.set('value','.8')
for p in sr.findall('node/param'):
 if p.get('name') in updates:p.set('value',str(updates[p.get('name')]))
st.write(out/'racer-single.xml')
r=ET.Element('launch');ET.SubElement(r,'param',name='use_sim_time',value='true')
for i in (1,2):
 inc=ET.SubElement(r,'include',file='/comparison/racer-single.xml')
 for k,v in dict(drone_id=i,drone_num=2,map_size_x=14,map_size_y=14,map_size_z=3.5,odom_prefix='/common/odom',simulation='false',sensor_pose_topic=f'/common/sensor_pose_{i}').items():ET.SubElement(inc,'arg',name=k,value=str(v))
ET.ElementTree(r).write(out/'racer.launch')
# GVP copied launch with native planning, communication and trajectory executor.
r=ET.Element('launch');ET.SubElement(r,'param',name='use_sim_time',value='true')
for i,start in enumerate(world['search_starts'],1):
 t=ET.parse(root/'ros1-sim/sources/GVP-MREP/Exploration/murder_swarm/launch/murder_single.launch')
 for node in t.getroot().findall('node'):
  raw=ET.tostring(node,encoding='unicode')
  for k,v in dict(id=i,drone_num=2,init_x=start[0],init_y=start[1],init_z=start[2],world='maze4').items():raw=raw.replace('$(arg '+k+')',str(v))
  n=ET.fromstring(raw)
  for rem in n.findall('remap'):
   f=rem.get('from');to=rem.get('to')
   if f in ('/odom','/vi_odom'):rem.set('to',f'/common/odom_{i}')
   if f=='/pointcloud':rem.set('to',f'/pcl_render_node/cloud_{i}')
   if f=='/command/trajectory':rem.set('to',f'/common/command_{i}')
   if to.startswith('/Communication/'):rem.set('to',to.replace('_send','').replace('_rec',''))
  u={'Exp/takeoff_z':1.2,'opt/MaxVel':.6,'opt/MaxAcc':.8,'opt/YawVel':.65,'opt/YawAcc':.8,'Frontier/cam_hor':2*math.pi/3,'Frontier/cam_ver':2*math.pi/3,'block_map/sensor_max_range':4.5,'block_map/CamtoBody_Quater_x':0.,'block_map/CamtoBody_Quater_y':0.,'block_map/CamtoBody_Quater_z':0.,'block_map/CamtoBody_Quater_w':1.,'block_map/CamtoBody_z':.4}
  for axis,a,b in zip('XYZ',[-6.,-6.,0.],[6.,6.,3.]):
   for prefix in ('Exp','block_map'):u[prefix+'/min'+axis]=a;u[prefix+'/max'+axis]=b
  for k,v in u.items():ET.SubElement(n,'param',name=k,value=str(v),type='double')
  r.append(n)
ET.ElementTree(r).write(out/'gvp.launch')
print('prepared',out,'GVP patch lines',len(diffs))
