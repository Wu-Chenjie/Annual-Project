import os,sys,json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess,SetEnvironmentVariable,TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory,get_package_prefix

def generate_launch_description():
 share=Path(get_package_share_directory('annual_swarm')); prefix=Path(get_package_prefix('annual_swarm'))
 sys.path.insert(0,str(share/'launch'))
 from scene import generate
 out=Path(os.environ['COMPARISON_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
 mode=os.environ['COMPARISON_MODE'];data=json.loads(Path('/study/map.json').read_text());starts=data['search_starts']
 world,bridge=generate(share,'/study/map.json',out/'scene',starts[0],starts=starts,lidar=True)
 common={'use_sim_time':True}
 nodes=[SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',str(prefix/'lib')),TimerAction(period=8.,actions=[ExecuteProcess(cmd=['gz','sim','-r','-s','--headless-rendering','--seed','900',world],output='screen')]),Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':bridge}],output='screen')]
 for i,start in enumerate(starts):
  ns=f'drone_{i}'
  for exe,params in [('localization_measurement_node.py',{'drone_id':i}),('state_estimator_node.py',{'drone_id':i})]:
   nodes.append(Node(package='annual_swarm',executable=exe,namespace=ns,parameters=[common,params],output='screen'))
  nodes.append(ExecuteProcess(cmd=['python3','/study/common_mapper.py','--ros-args','-r','__ns:=/'+ns,'-p','use_sim_time:=true','-p','drone_id:='+str(i),'-p','bounds:='+json.dumps(json.dumps(data['bounds']))],output='screen'))
  if mode=='annual':
   nodes.append(Node(package='annual_swarm',executable='decentralized_agent_node.py',namespace=ns,parameters=[str(share/'config/exploration_priority.yaml'),common,{'drone_id':i,'bounds':json.dumps(data['bounds']),'fleet_starts':json.dumps(starts),'output_dir':str(out)}],output='screen'))
   nodes.append(Node(package='annual_swarm',executable='view_executor_node.py',namespace=ns,parameters=[common,{'drone_id':i,'start':json.dumps(start),'fleet_starts':json.dumps(starts)}],remappings=[('target','native_target'),('trajectory_target','native_trajectory_target')],output='screen'))
  nodes.append(Node(package='annual_swarm',executable='controller_node',namespace=ns,parameters=[str(share/'config/flight.yaml'),common,{'controller_type':'pid'}],remappings=[('odometry','estimated_odometry')],output='screen'))
 return LaunchDescription(nodes)
