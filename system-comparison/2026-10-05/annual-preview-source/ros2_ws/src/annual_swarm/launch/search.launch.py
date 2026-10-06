"""Independent observed-map multi-UAV search; formation launch remains separate."""
import json
import os
import sys
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,SetEnvironmentVariable,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
sys.path.insert(0,str(Path(__file__).parent))
from scene import generate

def setup(context):
    share=Path(get_package_share_directory('annual_swarm'))
    arg=lambda key:LaunchConfiguration(key).perform(context)
    map_file=arg('map');data=json.loads(Path(map_file).read_text());starts=data['search_starts']
    if len(starts)!=3:raise ValueError('This flight configuration requires exactly three start positions')
    directory=tempfile.mkdtemp(prefix='annual_search_');world,bridge=generate(share,map_file,directory,starts[0],starts=starts,dynamic=arg('dynamic_obstacle')=='true')
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':f'-r {"-s" if arg("headless")=="true" else ""} {world}'}.items())
    common={'use_sim_time':True}
    nodes=[Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':bridge}],output='screen'),
        Node(package='annual_swarm',executable='search_node.py',parameters=[common,{'map_file':map_file,'output_dir':arg('output_dir'),'policy':arg('policy'),
            'coverage_target':float(arg('coverage_target')),'pause_after':float(arg('pause_after')),'pause_duration':float(arg('pause_duration'))}],output='screen'),
        Node(package='annual_swarm',executable='search_executor_node.py',parameters=[common,{'starts':json.dumps(starts)}],output='screen')]
    if arg('dynamic_obstacle')=='true':
        nodes.extend([Node(package='ros_gz_bridge',executable='parameter_bridge',name='obstacle_service_bridge',arguments=['/world/indoor/set_pose@ros_gz_interfaces/srv/SetEntityPose'],output='screen'),
            Node(package='annual_swarm',executable='dynamic_obstacle_node.py',parameters=[common,{'demo':False}],output='screen')])
    for i in range(3):nodes.append(Node(package='annual_swarm',executable='controller_node',namespace=f'drone_{i}',parameters=[str(share/'config/flight.yaml'),common,{'controller_type':'pid'}],output='screen'))
    handlers=[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='search node exited'))])) for n in nodes]
    plugin=str(Path(get_package_prefix('annual_swarm'))/'lib')
    return [SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',plugin+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),*handlers,sim,*nodes]

def generate_launch_description():
    share=Path(get_package_share_directory('annual_swarm'))
    return LaunchDescription([DeclareLaunchArgument('headless',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('dynamic_obstacle',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('map',default_value=str(share/'maps/search_office.json')),
        DeclareLaunchArgument('policy',default_value='gvp_pairwise',choices=['nearest_frontier','graph_voronoi','gvp_pairwise']),
        DeclareLaunchArgument('output_dir',default_value='/tmp/annual_search'),DeclareLaunchArgument('coverage_target',default_value='0.95'),
        DeclareLaunchArgument('pause_after',default_value='0.0'),DeclareLaunchArgument('pause_duration',default_value='30.0'),OpaqueFunction(function=setup)])
