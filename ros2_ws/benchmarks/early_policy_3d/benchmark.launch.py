"""Explicit benchmark launch; the production fused entry remains unchanged."""
import json
import os
import sys
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,SetEnvironmentVariable,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
HERE=Path(__file__).resolve().parent

def setup(context):
    share=Path(get_package_share_directory('annual_swarm'));prefix=Path(get_package_prefix('annual_swarm'))
    scripts=prefix/'lib/annual_swarm';sys.path.insert(0,str(share/'launch'))
    from scene import generate
    arg=lambda k:LaunchConfiguration(k).perform(context)
    file=arg('map');data=json.loads(Path(file).read_text());starts=data['search_starts'];output=arg('output_dir')
    world,bridge=generate(share,file,tempfile.mkdtemp(prefix='annual_early_'),starts[0],starts=starts,dynamic=True,lidar=True)
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':f'-r -s --headless-rendering {world}'}.items())
    common={'use_sim_time':True}
    def bench(name,parameters,respawn=False):
        return Node(executable='/usr/bin/python3',namespace=f"drone_{parameters['drone_id']}" if 'drone_id' in parameters else '',arguments=[str(HERE/name)],parameters=[common,parameters],output='screen',respawn=respawn,respawn_delay=2.)
    nodes=[Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':bridge}],output='screen'),
        bench('experiment.py',dict(map_file=file,output_dir=output,coverage_target=.95,pause_after=60.,pause_duration=25.,network_after=100.,network_duration=18.,restart_after=210.,dynamic_enabled=True)),
        bench('coordinator.py',dict(bounds=json.dumps(data['bounds']),output_dir=output))]
    for i in range(3):
        nodes.extend([
            Node(package='annual_swarm',executable='localization_measurement_node.py',namespace=f'drone_{i}',parameters=[common,{'drone_id':i}],output='screen'),
            Node(package='annual_swarm',executable='state_estimator_node.py',namespace=f'drone_{i}',parameters=[common,{'drone_id':i}],output='screen'),
            Node(package='annual_swarm',executable='pointcloud_mapping_node.py',namespace=f'drone_{i}',parameters=[common,{'drone_id':i,'bounds':json.dumps(data['bounds'])}],output='screen'),
            bench('client.py',dict(drone_id=i,output_dir=output),respawn=True),
            bench('executor.py',dict(drone_id=i,start=json.dumps(starts[i]),fleet_starts=json.dumps(starts))),
            Node(package='annual_swarm',executable='controller_node',namespace=f'drone_{i}',parameters=[str(share/'config/flight.yaml'),common,{'controller_type':'pid'}],remappings=[('odometry','estimated_odometry')],output='screen')])
    nodes.extend([bench('visualization.py',dict(fleet_size=3)),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/exploration.rviz')],parameters=[common],output='screen'),
        ExecuteProcess(cmd=['gz','sim','-g','--gui-config',str(share/'config/exploration-gazebo.config')],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='obstacle_service_bridge',arguments=['/world/indoor/set_pose@ros_gz_interfaces/srv/SetEntityPose'],output='screen'),
        Node(package='annual_swarm',executable='dynamic_obstacle_node.py',parameters=[common,{'demo':False}],output='screen')])
    return [SetEnvironmentVariable('PYTHONPATH',os.pathsep.join([str(HERE),str(scripts),str(scripts/'legacy'),os.environ.get('PYTHONPATH','')])),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',str(prefix/'lib')+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),sim,*nodes]

def generate_launch_description():
    share=Path(get_package_share_directory('annual_swarm'))
    return LaunchDescription([DeclareLaunchArgument('map',default_value=str(share/'maps/search_fusion_3d.json')),
        DeclareLaunchArgument('output_dir'),OpaqueFunction(function=setup)])
