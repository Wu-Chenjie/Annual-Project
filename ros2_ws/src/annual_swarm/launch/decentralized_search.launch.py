"""Three peer exploration agents and executors, with simulation-only evaluation."""
import json
import os
import sys
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
sys.path.insert(0, str(Path(__file__).parent))
from scene import generate


def setup(context):
    share = Path(get_package_share_directory('annual_swarm'))
    arg = lambda key: LaunchConfiguration(key).perform(context)
    file = arg('map'); data = json.loads(Path(file).read_text()); starts = data['search_starts']
    directory = tempfile.mkdtemp(prefix='annual_decentralized_')
    world, bridge = generate(share, file, directory, starts[0], starts=starts, dynamic=arg('dynamic_obstacle') == 'true', lidar=True)
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s --headless-rendering {world}'}.items())
    common = {'use_sim_time': True}; output = arg('output_dir')
    nodes = [Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': bridge}], output='screen'),
        Node(package='annual_swarm', executable='exploration_experiment_node.py', parameters=[common, {'map_file': file,
            'output_dir': output, 'coverage_target': float(arg('coverage_target')), 'pause_after': float(arg('pause_after')),
            'pause_duration': float(arg('pause_duration')), 'network_after': float(arg('network_after')),
            'network_duration': float(arg('network_duration')), 'restart_after': float(arg('restart_after')), 'dynamic_enabled': arg('dynamic_obstacle') == 'true'}], output='screen')]
    for i in range(len(starts)):
        namespace = f'drone_{i}'
        nodes.extend([
            Node(package='annual_swarm', executable='localization_measurement_node.py', namespace=namespace,
                parameters=[common, {'drone_id': i}], output='screen'),
            Node(package='annual_swarm', executable='state_estimator_node.py', namespace=namespace,
                parameters=[common, {'drone_id': i}], output='screen'),
            Node(package='annual_swarm', executable='pointcloud_mapping_node.py', namespace=namespace,
                parameters=[common, {'drone_id': i, 'bounds': json.dumps(data['bounds'])}], output='screen'),
            Node(package='annual_swarm', executable='decentralized_agent_node.py', namespace=namespace, respawn=True, respawn_delay=2.,
                parameters=[arg('priority_config'), common, {'drone_id': i, 'bounds': json.dumps(data['bounds']), 'fleet_starts': json.dumps(starts), 'output_dir': output}], output='screen'),
            Node(package='annual_swarm', executable='view_executor_node.py', namespace=namespace,
                parameters=[common, {'drone_id': i, 'start': json.dumps(starts[i]), 'fleet_starts': json.dumps(starts)}], output='screen'),
            Node(package='annual_swarm', executable='controller_node', namespace=namespace,
                parameters=[str(share/'config/flight.yaml'), common, {'controller_type': 'pid'}], remappings=[('odometry', 'estimated_odometry')], output='screen')])
    if arg('visualize') == 'true':
        nodes.append(Node(package='annual_swarm', executable='exploration_visualization_node.py', parameters=[common, {'fleet_size': len(starts)}], output='screen'))
    if arg('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/exploration.rviz')], parameters=[common], output='screen'))
    if arg('headless') == 'false':
        nodes.append(ExecuteProcess(cmd=['gz', 'sim', '-g', '--gui-config', str(share/'config/exploration-gazebo.config')], output='screen'))
    if arg('dynamic_obstacle') == 'true':
        nodes.extend([Node(package='ros_gz_bridge', executable='parameter_bridge', name='obstacle_service_bridge',
            arguments=['/world/indoor/set_pose@ros_gz_interfaces/srv/SetEntityPose'], output='screen'),
            Node(package='annual_swarm', executable='dynamic_obstacle_node.py', parameters=[common, {'demo': False}], output='screen')])
    plugin = str(Path(get_package_prefix('annual_swarm'))/'lib')
    return [SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')), sim, *nodes]


def generate_launch_description():
    share = Path(get_package_share_directory('annual_swarm'))
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value=str(share/'maps/search_fusion_3d.json')),
        DeclareLaunchArgument('output_dir', default_value='/tmp/annual_decentralized'),
        DeclareLaunchArgument('priority_config', default_value=str(share/'config/exploration_priority.yaml')),
        DeclareLaunchArgument('coverage_target', default_value='0.95'),
        DeclareLaunchArgument('pause_after', default_value='60.0'),
        DeclareLaunchArgument('pause_duration', default_value='25.0'),
        DeclareLaunchArgument('network_after', default_value='100.0'),
        DeclareLaunchArgument('network_duration', default_value='18.0'),
        DeclareLaunchArgument('restart_after', default_value='210.0'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('visualize', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup)])
