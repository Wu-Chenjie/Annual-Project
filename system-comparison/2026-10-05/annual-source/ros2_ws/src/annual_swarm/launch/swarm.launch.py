import os
from pathlib import Path
import sys
import tempfile
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Works from both source tree and installed share directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'scripts'))
from scene import generate

def setup(context):
    share = Path(get_package_share_directory('annual_swarm'))
    map_file = LaunchConfiguration('map').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    output = LaunchConfiguration('output_dir').perform(context)
    start = [float(x) for x in LaunchConfiguration('start').perform(context).split(',')]
    goal = [float(x) for x in LaunchConfiguration('goal').perform(context).split(',')]
    if len(start)!=3 or len(goal)!=3: raise ValueError('start and goal require x,y,z')
    scene_dir = tempfile.mkdtemp(prefix='annual_swarm_')
    dynamic=LaunchConfiguration('dynamic_obstacle').perform(context)=='true'
    portfolio=LaunchConfiguration('planner').perform(context)=='portfolio'
    if dynamic and not portfolio: raise ValueError('dynamic_obstacle requires planner:=portfolio')
    world, bridge = generate(share,map_file,scene_dir,start,dynamic)
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':f'-r {"-s" if headless else ""} {world}', 'on_exit_shutdown':'true'}.items())
    common = {'use_sim_time':True}
    config = str(share/'config/flight.yaml')
    nodes = [
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='gazebo_bridge',parameters=[{'config_file':bridge}],output='screen'),
        Node(package='annual_swarm',executable='candidate_pool_node.py' if portfolio else 'planning_node.py',parameters=[config,common,{'map_file':str(Path(scene_dir)/'planner_map.json'),'start':start,'goal':goal, 'recovery_policy':LaunchConfiguration('recovery_policy').perform(context),'candidate_planners':LaunchConfiguration('candidate_planners').perform(context),'candidate_variants':int(LaunchConfiguration('candidate_variants').perform(context)),'quality_weights':LaunchConfiguration('quality_weights').perform(context), 'planner_type':LaunchConfiguration('planner').perform(context), 'replan_interval':float(LaunchConfiguration('replan_interval').perform(context)), 'esdf':LaunchConfiguration('esdf').perform(context)=='true', 'firi':LaunchConfiguration('firi').perform(context)=='true', 'trajectory':LaunchConfiguration('trajectory').perform(context)}],output='screen'),
        Node(package='annual_swarm',executable='formation_node.py',parameters=[config,common,{'map_file':str(Path(scene_dir)/'planner_map.json'), 'apf':LaunchConfiguration('apf').perform(context)=='true','formation_apf':LaunchConfiguration('formation_apf').perform(context)=='true','velocity_feedforward':LaunchConfiguration('velocity_feedforward').perform(context)=='true'}],output='screen'),
        Node(package='annual_swarm',executable='metrics_node.py',parameters=[config,common,{'output_dir':output}],output='screen')]
    if dynamic:
        nodes.extend([
            Node(package='ros_gz_bridge',executable='parameter_bridge',name='obstacle_service_bridge',
                 arguments=['/world/indoor/set_pose@ros_gz_interfaces/srv/SetEntityPose'],output='screen'),
            Node(package='annual_swarm',executable='dynamic_obstacle_node.py',parameters=[common,{'demo':LaunchConfiguration('dynamic_demo').perform(context)=='true','demo_mode':LaunchConfiguration('dynamic_demo_mode').perform(context)}],output='screen')])
    for i in range(3):
        nodes.append(Node(package='annual_swarm',executable='controller_node',namespace=f'drone_{i}',parameters=[config,common,{'controller_type':LaunchConfiguration('controller').perform(context)}],output='screen'))
    # A crashed bridge / controller / coordinator must stop the run, not leave an unobserved vehicle flying.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[EmitEvent(event=Shutdown(reason='swarm process exited'))])) for node in nodes]
    plugin_path = str(Path(get_package_prefix('annual_swarm'))/'lib')
    return [SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',plugin_path+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')), *handlers, sim, *nodes]

def generate_launch_description():
    share = get_package_share_directory('annual_swarm')
    return LaunchDescription([
        DeclareLaunchArgument('headless',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('apf',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('formation_apf',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('velocity_feedforward',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('planner',default_value='astar',choices=['astar','heading_astar','hybrid_astar','dijkstra','rrt_star','informed_rrt_star','dstar_lite','gnn','window','portfolio']),
        DeclareLaunchArgument('recovery_policy',default_value='cache',choices=['cache','replan_single','replan_pool']),
        DeclareLaunchArgument('candidate_planners',default_value='astar,hybrid_astar,rrt_star'),
        DeclareLaunchArgument('candidate_variants',default_value='4'),
        DeclareLaunchArgument('quality_weights',default_value='{"length":1.0,"clearance":0.8,"turning":0.15,"acceleration":0.15,"jerk":0.05}'),
        DeclareLaunchArgument('dynamic_obstacle',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('dynamic_demo_mode',default_value='backup_switch',choices=['backup_switch','block_goal','benchmark']),
        DeclareLaunchArgument('dynamic_demo',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('controller' ,default_value='pid',choices=['pid','smc','backstepping','backstepping_pid','geometric_euler','super_twisting']),
        DeclareLaunchArgument('replan_interval',default_value='0.0'),
        DeclareLaunchArgument('esdf',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('firi',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('trajectory',default_value='none',choices=['none','moving_average','minimum_jerk','min_snap_proxy','min_jerk_cost']),
        DeclareLaunchArgument('map',default_value=share+'/maps/sample_simple.json'),
        DeclareLaunchArgument('start',default_value='2,3,1.5'),
        DeclareLaunchArgument('goal',default_value='18,16,1.5'),
        DeclareLaunchArgument('output_dir',default_value=str(Path.home()/'annual_swarm_results')),
        OpaqueFunction(function=setup)])
