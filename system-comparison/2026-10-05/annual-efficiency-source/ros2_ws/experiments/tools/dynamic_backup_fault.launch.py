"""External physical-obstacle test; the frozen flight policy stays unchanged."""
import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


def generate_launch_description():
    path = Path(get_package_share_directory('annual_swarm'))/'launch/decentralized_search.launch.py'
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location('frozen_dynamic_swarm_launch', path)
    base = importlib.util.module_from_spec(spec); spec.loader.exec_module(base)

    def setup(context):
        actions = base.setup(context); evaluator = actions[3]
        executable = perform_substitutions(context, normalize_to_list_of_substitutions(evaluator.node_executable)) if isinstance(evaluator, Node) else None
        if executable != 'exploration_experiment_node.py':
            raise RuntimeError('Frozen launch composition changed; unknown evaluator')
        # Keep the cylinder and its Gazebo service adapter, but give this one
        # diagnostic injector sole control over its placement and timing.
        arg = lambda name: LaunchConfiguration(name).perform(context)
        replacement = Node(package='annual_swarm', executable='exploration_experiment_node.py',
            parameters=[{'use_sim_time': True}, {'map_file': arg('map'), 'output_dir': arg('output_dir'),
                'coverage_target': float(arg('coverage_target')), 'pause_after': float(arg('pause_after')),
                'pause_duration': float(arg('pause_duration')), 'network_after': float(arg('network_after')),
                'network_duration': float(arg('network_duration')), 'restart_after': float(arg('restart_after')),
                'dynamic_enabled': False}], output='screen')
        probe = ExecuteProcess(cmd=[sys.executable, str(Path(__file__).with_name('dynamic_backup_probe.py')),
            '--ros-args', '-p', 'use_sim_time:=true', '-p', 'output_dir:='+LaunchConfiguration('output_dir').perform(context),
            '-p', 'map_file:='+LaunchConfiguration('map').perform(context)], output='screen')
        return actions[:3]+[replacement]+actions[4:]+[probe]

    return LaunchDescription([OpaqueFunction(function=setup) if isinstance(action, OpaqueFunction) else action
                              for action in base.generate_launch_description().entities])
