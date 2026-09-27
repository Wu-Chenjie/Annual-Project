"""External test composition: only executor 0's drone-2 odometry is relayed.

All original controllers, estimators, mappers and planning peer links receive
their unchanged topics. This is a diagnostic launch file, not a policy mode.
"""
import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,GroupAction,OpaqueFunction
from launch.utilities import perform_substitutions,normalize_to_list_of_substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetRemap


def generate_launch_description():
    path=Path(get_package_share_directory('annual_swarm'))/'launch/decentralized_search.launch.py'
    sys.path.insert(0,str(path.parent))
    spec=importlib.util.spec_from_file_location('frozen_swarm_launch',path)
    base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
    def setup(context):
        actions=base.setup(context);executor=actions[8]
        if not isinstance(executor,Node) or perform_substitutions(context,normalize_to_list_of_substitutions(executor.node_executable))!='view_executor_node.py':
            raise RuntimeError('Frozen launch composition changed; do not remap an unknown node')
        remapped=GroupAction(actions=[SetRemap('/drone_2/estimated_odometry','/fault/safety/drone_2/estimated_odometry'),executor],scoped=True)
        relay=ExecuteProcess(cmd=[sys.executable,str(Path(__file__).with_name('safety_input_relay.py')),
            '--ros-args','-p','use_sim_time:=true','-p','output_dir:='+LaunchConfiguration('output_dir').perform(context)],output='screen')
        return actions[:8]+[remapped]+actions[9:]+[relay]
    return LaunchDescription([OpaqueFunction(function=setup) if isinstance(action,OpaqueFunction) else action
                              for action in base.generate_launch_description().entities])
