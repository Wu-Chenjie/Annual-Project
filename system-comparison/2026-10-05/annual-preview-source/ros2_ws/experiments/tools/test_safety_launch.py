"""Construct the fault composition without starting any ROS or Gazebo process."""
import importlib.util
from pathlib import Path
import pytest

pytest.importorskip('rclpy')
from launch import LaunchContext
from launch.actions import OpaqueFunction,DeclareLaunchArgument,GroupAction
from launch_ros.actions import Node


def test_only_one_executor_is_inside_scoped_fault_group():
    file=Path(__file__).with_name('safety_input_fault.launch.py')
    spec=importlib.util.spec_from_file_location('fault_composition',file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    description=module.generate_launch_description();context=LaunchContext()
    context.launch_configurations.update(headless='true',rviz='false',visualize='false',dynamic_obstacle='false')
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    setup=next(action for action in description.entities if isinstance(action,OpaqueFunction))
    actions=setup.execute(context)
    assert len(actions)==23
    assert sum(isinstance(action,GroupAction) for action in actions)==1
    assert sum(isinstance(action,Node) for action in actions)==19
