"""Construct a diagnostic launch without starting flight or simulation nodes."""
import importlib.util
from pathlib import Path
import pytest

pytest.importorskip('rclpy')
from launch import LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node


def test_physical_cylinder_and_frozen_flight_nodes_are_retained():
    file = Path(__file__).with_name('dynamic_backup_fault.launch.py')
    spec = importlib.util.spec_from_file_location('dynamic_composition', file)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    description = module.generate_launch_description(); context = LaunchContext()
    context.launch_configurations.update(headless='true', rviz='false', visualize='false', dynamic_obstacle='true')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument): action.execute(context)
    actions = next(action for action in description.entities if isinstance(action, OpaqueFunction)).execute(context)
    assert len(actions) == 25
    assert sum(isinstance(action, Node) for action in actions) == 22
