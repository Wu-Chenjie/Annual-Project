#!/usr/bin/env bash
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is required. See ros2_ws/README.md for Ubuntu and Docker setup." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "Build first: cd ros2_ws && colcon build" >&2
  exit 1
fi
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 launch annual_swarm swarm.launch.py "$@"
