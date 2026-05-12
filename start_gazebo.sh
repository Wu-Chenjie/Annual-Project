#!/bin/bash
# ==========================================
# 无人机仿真一键启动脚本
# 使用: bash start_gazebo.sh [world_file]
# ==========================================

export DISPLAY=localhost:0
export LIBGL_ALWAYS_SOFTWARE=1

source /opt/ros/humble/setup.bash 2>/dev/null
source /usr/share/gazebo/setup.bash 2>/dev/null

WORLD=${1:-/usr/share/gazebo-11/worlds/empty.world}

echo "=== 无人机集群仿真环境 ==="
echo "Gazebo 世界: $WORLD"
echo "显示器: $DISPLAY (VcXsrv)"
echo "渲染: 软件 (llvmpipe)"
echo "============================"

gazebo --verbose "$WORLD"
