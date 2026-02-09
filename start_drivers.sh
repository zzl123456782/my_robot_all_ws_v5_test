#!/bin/bash

# # 自动查找工作空间路径 (获取当前脚本所在目录)
# WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# # 加载 ROS 环境
# source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash

echo "=========================================="
echo "   Starting Robot Drivers (Unified)       "
echo "=========================================="
echo "Lidar, IMU, and Chassis Driver will be launched."

# 启动 unified launch file
roslaunch robot_bringup drivers.launch

