#!/bin/bash

source devel_isolated/setup.bash

# 获取工作空间的根目录
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 设置相对路径到目标目录
map_dir="$workspace_dir/src/cartographer_ros/cartographer_ros/map"
map_name="2d"

# 检查文件夹是否存在，如果不存在就创建文件夹
if [ ! -d "$map_dir" ]; then
  echo "地图文件夹不存在，正在创建文件夹: $map_dir"
  mkdir -p $map_dir
fi

# 检查pbstream文件是否已存在，如果存在则重命名
if [ -f "$map_dir/$map_name.pbstream" ]; then
  timestamp=$(date +"%Y%m%d_%H%M%S")
  mv "$map_dir/$map_name.pbstream" "$map_dir/${map_name}_backup_$timestamp.pbstream"
  echo "已备份现有pbstream文件"
fi

# finish slam
echo "正在结束轨迹..."
rosservice call /finish_trajectory 0

# 等待一小段时间
sleep 2

# make pbstream
echo "正在保存地图状态..."
rosservice call /write_state "{filename: '$map_dir/$map_name.pbstream', include_unfinished_submaps: false}"

# 等待文件写入完成
sleep 3

# 检查pbstream文件是否成功创建
if [ ! -f "$map_dir/$map_name.pbstream" ]; then
  echo "错误: pbstream文件未创建成功!"
  exit 1
fi

echo "pbstream文件创建成功，大小: $(du -h "$map_dir/$map_name.pbstream" | cut -f1)"

# 转换为ROS地图格式
echo "正在转换为ROS地图格式..."
# 添加-resolution参数，明确指定分辨率
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename="$map_dir/$map_name.pbstream" \
  -map_filestem="$map_dir/$map_name" \
  -resolution=0.05

# 检查转换是否成功
if [ $? -eq 0 ] && [ -f "$map_dir/${map_name}.pgm" ]; then
  echo "地图转换成功完成!"
  echo "PGM地图文件: $map_dir/${map_name}.pgm"
  echo "YAML配置文件: $map_dir/${map_name}.yaml"
else
  echo "地图转换可能失败，检查错误信息"
fi