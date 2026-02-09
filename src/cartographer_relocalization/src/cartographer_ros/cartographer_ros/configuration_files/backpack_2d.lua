-- Copyright 2016 The Cartographer Authors
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
-- http://www.apache.org/licenses/LICENSE-2.0
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"


options = {
map_builder = MAP_BUILDER,
trajectory_builder = TRAJECTORY_BUILDER,
map_frame = "map",
tracking_frame = "base_link",
published_frame = "base_link",
odom_frame = "odom",
provide_odom_frame = true,
publish_to_tf = true,
publish_frame_projected_to_2d = false,
use_pose_extrapolator = true,
use_odometry = false,
use_nav_sat = false,
use_landmarks = false,
num_laser_scans = 1,
num_multi_echo_laser_scans = 0,
num_subdivisions_per_laser_scan = 1,
num_point_clouds = 0,
lookup_transform_timeout_sec = 0.2,
submap_publish_period_sec = 0.3,
pose_publish_period_sec = 5e-3,
trajectory_publish_period_sec = 30e-3,
rangefinder_sampling_ratio = 1.,
odometry_sampling_ratio = 0.1,
fixed_frame_pose_sampling_ratio = 1.,
imu_sampling_ratio = 1.,
landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 16

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data =45
TRAJECTORY_BUILDER_2D.min_range = 1
TRAJECTORY_BUILDER_2D.max_range = 20

TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.015
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10  -- 增加到25米支持中距离匹配
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(70.)  -- 增加角度搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1

-- Ceres扫描匹配器优化
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 15  -- 减少迭代次数提高速度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- 自适应滤波和运动滤波
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.1
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 20.  -- 支持中距离范围
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.25



POSE_GRAPH.optimize_every_n_nodes = 0 
POSE_GRAPH.optimization_problem.huber_scale = 1e2

POSE_GRAPH.constraint_builder.sampling_ratio = 0.15  -- 增加采样率支持更多约束

-- 分层距离配置（对应C++代码中的三个范围）
POSE_GRAPH.constraint_builder.max_constraint_distance = 20  -- 扩展到80米支持远距离重定位

-- 匹配阈值优化
POSE_GRAPH.constraint_builder.min_score = 0.5  -- 略微降低近距离匹配阈值
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.3-- 降低全局重定位阈值
POSE_GRAPH.global_constraint_search_after_n_seconds = 10  -- 减少到10秒，更频繁的全局搜索
POSE_GRAPH.global_sampling_ratio = 0.002  -- 增加全局采样率



-- 快速correlative扫描匹配器（支持远距离匹配）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.  -- 增加线性搜索窗口
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(90.)  -- 增加角度搜索 45
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 6 -- 减少搜索深度提高速度

-- Ceres约束优化（平衡精度和速度）
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30  -- 减少迭代次数
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 15  -- 增加占用空间权重
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 8    -- 平衡平移权重
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 100      -- 增加旋转权重

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1 -- 适中的分辨率

-- 范围数据配置
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49
POSE_GRAPH.constraint_builder.log_matches = true  -- 启用匹配日志
return options
