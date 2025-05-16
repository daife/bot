include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  -- 麦轮必须使用里程计数据
  use_odometry = true,
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
  -- 麦轮的里程计数据权重提高
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  -- 麦轮需要高度依赖IMU补偿侧向滑动
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


MAP_BUILDER.use_trajectory_builder_2d = true

-- 根据您机器人的尺寸调整
TRAJECTORY_BUILDER_2D.min_range = 0.06
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- 麦轮一定要启用IMU数据补偿潜在的滑动
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 麦轮特别需要实时回环检测以修正侧向滑动累积误差
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- 对于麦轮，减小最大角度阈值，增加更新频率
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- 麦轮特有设置：提高扫描匹配权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 15
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 12
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50

-- 提高麦轮最小分数阈值，减少错误回环的可能性
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 麦轮专用优化参数
POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- 麦轮的本地SLAM重要性高
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1.2e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1.2e5
-- 麦轮里程计权重需要增加
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1.2e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1.2e5
-- 提高加速度权重，优化麦轮快速变向能力
POSE_GRAPH.optimization_problem.acceleration_weight = 3e3
-- 麦轮专用：增加回环检测频率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.max_num_final_iterations = 10

return options