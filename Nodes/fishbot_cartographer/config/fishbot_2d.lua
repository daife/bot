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
  publish_frame_projected_to_2d = true,  -- 控制是否将位姿信息强制投影到2D平面上。设置为false表示保留完整的3D位姿信息，不进行平面投影。（）
  -- 使用里程计数据
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,  -- 从0.2增加到0.5秒
  submap_publish_period_sec = 0.2,     -- 从0.3减少到0.2秒
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  -- 若使用里程计数据
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  -- 若使用IMU数据
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  
}


MAP_BUILDER.use_trajectory_builder_2d = true
-- 启用实时地图更新
MAP_BUILDER.num_background_threads = 4

-- 确保地图在初始化后立即发布
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- 减少以加快子地图创建

-- 根据机器人的尺寸调整
TRAJECTORY_BUILDER_2D.min_range = 0.06
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- 不使用IMU数据,没调过使用会没有map
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 启用实时回环检测以提高匹配精度
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- 设置最大角度阈值
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- 调整扫描匹配权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 15
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 12
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50

-- 设置最小分数阈值，减少错误回环的可能性
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 优化参数
POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- 本地SLAM权重
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1.2e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1.2e5
-- 使用里程计，里程计权重
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1.2e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1.2e5
-- 加速度权重
POSE_GRAPH.optimization_problem.acceleration_weight = 3e3
-- 回环检测设置
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.max_num_final_iterations = 10
-- 确保启用全局SLAM以创建地图框架
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.global_constraint_search_after_n_seconds = 5  -- 减少时间

return options