
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "buddy",
  published_frame = "buddy",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 0,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  num_subdivisions_per_laser_scan = 1,
  trajectory_publish_period_sec = 30e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
--TRAJECTORY_BUILDER.pure_localization = true

--TRAJECTORY_BUILDER_2D.min_range = 0.1
--TRAJECTORY_BUILDER_2D.max_range = 10.
--TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.
--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

--SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.65
--SPARSE_POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
SPARSE_POSE_GRAPH.optimize_every_n_scans = 40

--Increasing the loop closure search window in cartographer/configuration_files/sparse_pose_graph.lua
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.0
SPARSE_POSE_GRAPH.constraint_builder.max_constraint_distance = 11.0

return options
