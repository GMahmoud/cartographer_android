#include <string>
#include <glog/logging.h>

#ifndef CARTOGRAPHER_GENERIC_BUDDY_LIDAR_2D_LUA_H_
#define CARTOGRAPHER_GENERIC_BUDDY_LIDAR_2D_LUA_H_

namespace cartographer_generic{
class buddy_lua{
public :
	std::string code_lua ,code_sparse, code_map_builder, code_traj_2d, code_traj_3d, code_traj;


	std::string GetCode(const string& basename){
		if(basename == "buddy_lidar_2d.lua"){
			code_lua = "\ninclude \"map_builder.lua\"\ninclude \"trajectory_builder.lua\"\n\noptions = {\n  map_builder = MAP_BUILDER,\n  trajectory_builder = TRAJECTORY_BUILDER,\n  map_frame = \"map\",\n  tracking_frame = \"buddy\",\n  published_frame = \"buddy\",\n  odom_frame = \"odom\",\n  provide_odom_frame = true,\n  use_odometry = true,\n  use_laser_scan = true,\n  use_multi_echo_laser_scan = false,\n  num_point_clouds = 0,\n  num_laser_scans = 1,\n  num_multi_echo_laser_scans = 0,\n  lookup_transform_timeout_sec = 0.2,\n  submap_publish_period_sec = 0.3,\n  pose_publish_period_sec = 5e-3,\n  num_subdivisions_per_laser_scan = 1,\n  trajectory_publish_period_sec = 30e-3,\n}\n\nMAP_BUILDER.use_trajectory_builder_2d = true\nTRAJECTORY_BUILDER_2D.use_imu_data = false\n--TRAJECTORY_BUILDER.pure_localization = true\n\n--TRAJECTORY_BUILDER_2D.min_range = 0.1\n--TRAJECTORY_BUILDER_2D.max_range = 10.\n--TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.\n--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true\n--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)\n\n--SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.65\n--SPARSE_POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7\nSPARSE_POSE_GRAPH.optimize_every_n_scans = 40\n\n--Increasing the loop closure search window in cartographer/configuration_files/sparse_pose_graph.lua\nSPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.0\nSPARSE_POSE_GRAPH.constraint_builder.max_constraint_distance = 11.0\n\nreturn options";
			LOG(INFO) << "Calling buddy.lua" ;
			return code_lua;
		}
		if(basename == "map_builder.lua"){
			code_map_builder = "include \"sparse_pose_graph.lua\"\n\nMAP_BUILDER = {\n  use_trajectory_builder_2d = false,\n  use_trajectory_builder_3d = false,\n  num_background_threads = 4,\n  sparse_pose_graph = SPARSE_POSE_GRAPH,\n}";
			LOG(INFO) << "Calling map_builder.lua" ;
			return code_map_builder;
		}
		if(basename == "sparse_pose_graph.lua"){
			code_sparse = "SPARSE_POSE_GRAPH = {\n  optimize_every_n_scans = 90,\n  constraint_builder = {\n    sampling_ratio = 0.3,\n    max_constraint_distance = 15.,\n    adaptive_voxel_filter = {\n      max_length = 0.9,\n      min_num_points = 100,\n      max_range = 50.,\n    },\n    min_score = 0.55,\n    global_localization_min_score = 0.6,\n    loop_closure_translation_weight = 1.1e4,\n    loop_closure_rotation_weight = 1e5,\n    log_matches = true,\n    fast_correlative_scan_matcher = {\n      linear_search_window = 7.,\n      angular_search_window = math.rad(30.),\n      branch_and_bound_depth = 7,\n    },\n    ceres_scan_matcher = {\n      occupied_space_weight = 20.,\n      translation_weight = 10.,\n      rotation_weight = 1.,\n      ceres_solver_options = {\n        use_nonmonotonic_steps = true,\n        max_num_iterations = 10,\n        num_threads = 1,\n      },\n    },\n    fast_correlative_scan_matcher_3d = {\n      branch_and_bound_depth = 8,\n      full_resolution_depth = 3,\n      rotational_histogram_size = 120,\n      min_rotational_score = 0.77,\n      linear_xy_search_window = 5.,\n      linear_z_search_window = 1.,\n      angular_search_window = math.rad(15.),\n    },\n    high_resolution_adaptive_voxel_filter = {\n      max_length = 2.,\n      min_num_points = 150,\n      max_range = 15.,\n    },\n    low_resolution_adaptive_voxel_filter = {\n      max_length = 4.,\n      min_num_points = 200,\n      max_range = 60.,\n    },\n    ceres_scan_matcher_3d = {\n      occupied_space_weight_0 = 5.,\n      occupied_space_weight_1 = 30.,\n      translation_weight = 10.,\n      rotation_weight = 1.,\n      only_optimize_yaw = false,\n      ceres_solver_options = {\n        use_nonmonotonic_steps = false,\n        max_num_iterations = 10,\n        num_threads = 1,\n      },\n    },\n  },\n  matcher_translation_weight = 5e2,\n  matcher_rotation_weight = 1.6e3,\n  optimization_problem = {\n    huber_scale = 1e1,\n    acceleration_weight = 1e3,\n    rotation_weight = 3e5,\n    consecutive_scan_translation_penalty_factor = 1e5,\n    consecutive_scan_rotation_penalty_factor = 1e5,\n    log_solver_summary = false,\n    ceres_solver_options = {\n      use_nonmonotonic_steps = false,\n      max_num_iterations = 50,\n      num_threads = 7,\n    },\n  },\n  max_num_final_iterations = 200,\n  global_sampling_ratio = 0.003,\n}";
			LOG(INFO) << "Calling sparse_pose_graph.lua" ;
			return code_sparse;
		}
		if(basename == "trajectory_builder_2d.lua"){
			code_traj_2d = "TRAJECTORY_BUILDER_2D = {\n  use_imu_data = true,\n  min_range = 0.,\n  max_range = 30.,\n  min_z = -0.8,\n  max_z = 2.,\n  missing_data_ray_length = 5.,\n  scans_per_accumulation = 1,\n  voxel_filter_size = 0.025,\n\n  adaptive_voxel_filter = {\n    max_length = 0.5,\n    min_num_points = 200,\n    max_range = 50.,\n  },\n\n  use_online_correlative_scan_matching = false,\n  real_time_correlative_scan_matcher = {\n    linear_search_window = 0.1,\n    angular_search_window = math.rad(20.),\n    translation_delta_cost_weight = 1e-1,\n    rotation_delta_cost_weight = 1e-1,\n  },\n\n  ceres_scan_matcher = {\n    occupied_space_weight = 1.,\n    translation_weight = 10.,\n    rotation_weight = 40.,\n    ceres_solver_options = {\n      use_nonmonotonic_steps = false,\n      max_num_iterations = 20,\n      num_threads = 1,\n    },\n  },\n\n  motion_filter = {\n    max_time_seconds = 5.,\n    max_distance_meters = 0.2,\n    max_angle_radians = math.rad(1.),\n  },\n\n  imu_gravity_time_constant = 10.,\n  num_odometry_states = 1000,\n\n  submaps = {\n    resolution = 0.05,\n    num_range_data = 90,\n    range_data_inserter = {\n      insert_free_space = true,\n      hit_probability = 0.55,\n      miss_probability = 0.49,\n    },\n  },\n}";
			LOG(INFO) << "Calling trajectory_builder_2d.lua" ;
			return code_traj_2d;
		}
		if(basename == "trajectory_builder_3d.lua"){
			code_traj_3d = "MAX_3D_RANGE = 60.\n\nTRAJECTORY_BUILDER_3D = {\n  min_range = 1.,\n  max_range = MAX_3D_RANGE,\n  scans_per_accumulation = 1,\n  voxel_filter_size = 0.15,\n\n  high_resolution_adaptive_voxel_filter = {\n    max_length = 2.,\n    min_num_points = 150,\n    max_range = 15.,\n  },\n\n  low_resolution_adaptive_voxel_filter = {\n    max_length = 4.,\n    min_num_points = 200,\n    max_range = MAX_3D_RANGE,\n  },\n\n  use_online_correlative_scan_matching = false,\n  real_time_correlative_scan_matcher = {\n    linear_search_window = 0.15,\n    angular_search_window = math.rad(1.),\n    translation_delta_cost_weight = 1e-1,\n    rotation_delta_cost_weight = 1e-1,\n  },\n\n  ceres_scan_matcher = {\n    occupied_space_weight_0 = 1.,\n    occupied_space_weight_1 = 6.,\n    translation_weight = 5.,\n    rotation_weight = 4e2,\n    only_optimize_yaw = false,\n    ceres_solver_options = {\n      use_nonmonotonic_steps = false,\n      max_num_iterations = 12,\n      num_threads = 1,\n    },\n  },\n\n  motion_filter = {\n    max_time_seconds = 0.5,\n    max_distance_meters = 0.1,\n    max_angle_radians = 0.004,\n  },\n\n  imu_gravity_time_constant = 10.,\n  num_odometry_states = 1,\n\n  submaps = {\n    high_resolution = 0.10,\n    high_resolution_max_range = 20.,\n    low_resolution = 0.45,\n    num_range_data = 160,\n    range_data_inserter = {\n      hit_probability = 0.55,\n      miss_probability = 0.49,\n      num_free_space_voxels = 2,\n    },\n  },\n}";
			LOG(INFO) << "Calling trajectory_builder_3d.lua" ;
			return code_traj_3d;
		}
		if(basename == "trajectory_builder.lua"){
			code_traj = "include \"trajectory_builder_2d.lua\"\ninclude \"trajectory_builder_3d.lua\"\n\nTRAJECTORY_BUILDER = {\n  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,\n  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,\n  pure_localization = false,\n}";
			LOG(INFO) << "Calling trajectory_builder.lua" ;
			return code_traj;
		}

		LOG(FATAL) << "Check basenames in your lua import ";
	}
};

}//namespace cartographer_generic
#endif //CARTOGRAPHER_GENERIC_BUDDY_LIDAR_2D_LUA_H_
