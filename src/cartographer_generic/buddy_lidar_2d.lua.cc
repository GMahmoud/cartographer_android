/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_generic/buddy_lidar_2d.lua.h"

namespace cartographer_generic{

std::string GetCode(const std::string& basename){
	if(basename == "buddy_lidar_2d.lua"){
		LOG(INFO) << "Calling buddy.lua" ;
		return code_lua;
	}
	if(basename == "map_builder.lua"){
		LOG(INFO) << "Calling map_builder.lua" ;
		return code_map_builder;
	}
	if(basename == "sparse_pose_graph.lua"){
		LOG(INFO) << "Calling sparse_pose_graph.lua" ;
		return code_sparse;
	}
	if(basename == "trajectory_builder_2d.lua"){
		LOG(INFO) << "Calling trajectory_builder_2d.lua" ;
		return code_traj_2d;
	}
	if(basename == "trajectory_builder_3d.lua"){
		LOG(INFO) << "Calling trajectory_builder_3d.lua" ;
		return code_traj_3d;
	}
	if(basename == "trajectory_builder.lua"){
		LOG(INFO) << "Calling trajectory_builder.lua" ;
		return code_traj;
	}

	LOG(FATAL) << "Check basenames in your lua export ";
}

void SetCode(std::string& basename, std::string& code){

	if(basename == "buddy_lidar_2d"){
		LOG(INFO) << "Setting buddy.lua" ;
		code_lua = code;
		return;
	}
	if(basename == "map_builder"){
		LOG(INFO) << "Setting map_builder.lua" ;
		code_map_builder = code;
		return;
	}
	if(basename == "sparse_pose_graph"){
		LOG(INFO) << "Setting sparse_pose_graph.lua" ;
		code_sparse = code;
		return;
	}
	if(basename == "trajectory_builder_2d"){
		LOG(INFO) << "Setting trajectory_builder_2d.lua" ;
		code_traj_2d = code;
		return;
	}
	if(basename == "trajectory_builder_3d"){
		LOG(INFO) << "Setting trajectory_builder_3d.lua" ;
		code_traj_3d = code;
		return;
	}
	if(basename == "trajectory_builder"){
		LOG(INFO) << "Setting trajectory_builder.lua" ;
		code_traj = code;
		return;
	}

	LOG(FATAL) << "Check basenames in your lua import ";
}

}//namespace cartographer_generic

