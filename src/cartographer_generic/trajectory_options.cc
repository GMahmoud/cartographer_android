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

#include "cartographer_generic/trajectory_options.h"
#include "glog/logging.h"

namespace cartographer_generic {

namespace {

void CheckTrajectoryOptions(const TrajectoryOptions& options) {
  CHECK_GE(options.num_subdivisions_per_laser_scan, 1);
  CHECK_GE(options.num_laser_scans + options.num_multi_echo_laser_scans +
               options.num_point_clouds,
           1)
      << "Configuration error: 'num_laser_scans', "
         "'num_multi_echo_laser_scans' and 'num_point_clouds' are "
         "all zero, but at least one is required.";
}

}  // namespace

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  TrajectoryOptions options;
  options.trajectory_builder_options =
      ::cartographer::mapping::CreateTrajectoryBuilderOptions(
          lua_parameter_dictionary->GetDictionary("trajectory_builder").get());
  options.tracking_frame =
      lua_parameter_dictionary->GetString("tracking_frame");
  options.published_frame =
      lua_parameter_dictionary->GetString("published_frame");
  options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
  options.provide_odom_frame =
      lua_parameter_dictionary->GetBool("provide_odom_frame");
  options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
  options.num_laser_scans =
      lua_parameter_dictionary->GetNonNegativeInt("num_laser_scans");
  options.num_multi_echo_laser_scans =
      lua_parameter_dictionary->GetNonNegativeInt("num_multi_echo_laser_scans");
  options.num_subdivisions_per_laser_scan =
      lua_parameter_dictionary->GetNonNegativeInt(
          "num_subdivisions_per_laser_scan");
  options.num_point_clouds = lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");
  CheckTrajectoryOptions(options);
  return options;
}

}  // namespace cartographer_generic
