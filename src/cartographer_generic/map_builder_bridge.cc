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

#include "cartographer_generic/map_builder_bridge.h"

#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer_generic/msg_conversion.h"

namespace cartographer_generic {

namespace {

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kConstraintMarkerScale = 0.025;

}  // namespace

MapBuilderBridge::MapBuilderBridge(const NodeOptions& node_options)
    : node_options_(node_options),
      map_builder_(node_options.map_builder_options){}


int MapBuilderBridge::AddTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  const int trajectory_id = map_builder_.AddTrajectoryBuilder(expected_sensor_ids, trajectory_options.trajectory_builder_options);

  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          trajectory_options.num_subdivisions_per_laser_scan,
          trajectory_options.tracking_frame,
          node_options_.lookup_transform_timeout_sec, /*tf_buffer_,*/
          map_builder_.GetTrajectoryBuilder(trajectory_id));
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  //CHECK(emplace_result.second == true);
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  map_builder_.FinishTrajectory(trajectory_id);
  map_builder_.sparse_pose_graph()->RunFinalOptimization();
  sensor_bridges_.erase(trajectory_id);
}


SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

}  // namespace cartographer_generic
