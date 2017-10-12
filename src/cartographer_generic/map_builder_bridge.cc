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
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          trajectory_options.num_subdivisions_per_laser_scan,
          trajectory_options.tracking_frame,
          node_options_.lookup_transform_timeout_sec, /*tf_buffer_,*/
          map_builder_.GetTrajectoryBuilder(trajectory_id));

  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  map_builder_.FinishTrajectory(trajectory_id);
  map_builder_.sparse_pose_graph()->RunFinalOptimization();
  sensor_bridges_.erase(trajectory_id);
}

cartographer_generic_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_generic_msgs::SubmapList submap_list;
  //TODO
  //submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  const auto all_submap_data =
      map_builder_.sparse_pose_graph()->GetAllSubmapData();
  for (size_t trajectory_id = 0; trajectory_id < all_submap_data.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index < all_submap_data[trajectory_id].size(); ++submap_index) {
      const auto& submap_data = all_submap_data[trajectory_id][submap_index];
      if (submap_data.submap == nullptr) {
        continue;
      }
      cartographer_generic_msgs::SubmapEntry submap_entry;
      submap_entry.trajectory_id = trajectory_id;
      submap_entry.submap_index = submap_index;
      submap_entry.submap_version = submap_data.submap->num_range_data();
      submap_entry.pose = ToGeometryMsgPose(submap_data.pose);
      submap_list.submap.push_back(submap_entry);
    }
  }
  LOG(INFO) << "submap size " <<  submap_list.submap.size();
  return submap_list;
}

bool MapBuilderBridge::HandleSubmapQuery(
cartographer_generic_msgs::SubmapQuery::Request& request,
cartographer_generic_msgs::SubmapQuery::Response& response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  const std::string error = map_builder_.SubmapToProto(
      cartographer::mapping::SubmapId{request.trajectory_id,
                                      request.submap_index},
      &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    return false;
  }

  response.submap_version = response_proto.submap_version();
  response.cells.insert(response.cells.begin(), response_proto.cells().begin(),
                        response_proto.cells().end());
  response.width = response_proto.width();
  response.height = response_proto.height();
  response.resolution = response_proto.resolution();
  response.slice_pose = ToGeometryMsgPose(
      cartographer::transform::ToRigid3(response_proto.slice_pose()));
  return true;
}

std::unordered_map<int, MapBuilderBridge::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  std::unordered_map<int, TrajectoryState> trajectory_states;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    const cartographer::mapping::TrajectoryBuilder* const trajectory_builder =
        map_builder_.GetTrajectoryBuilder(trajectory_id);
    const cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate =
        trajectory_builder->pose_estimate();
    if (cartographer::common::ToUniversal(pose_estimate.time) < 0) {
      continue;
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
    trajectory_states[trajectory_id] = {
        pose_estimate,
        map_builder_.sparse_pose_graph()->GetLocalToGlobalTransform(
            trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(
            pose_estimate.time,
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};
  }
  return trajectory_states;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

cartographer_generic_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList(::cartographer::common::Time time) {
  cartographer_generic_msgs::MarkerArray trajectory_node_list;
  const auto all_trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  int marker_id = 0;
  for (int trajectory_id = 0;
       trajectory_id < static_cast<int>(all_trajectory_nodes.size());
       ++trajectory_id) {
    const auto& single_trajectory_nodes = all_trajectory_nodes[trajectory_id];
    cartographer_generic_msgs::Marker marker;
    marker.id = marker_id++;
    marker.header.stamp = time;
    marker.header.frame_id = node_options_.map_frame;
    marker.scale.x = kTrajectoryLineStripMarkerScale;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.z = 0.05;
    for (const auto& node : single_trajectory_nodes) {
      if (node.trimmed()) {
        continue;
      }
      // In 2D, the pose in node.pose is xy-aligned. Multiplying by
      // node.constant_data->tracking_to_pose would give the full orientation,
      // but that is not needed here since we are only interested in the
      // translational part.
      const ::cartographer_generic_msgs::Pose node_pose =
          ToGeometryMsgPose(node.pose);
      marker.poses.push_back(node_pose);
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      if (marker.poses.size() == 16384) {
        trajectory_node_list.markers.push_back(marker);
        marker.id = marker_id++;
        marker.poses.clear();
        // Push back the last point, so the two markers appear connected.
        marker.poses.push_back(node_pose);
      }
    }
    trajectory_node_list.markers.push_back(marker);
  }
  return trajectory_node_list;
}

}  // namespace cartographer_generic
