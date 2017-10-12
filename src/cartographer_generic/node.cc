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

#include "cartographer_generic/node.h"
#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_generic/msg_conversion.h"
#include "cartographer_generic/sensor_bridge.h"
#include "cartographer_generic/tf_bridge.h"
//#include "time_conversion.h"
//#include "glog/logging.h"


namespace cartographer_generic {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Node::Node(const NodeOptions& node_options)
: node_options_(node_options),
  map_builder_bridge_(node_options_) {
	carto::common::MutexLocker lock(&mutex_);

	// TODO Outputs
//	  submap_list_publisher_ =
//	      node_handle_.advertise<::cartographer_generic_msgs::SubmapList>(
//	          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
//	  trajectory_node_list_publisher_ =
//	      node_handle_.advertise<::visualization_msgs::MarkerArray>(
//	          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
//	  constraint_list_publisher_ =
//	      node_handle_.advertise<::visualization_msgs::MarkerArray>(
//	          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
//	  scan_matched_point_cloud_publisher_ =
//	      node_handle_.advertise<sensor_msgs::PointCloud2>(
//	          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

//	  wall_timers_.push_back(node_handle_.createWallTimer(
//	      ::ros::WallDuration(node_options_.submap_publish_period_sec),
//	      &Node::PublishSubmapList, this));
//	  wall_timers_.push_back(node_handle_.createWallTimer(
//	      ::ros::WallDuration(node_options_.pose_publish_period_sec),
//	      &Node::PublishTrajectoryStates, this));
//	  wall_timers_.push_back(node_handle_.createWallTimer(
//	      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
//	      &Node::PublishTrajectoryNodeList, this));
//	  wall_timers_.push_back(node_handle_.createWallTimer(
//	      ::ros::WallDuration(kConstraintPublishPeriodSec),
//	      &Node::PublishConstraintList, this));

}

Node::~Node() {}


MapBuilderBridge* Node::map_builder_bridge() { return &map_builder_bridge_; }

::cartographer_generic_msgs::SubmapList Node::GetSubmapList() {
	carto::common::MutexLocker lock(&mutex_);
	::cartographer_generic_msgs::SubmapList SubmapList = map_builder_bridge_.GetSubmapList();
	return SubmapList;
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

cartographer_generic_msgs::MarkerArray Node::GetTrajectoryNodeList(::cartographer::common::Time time) {
	carto::common::MutexLocker lock(&mutex_);
	cartographer_generic_msgs::MarkerArray TrajectoryList = map_builder_bridge_.GetTrajectoryNodeList(time);
	return  TrajectoryList;
}

bool Node::HandleSubmapQuery(
    ::cartographer_generic_msgs::SubmapQuery::Request& request,
    ::cartographer_generic_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  return map_builder_bridge_.HandleSubmapQuery(request, response);
}

//::cartographer::mapping::PoseExtrapolator* Node::GetExtrapolator(
//		int trajectory_id) {
//	constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
//	if (extrapolators_.count(trajectory_id) == 0) {
//		extrapolators_.emplace(
//				std::piecewise_construct, std::forward_as_tuple(trajectory_id),
//				std::forward_as_tuple(::cartographer::common::FromSeconds(
//						kExtrapolationEstimationTimeSec)));
//	}
//	return &extrapolators_.at(trajectory_id);
//}


//cartographer::transform::Rigid3d Node::GetTrajectoryStates() {
//	carto::common::MutexLocker lock(&mutex_);
//	cartographer::transform::Rigid3d transform;
//	for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
//		const auto& trajectory_state = entry.second;
//
//		 auto& extrapolator = extrapolators_.at(entry.first);
//		// We only publish a point cloud if it has changed. It is not needed at high
//		// frequency, and republishing it would be computationally wasteful.
//		if (trajectory_state.pose_estimate.time != extrapolator.GetLastPoseTime()) {
//			extrapolator.AddPose(trajectory_state.pose_estimate.time,
//					trajectory_state.pose_estimate.pose);
//		transform = trajectory_state.pose_estimate.pose;
//		}
//
//	}
//	return transform;
//}

std::unordered_set<string> Node::ComputeExpectedTopics(
		const TrajectoryOptions& options,
		const cartographer_generic_msgs::SensorTopics& topics) {
	std::unordered_set<string> expected_topics;

	for (const string& topic : ComputeRepeatedTopicNames(
			topics.laser_scan_topic, options.num_laser_scans)) {
		expected_topics.insert(topic);
	}
	for (const string& topic :
			ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
					options.num_multi_echo_laser_scans)) {
		expected_topics.insert(topic);
	}
	for (const string& topic : ComputeRepeatedTopicNames(
			topics.point_cloud2_topic, options.num_point_clouds)) {
		expected_topics.insert(topic);
	}
	// For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
	// required.
	if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
			(node_options_.map_builder_options.use_trajectory_builder_2d() &&
					options.trajectory_builder_options.trajectory_builder_2d_options()
					.use_imu_data())) {
		expected_topics.insert(topics.imu_topic);
	}
	if (options.use_odometry) {
		expected_topics.insert(topics.odometry_topic);
	}
	return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options, const cartographer_generic_msgs::SensorTopics& topics) {
	const std::unordered_set<string> expected_sensor_ids = ComputeExpectedTopics(options, topics);
	const int trajectory_id = map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
	AddExtrapolator(trajectory_id, options);
	is_active_trajectory_[trajectory_id] = true;
	return trajectory_id;
}

void Node::LaserScanCallback(::cartographer_generic_msgs::LaserScan::Ptr& msg, const int trajectory_id){
	map_builder_bridge_.sensor_bridge(trajectory_id) ->HandleLaserScanMessage("scan", msg);
}

void Node::OdometryCallback( ::cartographer_generic_msgs::Odometry::Ptr& msg, const int trajectory_id ){
	  carto::common::MutexLocker lock(&mutex_);
	  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
	  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
	  if (odometry_data_ptr != nullptr) {
	    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
	  }
	  sensor_bridge_ptr->HandleOdometryMessage("odom", msg);
}


int Node::StartTrajectory(const TrajectoryOptions& options) {
	carto::common::MutexLocker lock(&mutex_);
	cartographer_generic_msgs::SensorTopics topics;
	topics.laser_scan_topic = kLaserScanTopic;
	topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
	topics.point_cloud2_topic = kPointCloud2Topic;
	topics.imu_topic = kImuTopic;
	topics.odometry_topic = kOdometryTopic;

	const int trajectory_id = AddTrajectory(options, topics);
	is_active_trajectory_[trajectory_id] = true;

	return trajectory_id;
}

void Node::FinishAllTrajectories() {
	carto::common::MutexLocker lock(&mutex_);
	for (const auto& entry : is_active_trajectory_) {
		const int trajectory_id = entry.first;
		if (entry.second) {
			map_builder_bridge_.FinishTrajectory(trajectory_id);
		}
	}
}

//void Node::LoadMap(const std::string& map_filename) {
//  map_builder_bridge_.LoadMap(map_filename);
//}

}  // namespace cartographer_generic
