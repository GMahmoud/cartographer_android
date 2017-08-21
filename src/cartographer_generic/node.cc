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

Node::Node(const NodeOptions& node_options/*, tf2_ros::Buffer* const tf_buffer*/)
: node_options_(node_options),
  map_builder_bridge_(node_options_/*, tf_buffer*/) {
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


bool Node::HandleSubmapQuery(
    ::cartographer_generic_msgs::SubmapQuery::Request& request,
    ::cartographer_generic_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  return map_builder_bridge_.HandleSubmapQuery(request, response);
}

//TODO OUTPUTS
//void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
//	carto::common::MutexLocker lock(&mutex_);
//	submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
//}
//
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
//
//void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
//	carto::common::MutexLocker lock(&mutex_);
//	for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
//		const auto& trajectory_state = entry.second;
//
//		auto& extrapolator = *GetExtrapolator(entry.first);
//		// We only publish a point cloud if it has changed. It is not needed at high
//		// frequency, and republishing it would be computationally wasteful.
//		if (trajectory_state.pose_estimate.time != extrapolator.GetLastPoseTime()) {
//			scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
//					carto::common::ToUniversal(trajectory_state.pose_estimate.time),
//					node_options_.map_frame,
//					carto::sensor::TransformPointCloud(
//							trajectory_state.pose_estimate.point_cloud,
//							trajectory_state.local_to_map.cast<float>())));
//			extrapolator.AddPose(trajectory_state.pose_estimate.time,
//					trajectory_state.pose_estimate.pose);
//		}
//
//		geometry_msgs::TransformStamped stamped_transform;
//		// If we do not publish a new point cloud, we still allow time of the
//		// published poses to advance. If we already know a newer pose, we use its
//		// time instead. Since tf knows how to interpolate, providing newer
//		// information is better.
//		const ::cartographer::common::Time now =
//				std::max(FromRos(ros::Time::now()), extrapolator.GetLastPoseTime());
//		stamped_transform.header.stamp = ToRos(now);
//		const Rigid3d tracking_to_local = extrapolator.ExtrapolatePose(now);
//		const Rigid3d tracking_to_map =
//				trajectory_state.local_to_map * tracking_to_local;
//
//		if (trajectory_state.published_to_tracking != nullptr) {
//			if (trajectory_state.trajectory_options.provide_odom_frame) {
//				std::vector<geometry_msgs::TransformStamped> stamped_transforms;
//
//				stamped_transform.header.frame_id = node_options_.map_frame;
//				stamped_transform.child_frame_id =
//						trajectory_state.trajectory_options.odom_frame;
//				stamped_transform.transform =
//						ToGeometryMsgTransform(trajectory_state.local_to_map);
//				stamped_transforms.push_back(stamped_transform);
//
//				stamped_transform.header.frame_id =
//						trajectory_state.trajectory_options.odom_frame;
//				stamped_transform.child_frame_id =
//						trajectory_state.trajectory_options.published_frame;
//				stamped_transform.transform = ToGeometryMsgTransform(
//						tracking_to_local * (*trajectory_state.published_to_tracking));
//				stamped_transforms.push_back(stamped_transform);
//
//				tf_broadcaster_.sendTransform(stamped_transforms);
//			} else {
//				stamped_transform.header.frame_id = node_options_.map_frame;
//				stamped_transform.child_frame_id =
//						trajectory_state.trajectory_options.published_frame;
//				stamped_transform.transform = ToGeometryMsgTransform(
//						tracking_to_map * (*trajectory_state.published_to_tracking));
//				tf_broadcaster_.sendTransform(stamped_transform);
//			}
//		}
//	}
//}
//
//void Node::PublishTrajectoryNodeList(
//		const ::ros::WallTimerEvent& unused_timer_event) {
//	carto::common::MutexLocker lock(&mutex_);
//	if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
//		trajectory_node_list_publisher_.publish(
//				map_builder_bridge_.GetTrajectoryNodeList());
//	}
//}
//
//void Node::PublishConstraintList(
//		const ::ros::WallTimerEvent& unused_timer_event) {
//	carto::common::MutexLocker lock(&mutex_);
//	if (constraint_list_publisher_.getNumSubscribers() > 0) {
//		constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
//	}
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
	//LaunchSubscribers(options, topics, trajectory_id);
	//subscribed_topics_.insert(expected_sensor_ids.begin(), expected_sensor_ids.end());
	return trajectory_id;
}

void Node::LaserScanCallback(::cartographer_generic_msgs::LaserScan::Ptr& msg, const int trajectory_id ){
//	map_builder_bridge_.sensor_bridge(trajectory_id) ->HandleLaserScanMessage(kLaserScanTopic, msg);
	map_builder_bridge_.sensor_bridge(trajectory_id) ->HandleLaserScanMessage("scan", msg);
}

void Node::OdometryCallback( ::cartographer_generic_msgs::Odometry::Ptr& msg, const int trajectory_id ){
//	map_builder_bridge_.sensor_bridge(trajectory_id) ->HandleOdometryMessage(kOdometryTopic, msg);
	map_builder_bridge_.sensor_bridge(trajectory_id) ->HandleOdometryMessage("odom", msg);
}


//TODO INPUTS
//void Node::LaunchSubscribers(const TrajectoryOptions& options,
//		const cartographer_generic_msgs::SensorTopics& topics,
//		const int trajectory_id) {
//	for (const string& topic : ComputeRepeatedTopicNames(
//			topics.laser_scan_topic, options.num_laser_scans)) {
//		subscribers_[trajectory_id].push_back(
//				node_handle_.subscribe<sensor_msgs::LaserScan>(
//						topic, kInfiniteSubscriberQueueSize,
//						boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
//								[this, trajectory_id,
//								 topic](const sensor_msgs::LaserScan::ConstPtr& msg) {
//			map_builder_bridge_.sensor_bridge(trajectory_id)
//                    						  ->HandleLaserScanMessage(topic, msg);
//		})));
//	}
//	//	for (const string& topic :
//	//			ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
//	//					options.num_multi_echo_laser_scans)) {
//	//		subscribers_[trajectory_id].push_back(node_handle_.subscribe<
//	//				sensor_msgs::MultiEchoLaserScan>(
//	//						topic, kInfiniteSubscriberQueueSize,
//	//						boost::function<void(const sensor_msgs::MultiEchoLaserScan::ConstPtr&)>(
//	//								[this, trajectory_id,
//	//								 topic](const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
//	//			map_builder_bridge_.sensor_bridge(trajectory_id)
//	//                		  ->HandleMultiEchoLaserScanMessage(topic, msg);
//	//		})));
//	//	}
//	//	for (const string& topic : ComputeRepeatedTopicNames(
//	//			topics.point_cloud2_topic, options.num_point_clouds)) {
//	//		subscribers_[trajectory_id].push_back(node_handle_.subscribe(
//	//				topic, kInfiniteSubscriberQueueSize,
//	//				boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
//	//						[this, trajectory_id,
//	//						 topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
//	//			map_builder_bridge_.sensor_bridge(trajectory_id)
//	//                		  ->HandlePointCloud2Message(topic, msg);
//	//		})));
//	//	}
//
//	// For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
//	// required.
//	//	if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
//	//			(node_options_.map_builder_options.use_trajectory_builder_2d() &&
//	//					options.trajectory_builder_options.trajectory_builder_2d_options()
//	//					.use_imu_data())) {
//	//		string topic = topics.imu_topic;
//	//		subscribers_[trajectory_id].push_back(
//	//				node_handle_.subscribe<sensor_msgs::Imu>(
//	//						topic, kInfiniteSubscriberQueueSize,
//	//						boost::function<void(const sensor_msgs::Imu::ConstPtr&)>(
//	//								[this, trajectory_id,
//	//								 topic](const sensor_msgs::Imu::ConstPtr& msg) {
//	//			auto sensor_bridge_ptr =
//	//					map_builder_bridge_.sensor_bridge(trajectory_id);
//	//			sensor_bridge_ptr->HandleImuMessage(topic, msg);
//	//			auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
//	//			if (imu_data_ptr != nullptr) {
//	//				GetExtrapolator(trajectory_id)->AddImuData(*imu_data_ptr);
//	//			}
//	//		})));
//	//	}
//
//	if (options.use_odometry) {
//		string topic = topics.odometry_topic;
//		subscribers_[trajectory_id].push_back(
//				node_handle_.subscribe<nav_msgs::Odometry>(
//						topic, kInfiniteSubscriberQueueSize,
//						boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
//								[this, trajectory_id,
//								 topic](const nav_msgs::Odometry::ConstPtr& msg) {
//			map_builder_bridge_.sensor_bridge(trajectory_id)
//                    						  ->HandleOdometryMessage(topic, msg);
//		})));
//	}
//
//	is_active_trajectory_[trajectory_id] = true;
//}


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