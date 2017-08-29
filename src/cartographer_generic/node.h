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

#ifndef CARTOGRAPHER_GENERIC_NODE_H_
#define CARTOGRAPHER_GENERIC_NODE_H_

#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/common/mutex.h"
//#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_generic/node_constants.h"
#include "cartographer_generic/node_options.h"
#include "cartographer_generic/trajectory_options.h"
#include "cartographer_generic_msgs/SensorTopics.h"
#include "cartographer_generic/map_builder_bridge.h"
#include "cartographer_generic_msgs/SubmapList.h"
#include "cartographer_generic_msgs/SubmapQuery.h"
#include "cartographer_generic_msgs/MarkerArray.h"



namespace cartographer_generic {

// Wires up ROS topics to SLAM.
class Node {
public:
	Node(const NodeOptions& node_options/*, tf2_ros::Buffer* tf_buffer*/);
	~Node();

	Node(const Node&) = delete;
	Node& operator=(const Node&) = delete;

	// Finishes all yet active trajectories.
	void FinishAllTrajectories();

	// Starts the first trajectory with the default topics.
	int StartTrajectory(const TrajectoryOptions& options);

	//Callbacks
	void LaserScanCallback (::cartographer_generic_msgs::LaserScan::Ptr& msg, const int trajectory_id );
	void OdometryCallback(::cartographer_generic_msgs::Odometry::Ptr& msg, const int trajectory_id );

	// Loads a persisted state to use as a map.
	//void LoadMap(const std::string& map_filename);

	MapBuilderBridge* map_builder_bridge();
	::cartographer_generic_msgs::SubmapList GetSubmapList();
	::cartographer_generic_msgs::MarkerArray GetTrajectoryNodeList(::cartographer::common::Time time);

	bool HandleSubmapQuery(
	    ::cartographer_generic_msgs::SubmapQuery::Request& request,
	    ::cartographer_generic_msgs::SubmapQuery::Response& response);


private:
	// Returns the set of topic names we want to subscribe to.
	std::unordered_set<std::string> ComputeExpectedTopics(
			const TrajectoryOptions& options,
			const cartographer_generic_msgs::SensorTopics& topics);
	int AddTrajectory(const TrajectoryOptions& options,
			const cartographer_generic_msgs::SensorTopics& topics);

	//	void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
	//	::cartographer::mapping::PoseExtrapolator* GetExtrapolator(int trajectory_id);
	//	void PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event);
	//	void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
	//	void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);

	const NodeOptions node_options_;

	cartographer::common::Mutex mutex_;
	MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

	//TODO Our outputs
	//  ::ros::Publisher submap_list_publisher_;
	//  ::ros::Publisher trajectory_node_list_publisher_;
	//  ::ros::Publisher constraint_list_publisher_;
	//  ::ros::Publisher scan_matched_point_cloud_publisher_;

	// These are keyed with 'trajectory_id'.
	//std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
	//std::unordered_map<int, std::vector<::ros::Subscriber>> subscribers_;
	//std::unordered_set<std::string> subscribed_topics_;
	std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);

	// We have to keep the timer handles of ::ros::WallTimers around, otherwise
	// they do not fire. TODO TO PUBLISH MSGS
	//std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_generic

#endif  // CARTOGRAPHER_GENERIC_NODE_H_
