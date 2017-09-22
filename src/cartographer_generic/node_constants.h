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

#ifndef CARTOGRAPHER_GENERIC_NODE_CONSTANTS_H_
#define CARTOGRAPHER_GENERIC_NODE_CONSTANTS_H_

#include <string>
#include <vector>

namespace cartographer_generic {

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kTrajectoryNodeListTopic[] = "trajectory_node_list";
constexpr char kConstraintListTopic[] = "constraint_list";
constexpr double kConstraintPublishPeriodSec = 0.5;
constexpr char kSubmapQueryServiceName[] = "submap_query";

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

static std::vector<double> pose_estimate;
// For multiple topics adds numbers to the topic name and returns the list.
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   int num_topics);

std::vector<double>  GetPoseEstimate();

void SetPoseEstimate(double* pose_ros);



}  // namespace cartographer_generic

#endif  // CARTOGRAPHER_GENERIC_NODE_CONSTANTS_H_
