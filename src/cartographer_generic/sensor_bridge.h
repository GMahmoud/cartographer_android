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

#ifndef CARTOGRAPHER_GENERIC_SENSOR_BRIDGE_H_
#define CARTOGRAPHER_GENERIC_SENSOR_BRIDGE_H_

#include <memory>

#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_generic/tf_bridge.h"
#include "cartographer_generic_msgs/LaserScan.h"
#include "cartographer_generic_msgs/Odometry.h"

namespace cartographer_generic {

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge {
public:
	explicit SensorBridge(
			int num_subdivisions_per_laser_scan, const string& tracking_frame,
			double lookup_transform_timeout_sec, /*tf2_ros::Buffer* tf_buffer,*/
			::cartographer::mapping::TrajectoryBuilder* trajectory_builder);

	SensorBridge(const SensorBridge&) = delete;
	SensorBridge& operator=(const SensorBridge&) = delete;

	 std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(
	      ::cartographer_generic_msgs::Odometry::Ptr& msg);
	void HandleOdometryMessage(const string& sensor_id,
			 ::cartographer_generic_msgs::Odometry::Ptr& msg);
//	std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(
//			const sensor_msgs::Imu::Ptr& msg);
	//	void HandleImuMessage(const string& sensor_id,
	//			const sensor_msgs::Imu::Ptr& msg);
	void HandleLaserScanMessage(const string& sensor_id,
			::cartographer_generic_msgs::LaserScan::Ptr& msg);
	//  void HandleMultiEchoLaserScanMessage(
	//      const string& sensor_id,
	//      const sensor_msgs::MultiEchoLaserScan::Ptr& msg);
	//  void HandlePointCloud2Message(const string& sensor_id,
	//                                const sensor_msgs::PointCloud2::Ptr& msg);

	const TfBridge& tf_bridge() const;

private:
	void HandleLaserScan(const string& sensor_id,
			::cartographer::common::Time start_time,
			 const string& frame_id,
			 const ::cartographer::sensor::PointCloud& points,
			 double seconds_between_points);
	void HandleRangefinder(const string& sensor_id,
			::cartographer::common::Time time,
			 const string& frame_id,
			 const ::cartographer::sensor::PointCloud& ranges);

	const int num_subdivisions_per_laser_scan_;
	const TfBridge tf_bridge_;
	::cartographer::mapping::TrajectoryBuilder* const trajectory_builder_;
};

}  // namespace cartographer_generic

#endif  // CARTOGRAPHER_GENERIC_SENSOR_BRIDGE_H_
