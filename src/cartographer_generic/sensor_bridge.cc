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

#include "cartographer_generic/sensor_bridge.h"
#include "cartographer/common/make_unique.h"
#include "cartographer_generic/msg_conversion.h"


namespace cartographer_generic {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const string& CheckNoLeadingSlash(const string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. ";
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan, const string& tracking_frame,
    const double lookup_transform_timeout_sec,/* tf2_ros::Buffer* const tf_buffer,*/
    carto::mapping::TrajectoryBuilder* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec/*, tf_buffer*/),
      trajectory_builder_(trajectory_builder) {}

std::unique_ptr<::cartographer::sensor::OdometryData>
SensorBridge::ToOdometryData(::cartographer_generic_msgs::Odometry::Ptr& msg) {
  const carto::common::Time time = msg->header.stamp;
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  return ::cartographer::common::make_unique<
      ::cartographer::sensor::OdometryData>(
      ::cartographer::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

void SensorBridge::HandleOdometryMessage(
    const string& sensor_id, ::cartographer_generic_msgs::Odometry::Ptr& msg) {
  const carto::common::Time time = msg->header.stamp;
 /* const auto sensor_to_tracking = tf_bridge_.LookupToTracking( time, CheckNoLeadingSlash(msg->child_frame_id));*/
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking( time, msg->child_frame_id);
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddOdometerData(
        sensor_id, time,
        ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse());
  }
}

//std::unique_ptr<::cartographer::sensor::ImuData> SensorBridge::ToImuData(
//    const sensor_msgs::Imu::Ptr& msg) {
//  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
//      << "Your IMU data claims to not contain linear acceleration measurements "
//         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
//         "requires this data to work. See "
//         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
//  CHECK_NE(msg->angular_velocity_covariance[0], -1)
//      << "Your IMU data claims to not contain angular velocity measurements "
//         "by setting angular_velocity_covariance[0] to -1. Cartographer "
//         "requires this data to work. See "
//         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
//
//  const carto::common::Time time = FromRos(msg->header.stamp);
//  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
//      time, CheckNoLeadingSlash(msg->header.frame_id));
//  if (sensor_to_tracking == nullptr) {
//    return nullptr;
//  }
//  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
//      << "The IMU frame must be colocated with the tracking frame. "
//         "Transforming linear acceleration into the tracking frame will "
//         "otherwise be imprecise.";
//  return ::cartographer::common::make_unique<::cartographer::sensor::ImuData>(
//      ::cartographer::sensor::ImuData{
//          time,
//          sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
//          sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
//}

//void SensorBridge::HandleImuMessage(const string& sensor_id,
//                                    const sensor_msgs::Imu::Ptr& msg) {
//  std::unique_ptr<::cartographer::sensor::ImuData> imu_data = ToImuData(msg);
//  if (imu_data != nullptr) {
//    trajectory_builder_->AddImuData(sensor_id, imu_data->time,
//                                    imu_data->linear_acceleration,
//                                    imu_data->angular_velocity);
//  }
//}

void SensorBridge::HandleLaserScanMessage(const string& sensor_id, ::cartographer_generic_msgs::LaserScan::Ptr& msg) {
    HandleLaserScan(sensor_id, msg->header.stamp, msg->header.frame_id, ToPointCloudWithIntensities(*msg).points, msg->time_increment);
}

//void SensorBridge::HandleMultiEchoLaserScanMessage(
//    const string& sensor_id,
//    const sensor_msgs::MultiEchoLaserScan::Ptr& msg) {
//  HandleLaserScan(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
//                  ToPointCloudWithIntensities(*msg).points,
//                  msg->time_increment);
//}
//
//void SensorBridge::HandlePointCloud2Message(
//    const string& sensor_id, const sensor_msgs::PointCloud2::Ptr& msg) {
//  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
//  pcl::fromROSMsg(*msg, pcl_point_cloud);
//  carto::sensor::PointCloud point_cloud;
//  for (const auto& point : pcl_point_cloud) {
//    point_cloud.emplace_back(point.x, point.y, point.z);
//  }
//  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
//                    point_cloud);
//}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

void SensorBridge::HandleLaserScan(const string& sensor_id,
                                   const carto::common::Time start_time,
                                   const string& frame_id,
                                   const carto::sensor::PointCloud& points,
                                   const double seconds_between_points
								   ) {
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    const carto::sensor::PointCloud subdivision(points.begin() + start_index,
                                                points.begin() + end_index);
    const carto::common::Time subdivision_time =
        start_time + carto::common::FromSeconds((start_index + end_index) / 2. *
                                                seconds_between_points);
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

void SensorBridge::HandleRangefinder(const string& sensor_id,
                                     const carto::common::Time time,
                                     const string& frame_id,
                                     const carto::sensor::PointCloud& ranges) {
  /*const auto sensor_to_tracking = tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));*/
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(time, frame_id);
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddRangefinderData(
        sensor_id, time, sensor_to_tracking->translation().cast<float>(),
        carto::sensor::TransformPointCloud(ranges,
                                           sensor_to_tracking->cast<float>()));
  }
}

}  // namespace cartographer_generic
