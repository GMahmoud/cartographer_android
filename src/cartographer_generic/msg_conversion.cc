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

#include "msg_conversion.h"

#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer_generic_msgs/Pose.h"
#include "cartographer_generic_msgs/Quaternion.h"
//#include "cartographer_generic_msgs/Transform.h"
//#include "cartographer_generic_msgs/TransformStamped.h"
#include "cartographer_generic_msgs/Vector3.h"
//#include "glog/logging.h"
//#include "cartographer_generic_msgs/Imu.h"
#include "cartographer_generic_msgs/LaserScan.h"
//#include "cartographer_generic_msgs/MultiEchoLaserScan.h"
//#include "cartographer_generic_msgs/PointCloud2.h"

namespace cartographer_generic {

namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::cartographer::sensor::PointCloudWithIntensities;
using ::cartographer::transform::Rigid3d;

//cartographer_generic_msgs::PointCloud2 PreparePointCloud2Message(const int64 timestamp,
//                                                   const string& frame_id,
//                                                   const int num_points) {
//  cartographer_generic_msgs::PointCloud2 msg;
//  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
//  msg.header.frame_id = frame_id;
//  msg.height = 1;
//  msg.width = num_points;
//  msg.fields.resize(3);
//  msg.fields[0].name = "x";
//  msg.fields[0].offset = 0;
//  msg.fields[0].datatype = cartographer_generic_msgs::PointField::FLOAT32;
//  msg.fields[0].count = 1;
//  msg.fields[1].name = "y";
//  msg.fields[1].offset = 4;
//  msg.fields[1].datatype = cartographer_generic_msgs::PointField::FLOAT32;
//  msg.fields[1].count = 1;
//  msg.fields[2].name = "z";
//  msg.fields[2].offset = 8;
//  msg.fields[2].datatype = cartographer_generic_msgs::PointField::FLOAT32;
//  msg.fields[2].count = 1;
//  msg.is_bigendian = false;
//  msg.point_step = 16;
//  msg.row_step = 16 * msg.width;
//  msg.is_dense = true;
//  msg.data.resize(16 * num_points);
//  return msg;
//}

// For cartographer_generic_msgs::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

// For cartographer_generic_msgs::MultiEchoLaserScan.
//bool HasEcho(const cartographer_generic_msgs::LaserEcho& echo) {
//  return !echo.echoes.empty();
//}

//float GetFirstEcho(const cartographer_generic_msgs::LaserEcho& echo) {
//  return echo.echoes[0];
//}

// For cartographer_generic_msgs::LaserScan and cartographer_generic_msgs::MultiEchoLaserScan.
template <typename LaserMessageType>
PointCloudWithIntensities LaserScanToPointCloudWithIntensities(
    const LaserMessageType& msg) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        point_cloud.points.push_back(rotation *
                                     (first_echo * Eigen::Vector3f::UnitX()));
        if (msg.intensities.size() > 0) {
          //CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          const auto& echo_intensities = msg.intensities[i];
          //CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }
  return point_cloud;
}

//bool PointCloud2HasField(const cartographer_generic_msgs::PointCloud2& pc2,
//                         const std::string& field_name) {
//  for (const auto& field : pc2.fields) {
//    if (field.name == field_name) {
//      return true;
//    }
//  }
//  return false;
//}

}  // namespace

//cartographer_generic_msgs::PointCloud2 ToPointCloud2Message(
//    const int64 timestamp, const string& frame_id,
//    const ::cartographer::sensor::PointCloud& point_cloud) {
//  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
//  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
//  for (const auto& point : point_cloud) {
//    stream.next(point.x());
//    stream.next(point.y());
//    stream.next(point.z());
//    stream.next(kPointCloudComponentFourMagic);
//  }
//  return msg;
//}

PointCloudWithIntensities ToPointCloudWithIntensities(
    const ::cartographer_generic_msgs::LaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

//PointCloudWithIntensities ToPointCloudWithIntensities(
//    const cartographer_generic_msgs::MultiEchoLaserScan& msg) {
//  return LaserScanToPointCloudWithIntensities(msg);
//}

//PointCloudWithIntensities ToPointCloudWithIntensities(
//    const cartographer_generic_msgs::PointCloud2& message) {
//  PointCloudWithIntensities point_cloud;
//  // We check for intensity field here to avoid run-time warnings if we pass in
//  // a PointCloud2 without intensity.
//  if (PointCloud2HasField(message, "intensity")) {
//    pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
//    pcl::fromROSMsg(message, pcl_point_cloud);
//    for (const auto& point : pcl_point_cloud) {
//      point_cloud.points.emplace_back(point.x, point.y, point.z);
//      point_cloud.intensities.push_back(point.intensity);
//    }
//  } else {
//    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
//    pcl::fromROSMsg(message, pcl_point_cloud);
//
//    // If we don't have an intensity field, just copy XYZ and fill in
//    // 1.0.
//    for (const auto& point : pcl_point_cloud) {
//      point_cloud.points.emplace_back(point.x, point.y, point.z);
//      point_cloud.intensities.push_back(1.0);
//    }
//  }
//  return point_cloud;
//}

//Rigid3d ToRigid3d(const cartographer_generic_msgs::TransformStamped& transform) {
//  return Rigid3d(ToEigen(transform.transform.translation),
//                 ToEigen(transform.transform.rotation));
//}

Rigid3d ToRigid3d(const cartographer_generic_msgs::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const cartographer_generic_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const cartographer_generic_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

//cartographer_generic_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
//  cartographer_generic_msgs::Transform transform;
//  transform.translation.x = rigid3d.translation().x();
//  transform.translation.y = rigid3d.translation().y();
//  transform.translation.z = rigid3d.translation().z();
//  transform.rotation.w = rigid3d.rotation().w();
//  transform.rotation.x = rigid3d.rotation().x();
//  transform.rotation.y = rigid3d.rotation().y();
//  transform.rotation.z = rigid3d.rotation().z();
//  return transform;
//}

cartographer_generic_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
  cartographer_generic_msgs::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

cartographer_generic_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
  cartographer_generic_msgs::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

}  // namespace cartographer_generic
