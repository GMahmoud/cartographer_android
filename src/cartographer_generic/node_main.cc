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

#include <string>
#include <vector>
#include <android/log.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "../cartographer_generic_msgs/PointCloud2Iterator.h"
#include "cartographer_generic_msgs/PointCloud2.h"
#include "cartographer_generic_msgs/PointCloud.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_generic/node.h"
#include "cartographer_generic/node_options.h"
#include "cartographer_generic/buddy_lidar_2d.lua.h"
#include "cartographer_generic/trajectory_options.h"
#include "cartographer_generic_msgs/LaserScan.h"
#include "cartographer_generic_msgs/Odometry.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer_generic_msgs/SubmapQuery.h"
#include "cartographer_generic_msgs/SubmapList.h"
#include "cartographer_generic_msgs/MarkerArray.h"
#include "cartographer_generic_msgs/CameraInfo.h"

#if __cplusplus
extern "C" {
namespace cartographer_generic {
int counter = 0;
int counter_odom = 0;
int android_log_max_length = 1000;
//Current trajectory id
int trajectory_id = -1 ;
string configuration_directory = "/";
string configuration_basename = "buddy_lidar_2d.lua";

//LaserScan callback message
::cartographer_generic_msgs::LaserScan::Ptr laser_msg;
::cartographer_generic_msgs::Odometry::Ptr odom_msg;
::cartographer_generic_msgs::PointCloud2::Ptr pointcloud_msg;
::cartographer_generic_msgs::PointField::Ptr pointfield;
::cartographer_generic_msgs::CameraInfo::Ptr cameraInfo;
::cartographer_generic_msgs::RegionOfInterest::Ptr roi;

Eigen::Matrix3f K;
Eigen::Vector3f worldPoint;
Eigen::Vector3f planeCoeffs;
Eigen::Vector3f planePassingPoint;
Eigen::Vector4f plane;
Eigen::Isometry3f cameraTransfom;
Eigen::Isometry3f laserTransfom;

// The 720 value defines the angular resolution
int LASER_BUFFER_ELEMS=2047;
float laser_buffer[2047];

//Current submap query response
std::vector<::cartographer_generic_msgs::SubmapQuery::Response> responses;

double angle_min = -M_PI / 2.0;
double angle_max =  M_PI / 2.0;
double angle_increment =  M_PI / 360.0;
double scan_time = 1.0 / 30.0;
double range_min = 0.45;
double range_max = 6.0;
double max_height = -0.25;
double min_height = -0.5;

/*******************************************************************************
 * - Lua Configuration Load System
 * *****************************************************************************/
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions() {
	auto file_resolver = cartographer::common::make_unique<
			cartographer::common::ConfigurationFileResolver>(
					std::vector<string>{configuration_directory});
	const string code =file_resolver->GetFileContentOrDie(configuration_basename);
	cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
			code, std::move(file_resolver));
	LOG(INFO) << "LoadOptions() Ends";
	return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
			CreateTrajectoryOptions(&lua_parameter_dictionary));
}

//Pose format as [x,y,z] + quat[w,x,y,z]
void ComposePoses(double* pose, double* pose1 , double* pose2){
	double x1 = pose1[0];
	double y1 = pose1[1];
	double z1 = pose1[2];
	double qw1 = pose1[3];
	double qx1 = pose1[4];
	double qy1 = pose1[5];
	double qz1 = pose1[6];

	double x2 = pose2[0];
	double y2 = pose2[1];
	double z2 = pose2[2];
	double qw2 = pose2[3];
	double qx2 = pose2[4];
	double qy2 = pose2[5];
	double qz2 = pose2[6];

	pose[0] = x1 + x2 + 2*(-(qy1*qy1+qz1*qz1)*x2 + (qx1*qy1-qw1*qz1)*y2 + (qw1*qy1+qx1*qz1)*z2);
	pose[1] = y1 + y2 + 2*((qw1*qz1+qx1*qy1)*x2 - (qx1*qx1+qz1*qz1)*y2 + (qy1*qz1-qw1*qx1)*z2);
	pose[2] = z1 + z2 + 2*((qx1*qz1-qw1*qy1)*x2 + (qw1*qx1+qy1*qz1)*y2 - (qx1*qx1-qy1*qy1)*z2);

	pose[3] = qw1*qw2 - qx1*qx2 - qy1*qy2 - qz1*qz2;
	pose[4] = qw1*qx2 + qw2*qx1 + qy1*qz2 - qy2*qz1;
	pose[5] = qw1*qy2 + qw2*qy1 + qz1*qx2 - qz2*qx1;
	pose[6] = qw1*qz2 + qw2*qz1 + qx1*qy2 - qx2*qy1;
}


void _LoadLua(const char*basename, int size_basename, const char* code, int size_code ){
	string basename_str (basename, size_basename);
	string code_str (code, size_code);
	cartographer_generic::SetCode(basename_str,code_str);
}

/*******************************************************************************
 * - Init Cartographer
 ********************************************************************************/
Node* _Init() {

	constexpr double kTfBufferCacheTimeInSeconds = 1e6;

	//Init LaserScan msg constants
	laser_msg.reset(new cartographer_generic_msgs::LaserScan());
	laser_msg->ranges.reserve(360);
	laser_msg->header.seq = 1;
	laser_msg->header.frame_id = "buddy_tablet";
	laser_msg->angle_min=-1.57079637051;
	laser_msg->angle_max=1.57079637051;
	laser_msg->angle_increment=0.00872664619237;
	laser_msg->time_increment=0.0;
	laser_msg->scan_time=0.0333333350718;
	laser_msg->range_min=0.449999988079;
	laser_msg->range_max=6.0;

	//Init PointCloud2 msg constants
	pointcloud_msg.reset(new cartographer_generic_msgs::PointCloud2());
	pointcloud_msg->data.reserve(612864);
	pointcloud_msg->header.seq = 1;
	pointcloud_msg->header.frame_id = "buddy_tablet";
	pointcloud_msg->height=1;
	pointcloud_msg->width=38304;

	pointfield.reset(new cartographer_generic_msgs::PointField());
	pointfield->name = "x";
	pointfield->offset = 0;
	pointfield->datatype = 7;
	pointfield->count = 1;
	pointcloud_msg->fields.push_back(*pointfield);
	pointfield.reset(new cartographer_generic_msgs::PointField());
	pointfield->name = "y";
	pointfield->offset = 4;
	pointfield->datatype = 7;
	pointfield->count = 1;
	pointcloud_msg->fields.push_back(*pointfield);
	pointfield.reset(new cartographer_generic_msgs::PointField());
	pointfield->name = "z";
	pointfield->offset = 8;
	pointfield->datatype = 7;
	pointfield->count = 1;
	pointcloud_msg->fields.push_back(*pointfield);
	pointfield.reset(new cartographer_generic_msgs::PointField());
	pointfield->name = "rgb";
	pointfield->offset = 12;
	pointfield->datatype = 7;
	pointfield->count = 1;
	pointcloud_msg->fields.push_back(*pointfield);
	pointcloud_msg->is_bigendian = 0;
	pointcloud_msg->is_dense = 0;
	pointcloud_msg->point_step = 16;
	pointcloud_msg->row_step = 612864;
	//	laser_msg->ranges.reserve(1040);
	//	laser_msg->header.seq = 1;
	//	laser_msg->header.frame_id = "base_laser_link";
	//	laser_msg->angle_min=-2.26892805099;
	//	laser_msg->angle_max=2.26456475258;
	//	laser_msg->angle_increment=0.00436332309619;
	//	laser_msg->time_increment=1.73611115315e-05;
	//	laser_msg->scan_time=0.0500000007451;
	//	laser_msg->range_min=0.0230000000447;
	//	laser_msg->range_max=60.0;

	//Init Odometry msg constants
	odom_msg.reset(new cartographer_generic_msgs::Odometry());
	odom_msg->header.seq=1;
	odom_msg->header.frame_id = "world";
	odom_msg->child_frame_id = "buddy";

	//Init DepthImage msg constants
	cameraInfo.reset(new cartographer_generic_msgs::CameraInfo());
	cameraInfo->header.seq=1;
	cameraInfo->header.frame_id = "buddy_tablet";
	cameraInfo->height = 480;
	cameraInfo->width = 640;
	cameraInfo->distortion_model = "plumb_bob";
	double d_values[] = {0.0, 0.0, 0.0, 0.0, 0.0};
	for(int i=0; i<4; i++)
		cameraInfo->D.push_back(d_values[i]);
	double k_values[] = {570.3422241210938, 0.0, 314.5, 0.0, 570.3422241210938, 235.5, 0.0, 0.0, 1.0};
	for(int i=0; i<9; i++){
		cameraInfo->K[i] = k_values[i];
		int r = i % 3;
		int q = i / 3;
		K(q,r) = k_values[i];
		LOG(INFO) << "K(" << q << "," << r << ") = " << K(q,r);
	}

	double r_values[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	for(int i=0; i<9; i++)
		cameraInfo->R[i] = r_values[i];
	double p_values[] = {570.3422241210938, 0.0, 314.5, 0.0, 0.0, 570.3422241210938, 235.5, 0.0, 0.0, 0.0, 1.0, 0.0};
	for(int i=0; i<12; i++)
		cameraInfo->P[i] = p_values[i];
	cameraInfo->binning_x = 0;
	cameraInfo->binning_y = 0;
	roi.reset(new cartographer_generic_msgs::RegionOfInterest());
	roi->x_offset = 0;
	roi-> y_offset = 0;
	roi-> height = 0;
	roi-> width = 0;
	roi-> do_rectify = false;
	cameraInfo->roi = *roi;

	//TODO
	//  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
	//  tf2_ros::TransformListener tf(tf_buffer);

	NodeOptions node_options;
	TrajectoryOptions trajectory_options;
	std::tie(node_options, trajectory_options) = LoadOptions();

	//TODO FCA-MGO : check tf_buffer use !
	Node* node = new Node(node_options/*, &tf_buffer*/);
	//TODO If we want to add a primary map
	//	if (!FLAGS_map_filename.empty()) {
	//		node.LoadMap(FLAGS_map_filename);
	//	}
	trajectory_id = node->StartTrajectory(trajectory_options);
	return node;
}

void _Stop (Node* node) {
	LOG(INFO) << "_Stop(Node*) Begins" ;
	node->FinishAllTrajectories();
	//TODO FCA-MGO : this must be done sometime ! Check wether it should be here
	//or somewhere else (FinishAllTrajectories use ?)
	//delete node;
	//node = nullptr;
	LOG(INFO) << "_Stop(Node*) Ends" ;
}

/*******************************************************************************
 * - Main callback handler : LaserScan, Odometry, IMU...
 ********************************************************************************/
void _LaserScanCallback(Node* node, float* ranges, int64 time) {
	laser_msg->ranges.clear(); //clear does not resize
	counter ++;
	std::stringstream ss;
	ss << "Scan ----------------------------------------------------------------  " << counter << "\n" ;
	ss << "Scan: [" ;
	for(int i=0; i<360; i++){ //1° definition - LaserScan data
		if(ranges[i] == -1)  laser_msg->ranges.push_back(std::numeric_limits<float>::infinity()) ;
		else laser_msg->ranges.push_back(ranges[i]) ;
		ss << laser_msg->ranges[i] << ", ";
	}
	ss << "], Time: " << time;
	LOG(INFO) << ss.str();
	//Convert time ticks in common time
	laser_msg->header.stamp = ::cartographer::common::FromUniversal(time);
	node->LaserScanCallback(laser_msg, trajectory_id);
	//Increment header sequence
	laser_msg->header.seq++;
}

/*******************************************************************************
 * - Expecting pose, twist and covariance in the following format :
 * - pose as [x,y,z] + quat[x,y,z,w] and covariance pose [36]
 * - twist as [x,y,z] + [x,y,z] and covariance twist [36]
 ********************************************************************************/
void _OdometryCallback(Node* node, float* pose, float* pose_covariance, float* twist, float* twist_covariance, int64 time) {

	counter_odom ++;
	LOG(INFO) << "Odom ----------------------------------------- " << counter_odom << " " << time ;

	odom_msg->pose.pose.position.x = pose[0];
	odom_msg->pose.pose.position.y = pose[1];
	odom_msg->pose.pose.position.z = pose[2];
	odom_msg->pose.pose.orientation.x = pose[3];
	odom_msg->pose.pose.orientation.y = pose[4];
	odom_msg->pose.pose.orientation.z = pose[5];
	odom_msg->pose.pose.orientation.w = pose[6];

	odom_msg->twist.twist.linear.x = twist[0];
	odom_msg->twist.twist.linear.y = twist[1];
	odom_msg->twist.twist.linear.z = twist[2];
	odom_msg->twist.twist.angular.x = twist[3];
	odom_msg->twist.twist.angular.y = twist[4];
	odom_msg->twist.twist.angular.z = twist[5];

	for(int i=0 ; i<36 ; i++){
		odom_msg->pose.covariance[i]=pose_covariance[i];
		odom_msg->twist.covariance[i]=twist_covariance[i];
	}

	odom_msg->header.stamp = ::cartographer::common::FromUniversal(time);

	node->OdometryCallback(odom_msg, trajectory_id);

	odom_msg->header.seq++;
}

void _PointCloudCallback(Node* node, int* data, int64 time)
{
	bool use_inf = true;

	for(int i=0; i<612864; i++){ //1° definition - PointCloud data
		pointcloud_msg->data.push_back(data[i]) ;
	}

	pointcloud_msg->header.stamp = ::cartographer::common::FromUniversal(time);
	//Increment header sequence
	pointcloud_msg->header.seq++;

	//build laserscan scan_output
	cartographer_generic_msgs::LaserScan::Ptr scan_output ;
	scan_output.reset(new cartographer_generic_msgs::LaserScan());
	scan_output->header = pointcloud_msg->header;
	scan_output->angle_min = angle_min;
	scan_output->angle_max = angle_max;
	scan_output->angle_increment = angle_increment;
	scan_output->time_increment = 0.0;
	scan_output->scan_time = scan_time;
	scan_output->range_min = range_min;
	scan_output->range_max = range_max;

	LOG(INFO) << "angle_min =" << angle_min << "angle_max =" << angle_max << "angle_increment =" << angle_increment << "time_increment =" << 0.0 << "scan_time =" << scan_time
			<< "range_min =" << range_min
			<< "range_max =" << range_max;

	//determine amount of rays to create
	uint32_t ranges_size = std::ceil((scan_output->angle_max - scan_output->angle_min) / scan_output->angle_increment);

	//determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
	if (use_inf){
		scan_output->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
	}
	else{
		scan_output->ranges.assign(ranges_size, scan_output->range_max + 1.0);
	}

	std::stringstream ss;
	ss << "Scan from point cloud ----------------------------------------------------------------  \n" ;
	ss << "Scan: [" ;
	// Iterate through pointcloud
	for (::cartographer_generic_msgs::PointCloud2Iterator<float>
			iter_x(*pointcloud_msg, "x"), iter_y(*pointcloud_msg, "y"), iter_z(*pointcloud_msg, "z");
			iter_x != iter_x.end();
			++iter_x, ++iter_y, ++iter_z){

		if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)){
			//LOG(INFO) << "rejected for nan in point(" << *iter_x << "," <<  *iter_y << "," <<  *iter_z << ")\n";
			continue;
		}
		if (*iter_z > max_height || *iter_z < min_height){
			//LOG(INFO) << "rejected for height " << *iter_z << " not in range (" << min_height << "," <<  max_height << ")\n" ;
			*iter_x = 0.0;
			*iter_y = 0.0;
			*iter_z = 0.0;
			continue;
		}

		double range = hypot(*iter_x, *iter_y);
		if (range < range_min){
			//LOG(INFO) << "rejected for range " << range <<" below minimum value " << range_min <<". Point: (" << *iter_x << "," << *iter_y << "," << *iter_z << ")";
			continue;
		}

		double angle = atan2(*iter_y, *iter_x);
		if (angle < scan_output->angle_min || angle > scan_output->angle_max){
			//LOG(INFO) << "rejected for angle " << angle << " not in range (" << scan_output->angle_min << "," <<  scan_output->angle_max << ")\n" ;
			continue;
		}

		//overwrite range at laserscan ray if new range is smaller
		int index = (angle - scan_output->angle_min) / scan_output->angle_increment;
		if (range < scan_output->ranges[index]){
			scan_output->ranges[index] = range;
		}

	}
	for(int i=0; i < 360; i++){
		ss << scan_output->ranges[i] << ", ";
	}
	ss <<  "], Time: " << time;
	int read=0;
	std::string substr;
	do {
		substr = ss.str().substr(read,android_log_max_length);
		LOG(INFO) << substr;
		read+=substr.length();
	} while(read<ss.str().length());

	node->LaserScanCallback(scan_output, trajectory_id);
}

bool convertPointCloudToPointCloud2 (const cartographer_generic_msgs::PointCloud &input, cartographer_generic_msgs::PointCloud2 &output)
{
	output.header = input.header;
	output.width  = input.points.size ();
	output.height = 1;
	output.fields.resize (3 + input.channels.size ());
	// Convert x/y/z to fields
	output.fields[0].name = "x"; output.fields[1].name = "y"; output.fields[2].name = "z";
	int offset = 0;
	// All offsets are *4, as all field data types are float32
	for (size_t d = 0; d < output.fields.size (); ++d, offset += 4)
	{
		output.fields[d].offset = offset;
		output.fields[d].datatype = cartographer_generic_msgs::PointField::FLOAT32;
		output.fields[d].count  = 1;
	}
	output.point_step = offset;
	output.row_step   = output.point_step * output.width;
	// Convert the remaining of the channels to fields
	for (size_t d = 0; d < input.channels.size (); ++d)
		output.fields[3 + d].name = input.channels[d].name;
	output.data.resize (input.points.size () * output.point_step);
	output.is_bigendian = false;  // @todo ?
	output.is_dense     = false;

	// Copy the data points
	for (size_t cp = 0; cp < input.points.size (); ++cp)
	{
		memcpy (&output.data[cp * output.point_step + output.fields[0].offset], &input.points[cp].x, sizeof (float));
		memcpy (&output.data[cp * output.point_step + output.fields[1].offset], &input.points[cp].y, sizeof (float));
		memcpy (&output.data[cp * output.point_step + output.fields[2].offset], &input.points[cp].z, sizeof (float));
		for (size_t d = 0; d < input.channels.size (); ++d)
		{
			if (input.channels[d].values.size() == input.points.size())
			{
				memcpy (&output.data[cp * output.point_step + output.fields[3 + d].offset], &input.channels[d].values[cp], sizeof (float));
			}
		}
	}
	return (true);
}

void _DepthImageCallback(Node* node, short* data, int64 time, float* ranges){
	LOG(INFO) << "HERE ";
	//****************************************** Transform
	cameraTransfom.setIdentity();
	//TODO cameraTransfom.translation() << xtion.getOrigin().x(),xtion.getOrigin().y(),xtion.getOrigin().z();
	cameraTransfom.translation() << 0.0,0.0,0.0;
	cameraTransfom.translation()*=1000.0f;//mm
	cameraTransfom.translation()<<0,0,0;
	laserTransfom.setIdentity();
	//TODO laserTransfom.translation() << xtion2laser.getOrigin().x(),xtion2laser.getOrigin().y(),xtion2laser.getOrigin().z();
	//Eigen::Quaternionf l(xtion2laser.getRotation().getW(),xtion2laser.getRotation().getX(),xtion2laser.getRotation().getY(),xtion2laser.getRotation().getZ());
	//laserTransfom.linear()=l.toRotationMatrix();
	laserTransfom.translation() << 0.0,0.0,0.0;
	Eigen::Quaternionf l(1.0,0.0,0.0,0.0);
	laserTransfom.linear()=l.toRotationMatrix();

	//bringin the pointcloud into the XTION frame (not the optical one!)
	Eigen::AngleAxisf yawAngle(0, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf pitchAngle(M_PI/2, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rollAngle(-M_PI/2, Eigen::Vector3f::UnitX());
	Eigen::Quaternionf q = rollAngle * pitchAngle * yawAngle;
	cameraTransfom.linear() = q.toRotationMatrix();

	planeCoeffs<<0.0f,0.0f,1.0f;
	Eigen::Isometry3f rotator;
	rotator.setIdentity();
	//	Eigen::Quaternionf t(xtion2laser.getRotation().getW(),
	//			xtion2laser.getRotation().getX(),
	//			xtion2laser.getRotation().getY(),
	//			xtion2laser.getRotation().getZ());
	//	rotator.linear() = t.toRotationMatrix();
	Eigen::Quaternionf t(1.0, 0.0, 0.0, 0.0);
	rotator.linear() = t.toRotationMatrix();
	//transforming the Z unit vector using the frame transform.
	rotator.setIdentity();
	planeCoeffs=rotator*planeCoeffs;
	//std::cout<<planeCoeffs.transpose()<<std::endl;
	planePassingPoint.setZero();
	//planePassingPoint<<xtion2laser.getOrigin().x(),xtion2laser.getOrigin().y(),xtion2laser.getOrigin().z();
	planePassingPoint<<0.0,0.0,0.0;
	//std::cout<<"POINT "<<planePassingPoint<<std::endl;
	float d = -(planePassingPoint(0)*planeCoeffs(0)+
			planePassingPoint(1)*planeCoeffs(1)+
			planePassingPoint(2)*planeCoeffs(2));
	plane<<planeCoeffs(0),planeCoeffs(1),planeCoeffs(2),d;
	//****************************************** Transform

	::cartographer_generic_msgs::PointCloud depth_to_cloud;
	::cartographer_generic_msgs::PointCloud depth_to_laser;
	::cartographer_generic_msgs::LaserScan depth_to_scan;


	//convert the image message to a standard opencv Mat.
	//NOTE: data is immutable, it's ok with that.
	depth_to_cloud.points.clear();
	depth_to_laser.points.clear();
	depth_to_scan.ranges.clear();
	int scan_points=0;
	float minx=0;
	float miny=0;

	float maxx=0;
	float maxy=0;
	cartographer_generic_msgs::Point32 point;
//	std::stringstream ss2;
//	ss2 << "K : " << K << std::endl;
//	ss2 << "xtionTransfom: " << cameraTransfom.matrix() << "\n";
//	ss2 << "laserTransfom: " << laserTransfom.matrix() << "\n";
//	ss2 << "plane: " << plane << "\n";
//	ss2 << "planeCoeffs: " << planeCoeffs << "\n";
//	LOG(INFO) << ss2.str();

	for(int i = 0;i<LASER_BUFFER_ELEMS;i++){
			laser_buffer[i]=INFINITY;
	}
	float x, y, teta, ro;
	int angle_bin;
	for(int i =cameraInfo->height/3;i<2*cameraInfo->height/3;++i){
		for(int j=0;j<cameraInfo->width;++j){
			if(data[cameraInfo->width*i+j]!=0){
				worldPoint<<j*data[cameraInfo->width*i+j],i*data[cameraInfo->width*i+j],data[cameraInfo->width*i+j];
				//LOG(INFO) << "worldPoint before = " << worldPoint;
				worldPoint = K.inverse()*worldPoint;
				worldPoint= cameraTransfom*worldPoint;
				//LOG(INFO) << "worldPoint after = " << worldPoint;
				worldPoint/=1000.0f;
//				point.x=worldPoint[0];
//				point.y=worldPoint[1];
//				point.z=worldPoint[2];
				//depth_to_cloud.points.push_back(point);

				//checking if the point lies on the requested plane.
				//if so, i'll add it to the laser pointcloud

				//worldPoint=laserTransfom.inverse()*worldPoint;
//				point.x=worldPoint[0];
//				point.y=worldPoint[1];
//				point.z=worldPoint[2];
				if(worldPoint[2]<=0.005f && worldPoint[2]>=-0.005f){
					scan_points++;
					point.x=worldPoint[0];
					point.y=worldPoint[1];
					point.z=worldPoint[2];
					if(worldPoint[1]<miny){
						miny=worldPoint[1];
						minx=worldPoint[0];
					}
					if(worldPoint[1]>maxy){
						maxy=worldPoint[1];
						maxx=worldPoint[0];
					}
					depth_to_laser.points.push_back(point);
					x = point.x;
					y = point.y;
					ro   = sqrt(pow(x,2)+pow(y,2));
					teta = atan(y/x);
					angle_bin = (teta*LASER_BUFFER_ELEMS/(2*M_PI))+(LASER_BUFFER_ELEMS/2);
					laser_buffer[angle_bin]=ro;
				}
			}
		}

	}
	//SETTING TIMINGS
	depth_to_cloud.header.stamp = ::cartographer::common::FromUniversal(time);
	depth_to_laser.header.stamp = ::cartographer::common::FromUniversal(time);
	depth_to_scan.header.stamp = ::cartographer::common::FromUniversal(time);
	depth_to_scan.angle_min=-M_PI;
	depth_to_scan.angle_max=M_PI;
	depth_to_scan.range_min=0;
	depth_to_scan.range_max=20.0f;
	depth_to_scan.angle_increment=2*M_PI/LASER_BUFFER_ELEMS;

//	for(int i=0;i<depth_to_laser.points.size();i++){
//		float x = depth_to_laser.points.at(i).x;
//		float y = depth_to_laser.points.at(i).y;
//		float ro   = sqrt(pow(x,2)+pow(y,2));
//		float teta = atan(y/x);
//		int angle_bin = (teta*LASER_BUFFER_ELEMS/(2*M_PI))+(LASER_BUFFER_ELEMS/2);
//		laser_buffer[angle_bin]=ro;
//
//	}
//	for(int i = 0;i<LASER_BUFFER_ELEMS;i++){
//			laser_buffer[i]=INFINITY;
//	}
	std::stringstream ss;
	ss << "Scan ----------------------------------------------------------------  " << counter << "\n" ;
	ss << "Scan: [" ;
	int j = 0;
	for(int i=0;i<LASER_BUFFER_ELEMS;i++){
		depth_to_scan.ranges.push_back(laser_buffer[i]);
		ranges[i] = (laser_buffer[i]==INFINITY) ? -1 : laser_buffer[i];
		if(ranges[i] == -1)  j++;
		else{
			if(j != 0){
				ss << "(inf x " << j <<")," << ranges[i] << ", ";
				j = 0;
			}
			else ss  << ranges[i] << ", ";
			j = 0;
		}
	}
	if(j != 0) ss << "(inf x " << j <<")" ;
	ss << "], Time: " << time ;

	LOG(INFO) << ss.str();

}
/*******************************************************************************
 * - Submap Query/Retrieve
 * HandleSubmapQuery must be call first.
 ********************************************************************************/
int _HandleSubmapQuery(Node* node){
	::cartographer_generic_msgs::SubmapList SubmapList = node->GetSubmapList();
	::cartographer_generic_msgs::SubmapQuery::Request request;
	::cartographer_generic_msgs::SubmapQuery::Response response;
	request.submap_index = SubmapList.submap.back().submap_index;
	LOG(INFO) << "request.submap_index = " << request.submap_index;
	request.trajectory_id = trajectory_id;
	response.cells.clear();
	node->HandleSubmapQuery(request,response);

	if(responses.size() < SubmapList.submap.size()){
		responses.push_back(response);
		LOG(INFO) << "Add new response " << request.submap_index << " to queries";
	}
	else{
		responses[request.submap_index] = response;
		LOG(INFO) << "Update response " << request.submap_index;
	}

	return request.submap_index;
}

void _UpdateSubmap(Node* node, int submap_index ){
	::cartographer_generic_msgs::SubmapList SubmapList = node->GetSubmapList();
	::cartographer_generic_msgs::SubmapQuery::Request request;
	::cartographer_generic_msgs::SubmapQuery::Response response;
	request.submap_index = submap_index;
	request.trajectory_id = trajectory_id;
	response.cells.clear();
	node->HandleSubmapQuery(request,response);
	responses[submap_index] = response;
	LOG(INFO) << "Last update response " << submap_index;
}

//Retrieve grid size (Unity should handle most of memory allocation)
// response.cells.size() = width * height * 2 (intensity and alpha channels)
void _GetGridSize(int* size, int submap_index){
	size[0] = responses[submap_index].cells.size()!=0 ? responses[submap_index].width : -1;
	size[1] = responses[submap_index].cells.size()!=0 ? responses[submap_index].height : -1;
}

double _GetGridResolution(int submap_index){
	return responses[submap_index].cells.size()!=0 ? responses[submap_index].resolution : -1;
}

//Pose format as [x,y,z] + quat[w,x,y,z]
void _GetSubmapPose(Node* node, double* pose, int submap_index)
{
	::cartographer_generic_msgs::SubmapList SubmapList = node->GetSubmapList();
	::cartographer_generic_msgs::SubmapEntry SubEnt = SubmapList.submap[submap_index];

	double slice_pose[7];
	double submap_pose[7];
	submap_pose[0] = SubEnt.pose.position.x ;
	submap_pose[1] = SubEnt.pose.position.y ;
	submap_pose[2] = SubEnt.pose.position.z ;
	submap_pose[3] = SubEnt.pose.orientation.w ;
	submap_pose[4] = SubEnt.pose.orientation.x ;
	submap_pose[5] = SubEnt.pose.orientation.y ;
	submap_pose[6] = SubEnt.pose.orientation.z ;

	slice_pose[0] =  responses[submap_index].slice_pose.position.x - responses[submap_index].height*responses[submap_index].resolution/2 ;
	slice_pose[1] =  responses[submap_index].slice_pose.position.y - responses[submap_index].width*responses[submap_index].resolution/2;
	slice_pose[2] =  responses[submap_index].slice_pose.position.z ;
	slice_pose[3] =  responses[submap_index].slice_pose.orientation.w ;
	slice_pose[4] =  responses[submap_index].slice_pose.orientation.x ;
	slice_pose[5] =  responses[submap_index].slice_pose.orientation.y ;
	slice_pose[6] =  responses[submap_index].slice_pose.orientation.z ;

	ComposePoses(pose, submap_pose, slice_pose);

}


//Retrieve occupancy grid
//intensity and alpha channels are converted form uint8 (char) to int
void _GetOccupancyGrid (int* intensity, int* alpha, int submap_index) {
	for (int i = 0; i < responses[submap_index].height; ++i) {
		for (int j = 0; j < responses[submap_index].width; ++j) {
			intensity[i*responses[submap_index].width + j] = static_cast<int>(responses[submap_index].cells[(i * responses[submap_index].width + j) * 2]);
			alpha[i*responses[submap_index].width + j] = static_cast<int>(responses[submap_index].cells[(i * responses[submap_index].width + j) * 2 + 1]);
		}
	}
}

/*******************************************************************************
 * - RobotPose format is [x,y,z] + quat[w,x,y,z]
 ********************************************************************************/
void _GetRobotPose (Node* node, int64 time, float* robot_pose){
	::cartographer::common::Time time_now = ::cartographer::common::FromUniversal(time);
	::cartographer_generic_msgs::MarkerArray TrajectoryList = node->GetTrajectoryNodeList(time_now);
	if (TrajectoryList.markers.size() == 0){
		robot_pose[0] = 0.0;
		robot_pose[1] = 0.0;
		robot_pose[2] = 0.0;
		robot_pose[3] = 1.0;
		robot_pose[4] = 0.0;
		robot_pose[5] = 0.0;
		robot_pose[6] = 0.0;
	}
	else{
		::cartographer_generic_msgs::Marker marker = TrajectoryList.markers.back();
		::cartographer_generic_msgs::Pose pose=marker.poses.back();
		robot_pose[0] = pose.position.x;
		robot_pose[1] = pose.position.y;
		robot_pose[2] = pose.position.z;
		robot_pose[3] = pose.orientation.w;
		robot_pose[4] = pose.orientation.x;
		robot_pose[5] = pose.orientation.y;
		robot_pose[6] = pose.orientation.z;
	}
}

//void _GetRobotPose (Node* node, float* robot_pose){
//	cartographer::transform::Rigid3d transform = node->GetTrajectoryStates();
//
//	robot_pose[0] = transform.translation().x();
//	robot_pose[1] = transform.translation().y();
//	robot_pose[2] = transform.translation().z();
//	robot_pose[3] = transform.rotation().w();
//	robot_pose[4] = transform.rotation().x();
//	robot_pose[5] = transform.rotation().y();
//	robot_pose[6] = transform.rotation().z();
//
//}

}  // namespace cartographer_generic
}  // extern "C"

#endif
