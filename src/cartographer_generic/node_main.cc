#include <string>
#include <vector>
#include <android/log.h>
#include <glog/logging.h>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
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

//Current submap query response
::cartographer_generic_msgs::SubmapQuery::Response response;
::cartographer_generic_msgs::SubmapQuery::Response responseZero;

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
void _LaserScanCallback(Node* node, float* ranges, int64 time, double* pose_ros) {
	laser_msg->ranges.clear(); //clear does not resize
	counter ++;
	//if(counter > 10) LOG(FATAL)<< "STOP";
	std::stringstream ss;
	ss << "Scan ----------------------------------------------------------------  " << counter << "\n" ;
	ss << "Scan: [" ;
	for(int i=0; i<360; i++){ //1° definition - LaserScan data
		if(ranges[i] == 80)  laser_msg->ranges.push_back(std::numeric_limits<float>::infinity()) ;
		else laser_msg->ranges.push_back(ranges[i]) ;
		ss << laser_msg->ranges[i] << ", ";
	}
	ss << "], Time: " << time;
	LOG(INFO) << ss.str();
	//Convert time ticks in common time
	laser_msg->header.stamp = ::cartographer::common::FromUniversal(time);
	//cartographer_generic::SetPoseEstimate(pose_ros);
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

/*******************************************************************************
 * - Submap Query/Retrieve
 * HandleSubmapQuery must be call first.
 ********************************************************************************/
int _HandleSubmapQuery(Node* node){
	::cartographer_generic_msgs::SubmapList SubmapList = node->GetSubmapList();
	::cartographer_generic_msgs::SubmapQuery::Request request;
	request.submap_index = SubmapList.submap.back().submap_index;
	request.trajectory_id = trajectory_id;
	response.cells.clear();
	node->HandleSubmapQuery(request,response);
	return request.submap_index;
}

int _HandleSubmapQueryZero(Node* node){
	::cartographer_generic_msgs::SubmapList SubmapList = node->GetSubmapList();
	::cartographer_generic_msgs::SubmapQuery::Request request;
	request.submap_index = 0;
	request.trajectory_id = trajectory_id;
	responseZero.cells.clear();
	node->HandleSubmapQuery(request,responseZero);
	return request.submap_index;
}

//Retrieve grid size (Unity should handle most of memory allocation)
// response.cells.size() = width * height * 2 (intensity and alpha channels)
void _GetGridSize(int* size){
	size[0] = response.cells.size()!=0 ? response.width : -1;
	size[1] = response.cells.size()!=0 ? response.height : -1;
}

void _GetGridSizeZero(int* size){
	size[0] = responseZero.cells.size()!=0 ? responseZero.width : -1;
	size[1] = responseZero.cells.size()!=0 ? responseZero.height : -1;
}
double _GetGridResolution(){
	return response.cells.size()!=0 ? response.resolution : -1;
}

double _GetGridResolutionZero(){
	return responseZero.cells.size()!=0 ? responseZero.resolution : -1;
}

//Pose format as [x,y,z] + quat[x,y,z,w]
void _GetSubmapPose(Node* node, double* pose)
{
	::cartographer_generic_msgs::SubmapList SubmapList = node->GetSubmapList();
	::cartographer_generic_msgs::SubmapEntry LastSubEnt = SubmapList.submap.back();

	double slice_pose[7];
	double submap_pose[7];
	submap_pose[0] = LastSubEnt.pose.position.x ;
	submap_pose[1] = LastSubEnt.pose.position.y ;
	submap_pose[2] = LastSubEnt.pose.position.z ;
	submap_pose[3] = LastSubEnt.pose.orientation.w ;
	submap_pose[4] = LastSubEnt.pose.orientation.x ;
	submap_pose[5] = LastSubEnt.pose.orientation.y ;
	submap_pose[6] = LastSubEnt.pose.orientation.z ;

	//	std::stringstream ss;
	//	ss << "INFO Submap" << LastSubEnt.submap_index << ": {" ;
	//	ss << "Position (" << LastSubEnt.pose.position.x << "," << LastSubEnt.pose.position.y << "," << LastSubEnt.pose.position.z << "), " ;
	//	ss << "Orientation (" << LastSubEnt.pose.orientation.w << "," << LastSubEnt.pose.orientation.x << "," << LastSubEnt.pose.orientation.y << "," << LastSubEnt.pose.orientation.z << ")" ;
	//	ss << "} \n";
	//	int size = SubmapList.submap.size();
	//	for(int i=0; i<size; i++ ){
	//		::cartographer_generic_msgs::SubmapEntry subEnt = SubmapList.submap[i];
	//		ss << "Submap" << subEnt.submap_index << ": {" ;
	//		ss << "Position (" << subEnt.pose.position.x << "," << subEnt.pose.position.y << "," << subEnt.pose.position.z << "), " ;
	//		ss << "Orientation (" << subEnt.pose.orientation.x << "," << subEnt.pose.orientation.y << "," << subEnt.pose.orientation.z << "," << subEnt.pose.orientation.w << ")" ;
	//		ss << "} \n";
	//	}

	slice_pose[0] =  response.slice_pose.position.x - response.height*response.resolution/2 ;
	slice_pose[1] =  response.slice_pose.position.y - response.width*response.resolution/2;
	slice_pose[2] =  response.slice_pose.position.z ;
	slice_pose[3] =  response.slice_pose.orientation.w ;
	slice_pose[4] =  response.slice_pose.orientation.x ;
	slice_pose[5] =  response.slice_pose.orientation.y ;
	slice_pose[6] =  response.slice_pose.orientation.z ;

	//	ss << "INFO Slice_pose" << LastSubEnt.submap_index << ": {" ;
	//	ss << "Position (" << response.slice_pose.position.x << "," << response.slice_pose.position.y << "," << response.slice_pose.position.z << "), " ;
	//	ss << "Orientation (" << response.slice_pose.orientation.w << "," << response.slice_pose.orientation.x << "," << response.slice_pose.orientation.y << "," << response.slice_pose.orientation.z << ")" ;
	//	ss << "} \n";

	ComposePoses(pose, submap_pose, slice_pose);
	//	ss << "INFO Pose_submap" << LastSubEnt.submap_index << ": {" ;
	//	ss << "Position (" << pose[0] << "," << pose[1] << "," << pose[2] << "), " ;
	//	ss << "Orientation (" << pose[3] << "," << pose[4] << "," << pose[5] << "," << pose[6] << ")" ;
	//	ss << "} \n";

	//	int read=0;
	//	std::string substr;
	//	do {
	//		substr = ss.str().substr(read,android_log_max_length);
	//		LOG(INFO) << substr;
	//		read+=substr.length();
	//	} while(read<ss.str().length());

}

void _GetSubmapPoseZero(Node* node, double* pose)
{
	::cartographer_generic_msgs::SubmapList SubmapList = node->GetSubmapList();
	::cartographer_generic_msgs::SubmapEntry ZeroSubEnt = SubmapList.submap[0];

	double slice_pose[7];
	double submap_pose[7];
	submap_pose[0] = ZeroSubEnt.pose.position.x ;
	submap_pose[1] = ZeroSubEnt.pose.position.y ;
	submap_pose[2] = ZeroSubEnt.pose.position.z ;
	submap_pose[3] = ZeroSubEnt.pose.orientation.w ;
	submap_pose[4] = ZeroSubEnt.pose.orientation.x ;
	submap_pose[5] = ZeroSubEnt.pose.orientation.y ;
	submap_pose[6] = ZeroSubEnt.pose.orientation.z ;

	slice_pose[0] =  responseZero.slice_pose.position.x - responseZero.height*responseZero.resolution/2 ;
	slice_pose[1] =  responseZero.slice_pose.position.y - responseZero.width*responseZero.resolution/2;
	slice_pose[2] =  responseZero.slice_pose.position.z ;
	slice_pose[3] =  responseZero.slice_pose.orientation.w ;
	slice_pose[4] =  responseZero.slice_pose.orientation.x ;
	slice_pose[5] =  responseZero.slice_pose.orientation.y ;
	slice_pose[6] =  responseZero.slice_pose.orientation.z ;

	ComposePoses(pose, submap_pose, slice_pose);
}

//Retrieve occupancy grid
//intensity and alpha channels are converted form uint8 (char) to int
void _GetOccupancyGrid (int* intensity, int* alpha) {
	std::stringstream ss1, ss2, ss3;
	ss1 << "Alpha" << response.submap_version << ": [" ;
	ss2 << "Intensity" << response.submap_version << ": [" ;
	ss3 << "Observed" << response.submap_version << ": [" ;
	for (int i = 0; i < response.height; ++i) {
		for (int j = 0; j < response.width; ++j) {
			intensity[i*response.width + j] = static_cast<int>(response.cells[(i * response.width + j) * 2]);
			alpha[i*response.width + j] = static_cast<int>(response.cells[(i * response.width + j) * 2 + 1]);
			uint8_t intensity = response.cells[(i * response.width + j) * 2];
			uint8_t alpha = response.cells[(i * response.width + j) * 2 + 1];
			uint8_t observed = (intensity == 0 && alpha == 0) ? 0 : 255;
			ss1 << alpha << ",";
			ss2 << intensity << ",";
			ss3 << observed << ",";


		}
	}
	ss1 << "] \n";
	ss2 << "] \n";
	ss3 << "] \n";
	LOG(INFO) << ss1.str();
	LOG(INFO) << ss2.str();
	LOG(INFO) << ss3.str();

}

void _GetOccupancyGridZero (int* intensity, int* alpha) {

	for (int i = 0; i < responseZero.height; ++i) {
		for (int j = 0; j < responseZero.width; ++j) {
			intensity[i*responseZero.width + j] = static_cast<int>(responseZero.cells[(i * responseZero.width + j) * 2]);
			alpha[i*responseZero.width + j] = static_cast<int>(responseZero.cells[(i * responseZero.width + j) * 2 + 1]);
		}
	}
}

/*******************************************************************************
 * - Trajectory Query/Retrieve
 ********************************************************************************/
void _GetPose (Node* node, int64 time, float* pose){
	::cartographer::common::Time time_now = ::cartographer::common::FromUniversal(time);
	::cartographer_generic_msgs::MarkerArray TrajectoryList = node->GetTrajectoryNodeList(time_now);
	std::stringstream ss;
	for(int i=0 ; i<TrajectoryList.markers.size(); i++ ){
		::cartographer_generic_msgs::Marker marker = TrajectoryList.markers.at(i);
		ss << "Trajectory" << marker.id << ": {" ;
		for(int j=0; j<marker.points.size(); j++ ){
			::cartographer_generic_msgs::Point point=marker.points.at(j);
			ss << "(" << point.x << "," << point.y << "," << point.z << ")" ;
			pose[0] = point.x;
			pose[1] = point.y;
			pose[2] = point.z;
		}
		ss << "} \n";
	}
	//	int read=0;
	//	std::string substr;
	//	do {
	//		substr = ss.str().substr(read,android_log_max_length);
	//		LOG(INFO) << substr;
	//		read+=substr.length();
	//	} while(read<ss.str().length());
}

}  // namespace cartographer_generic
}  // extern "C"


//int main(int argc, char** argv) {
//	google::InitGoogleLogging(argv[0]);
//	google::ParseCommandLineFlags(&argc, &argv, true);
//
//	CHECK(!FLAGS_configuration_directory.empty())
//	<< "-configuration_directory is missing.";
//	CHECK(!FLAGS_configuration_basename.empty())
//	<< "-configuration_basename is missing.";
//
//	//TODO
//	//	::ros::init(argc, argv, "cartographer_node");
//	//	::ros::start();
//
//	cartographer_generic::Run();
//
//	//TODO
//	//	::ros::shutdown();
//}
#endif
