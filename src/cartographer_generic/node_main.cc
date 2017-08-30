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

//Current trajectory id
int trajectory_id = -1 ;

string configuration_directory = "/";
string configuration_basename = "buddy_lidar_2d.lua";

//LaserScan callback message
::cartographer_generic_msgs::LaserScan::Ptr msg;

//Current submap query response
::cartographer_generic_msgs::SubmapQuery::Response response;

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
    msg.reset(new cartographer_generic_msgs::LaserScan());
    msg->ranges.reserve(360);
    msg->header.seq = 0;
    msg->header.frame_id = "buddy_tablet";
    msg->angle_min=-1.57079637051;
    msg->angle_max=1.57079637051;
    msg->angle_increment=0.00872664619237;
    msg->time_increment=0.0;
    msg->scan_time=0.0333333350718;
    msg->range_min=0.449999988079;
    msg->range_max=6.0;

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
    msg->ranges.clear(); //clear does not resize
    for(int i=0; i<360; i++) //1° definition - LaserScan data
	   msg->ranges.push_back(ranges[i]) ;
    //Convert time ticks in common time
    msg->header.stamp = ::cartographer::common::FromUniversal(time);

    node->LaserScanCallback(msg, trajectory_id);
    //Increment header sequence
    msg->header.seq++;
}

void _OdometryCallback(Node* node, ::cartographer_generic_msgs::Odometry::Ptr& msg) {
	LOG(INFO) << "_LaserScanCallback(Node*, ::cartographer_generic_msgs::Odometry::ConstPtr) Begins" ;
	node->OdometryCallback(msg, trajectory_id);
	LOG(INFO) << "_LaserScanCallback(Node*, ::cartographer_generic_msgs::Odometry::ConstPtr) Ends" ;
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

//Retrieve grid size (Unity should handle most of memory allocation)
// response.cells.size() = width * height * 2 (intensity and alpha channels)
void _GetGridSize(int* size){
    size[0] = response.cells.size()!=0 ? response.width : -1;
    size[1] = response.cells.size()!=0 ? response.height : -1;
}

double _GetGridResolution(){
	return response.cells.size()!=0 ? response.resolution : -1;
}

//Retrieve occupancy grid
//intensity and alpha channels are converted form uint8 (char) to int
void _GetOccupancyGrid (int* intensity, int* alpha) {
	for (int i = 0; i < response.height; ++i) {
		for (int j = 0; j < response.width; ++j) {
  			intensity[i*response.width + j] = static_cast<int>(response.cells[(i * response.width + j) * 2]);
  			alpha[i*response.width + j] = static_cast<int>(response.cells[(i * response.width + j) * 2 + 1]);
		}
	}
}

/*******************************************************************************
* - Trajectory Query/Retrieve
********************************************************************************/
void _GetPose (Node* node, int64 time){
	LOG(INFO) << "_GetTrajectoryList() Begins" ;
	::cartographer::common::Time time_now = ::cartographer::common::FromUniversal(time);
	::cartographer_generic_msgs::MarkerArray TrajectoryList = node->GetTrajectoryNodeList(time_now);
	LOG(INFO) << "TrajectoryList.markers.size() =" << TrajectoryList.markers.size();
	std::stringstream ss;
	for(int i=0 ; i<TrajectoryList.markers.size(); i++ ){
		::cartographer_generic_msgs::Marker marker = TrajectoryList.markers.at(i);
		ss << marker.id << ": {" ;
		for(int j=0; j<marker.points.size(); j++ ){
			::cartographer_generic_msgs::Point point=marker.points.at(j);
			ss << "(" << point.x << "," << point.y << "," << point.z << ")" ;
			pose[0] = point.x;
			pose[1] = point.y;
			pose[2] = point.z;
		}
		ss << "} \n";
	}
	LOG(INFO) << ss.str();
	LOG(INFO) << "_GetTrajectoryList() Ends" ;
	return;
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
