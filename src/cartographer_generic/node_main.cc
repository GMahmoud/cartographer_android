#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer_generic/node.h"
#include "cartographer_generic/node_options.h"
#include "cartographer_generic/trajectory_options.h"
#include "cartographer_generic_msgs/LaserScan.h"
#include "cartographer_generic_msgs/Odometry.h"
#ifdef __ANDROID__
#include <android/log.h>
#endif
#include <glog/logging.h>


#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer_generic/test.h"
#include "cartographer_generic_msgs/SubmapQuery.h"
#include "cartographer_generic_msgs/SubmapList.h"

#if __cplusplus
extern "C" {
namespace cartographer_generic {

int trajectory_id = -1 ;
uint32_t i = -1 ;
string configuration_directory = "/";
string configuration_basename = "buddy_lidar_2d.lua";
::cartographer_generic_msgs::SubmapQuery::Response response;

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



Node* _Run() {
	constexpr double kTfBufferCacheTimeInSeconds = 1e6;
	LOG(INFO) << "_Run() Begins";
	//TODO
	//  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
	//  tf2_ros::TransformListener tf(tf_buffer);

	NodeOptions node_options;
	TrajectoryOptions trajectory_options;
	std::tie(node_options, trajectory_options) = LoadOptions();

	Node* node = new Node(node_options/*, &tf_buffer*/);
	//TODO If we want to add a primary map
	//	if (!FLAGS_map_filename.empty()) {
	//		node.LoadMap(FLAGS_map_filename);
	//	}
	trajectory_id = node->StartTrajectory(trajectory_options);
	LOG(INFO) << "_Run() Ends";
	return node;

}

int _GetTrajectoryId (){
	LOG(INFO) << "_GetTrajectoryId() Begins" ;
	LOG(INFO) << "Trajectory_id = " << trajectory_id ;
	LOG(INFO) << "_GetTrajectoryId() Ends" ;
	return trajectory_id ;
}

void _LaserScanCallback(Node* node, float* ranges, int64 time) {
	//LOG(INFO) << "_LaserScanCallback(Node*, ::cartographer_generic_msgs::LaserScan::ConstPtr) Begins" ;
	::cartographer_generic_msgs::LaserScan::Ptr  msg(new ::cartographer_generic_msgs::LaserScan());
	i++;
	msg->ranges.reserve(360);
	for(int i=0; i<360; i++)
		msg->ranges.push_back(ranges[i]) ;
	msg->header.stamp = ::cartographer::common::FromUniversal(time);
	msg->header.seq = i;
	msg->header.frame_id = "buddy_tablet";
	//	LOG(INFO) << "ranges size = " << msg->ranges.size();
	//	LOG(INFO) << "range(size/2) = " << msg->ranges.at(floor(msg->ranges.size()/2));
	msg->angle_min=-1.57079637051;
	msg->angle_max=1.57079637051;
	msg->angle_increment=0.00872664619237;
	msg->time_increment=0.0;
	msg->scan_time=0.0333333350718;
	msg->range_min=0.449999988079;
	msg->range_max=6.0;
	node->LaserScanCallback(msg, trajectory_id);
	//LOG(INFO) << "_LaserScanCallback(Node*, ::cartographer_generic_msgs::LaserScan::ConstPtr) Ends" ;
}

void _OdometryCallback(Node* node, ::cartographer_generic_msgs::Odometry::Ptr& msg) {
	LOG(INFO) << "_LaserScanCallback(Node*, ::cartographer_generic_msgs::Odometry::ConstPtr) Begins" ;
	node->OdometryCallback(msg, trajectory_id);
	LOG(INFO) << "_LaserScanCallback(Node*, ::cartographer_generic_msgs::Odometry::ConstPtr) Ends" ;
}


int _GetOccupancyGrid (Node* node, uint8_t* cells) {
	LOG(INFO) << " _GetOccupancyGrid (Node* node) Begins" ;
	::cartographer_generic_msgs::SubmapList SubmapList = node->GetSubmapList();
	//LOG(INFO) << "Trajectory_id of submap(0) = " << SubmapList.submap.at(0).trajectory_id;

	::cartographer_generic_msgs::SubmapQuery::Request request;
	request.submap_index = 0;
	request.trajectory_id = 0;
	node->HandleSubmapQuery(request,response);
	LOG(INFO) << " Cell size() = " << response.cells.size();

	LOG(INFO) << " _GetOccupancyGrid (Node* node) Ends" ;
	int size = response.cells.size();

	cells = new uint8_t[size];
	for ( int it=0; it!=size; ++it ){
		cells[it] = response.cells.at(it);
		//LOG(INFO) << "cells = " << cells[it];
	}
	return size;
}

void _Stop (Node* node) {
	LOG(INFO) << "_Stop(Node*) Begins" ;
	node->FinishAllTrajectories();
	LOG(INFO) << "_Stop(Node*) Ends" ;
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







