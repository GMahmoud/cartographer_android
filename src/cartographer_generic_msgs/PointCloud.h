#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD_H

#include <string>
#include <vector>
#include <map>

#include "cartographer_generic_msgs/Header.h"
#include "cartographer_generic_msgs/Point32.h"
#include "cartographer_generic_msgs/ChannelFloat32.h"

namespace cartographer_generic_msgs
{

struct PointCloud
{

	PointCloud()
	: header()
	, points()
	, channels()  {
	}

	Header header;
	std::vector< Point32 >  points;
	std::vector< ChannelFloat32 > channels;


	typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud const> ConstPtr;

}; // struct PointCloud


typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud > PointCloudPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud const> PointCloudConstPtr;

} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD_H
