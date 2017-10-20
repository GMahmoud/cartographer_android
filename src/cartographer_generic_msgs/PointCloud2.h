#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD2_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD2_H

#include <string>
#include <vector>
#include <map>

#include <cartographer_generic_msgs/Header.h>
#include <cartographer_generic_msgs/PointField.h>

namespace cartographer_generic_msgs
{

struct PointCloud2
{

	PointCloud2()
	: header()
	, height(0)
	, width(0)
	, fields()
	, is_bigendian(false)
	, point_step(0)
	, row_step(0)
	, data()
	, is_dense(false)  {
	}

	Header header;
	uint32_t  height;
	uint32_t  width;
	std::vector< PointField >  fields;
	uint8_t is_bigendian;
	uint32_t  point_step;
	uint32_t row_step;
	std::vector<uint8_t >  data;
	uint8_t is_dense;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud2> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud2 const> ConstPtr;

}; // struct PointCloud

typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud2 > PointCloud2Ptr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud2 const> PointCloud2ConstPtr;


} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD2_H
