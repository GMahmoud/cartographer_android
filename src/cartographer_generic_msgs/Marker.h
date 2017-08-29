#ifndef CARTOGRAPHER_GENERIC_MSGS_MSGS_MESSAGE_MARKER_H
#define CARTOGRAPHER_GENERIC_MSGS_MSGS_MESSAGE_MARKER_H

#include <string>
#include <vector>
#include <map>

#include "cartographer_generic_msgs/Header.h"
#include "cartographer_generic_msgs/Pose.h"
#include "cartographer_generic_msgs/Vector3.h"
#include "cartographer_generic_msgs/Point.h"

namespace cartographer_generic_msgs
{

struct Marker
{

	Marker()
	: header()
	, ns()
	, id(0)
	, type(0)
	, action(0)
	, pose()
	, scale()
	, points()

	{
	}

	Header header;
	std::string ns;
	int32_t id;
	int32_t  type;
	int32_t action;
	Pose pose;
	Vector3 scale;
	std::vector< Point>  points;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::Marker> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::Marker const> ConstPtr;

}; // struct Marker

typedef boost::shared_ptr< ::cartographer_generic_msgs::Marker > MarkerPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Marker const> MarkerConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MSGS_MESSAGE_MARKER_H
