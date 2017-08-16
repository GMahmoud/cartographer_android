#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINT_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINT_H

#include <string>
#include <vector>
#include <map>


namespace cartographer_generic_msgs
{

struct Point
{
	Point()
	: x(0.0)
	, y(0.0)
	, z(0.0)  {
	}

	double x;
	double y;
	double z;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::Point> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::Point const> ConstPtr;

}; // struct Point

typedef boost::shared_ptr< ::cartographer_generic_msgs::Point > PointPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Point const> PointConstPtr;


} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINT_H
