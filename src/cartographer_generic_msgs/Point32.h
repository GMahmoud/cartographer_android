#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINT32_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINT32_H

#include <string>
#include <vector>
#include <map>


namespace cartographer_generic_msgs
{

struct Point32
{
	Point32()
	: x(0.0)
	, y(0.0)
	, z(0.0)  {
	}

	float x;
	float y;
	float z;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::Point32> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::Point32 const> ConstPtr;

}; // struct Point

typedef boost::shared_ptr< ::cartographer_generic_msgs::Point32 > Point32Ptr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Point32 const> Point32ConstPtr;


} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINT32_H
