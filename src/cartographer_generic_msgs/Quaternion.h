#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_QUATERNION_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_QUATERNION_H

#include <string>
#include <vector>
#include <map>

namespace cartographer_generic_msgs
{
struct Quaternion
{


	Quaternion()
	: x(0.0)
	, y(0.0)
	, z(0.0)
	, w(0.0)  {
	}

	double x;
	double y;
	double z;
	double w;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::Quaternion> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::Quaternion const> ConstPtr;

}; // struct Quaternion


typedef boost::shared_ptr< ::cartographer_generic_msgs::Quaternion > QuaternionPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Quaternion const> QuaternionConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_QUATERNION_H
