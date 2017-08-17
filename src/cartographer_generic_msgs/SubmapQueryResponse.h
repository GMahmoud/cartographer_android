#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPQUERYRESPONSE_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPQUERYRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include "cartographer_generic_msgs/Pose.h"

namespace cartographer_generic_msgs
{
struct SubmapQueryResponse
{

	SubmapQueryResponse()
	: submap_version(0)
	, cells()
	, width(0)
	, height(0)
	, resolution(0.0)
	, slice_pose()
	, error_message()  {
	}

	int32_t submap_version;
	std::vector<uint8_t>  cells;
	int32_t width;
	int32_t  height;
	double resolution;
	Pose slice_pose;
	std::string error_message;


	typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapQueryResponse > Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapQueryResponse const> ConstPtr;

}; // struct SubmapQueryResponse

typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapQueryResponse > SubmapQueryResponsePtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapQueryResponse const> SubmapQueryResponseConstPtr;

}

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPQUERYRESPONSE_H
