#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_HEADER_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_HEADER_H


#include <string>
#include <vector>
#include <map>

#include "cartographer/common/time.h"

namespace cartographer_generic_msgs
{

struct Header
{

	Header()
	: seq(0)
	, stamp()
	, frame_id()  {
	}

	uint32_t seq;
	::cartographer::common::Time  stamp;
	std::string frame_id;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::Header > Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::Header const> ConstPtr;
}; // struct Header


typedef boost::shared_ptr< ::cartographer_generic_msgs::Header > HeaderPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Header const> HeaderConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_HEADER_H
