#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_REGIONOFINTEREST_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_REGIONOFINTEREST_H

#include <string>
#include <vector>
#include <map>

namespace cartographer_generic_msgs
{
struct RegionOfInterest
{
	RegionOfInterest()
	: x_offset(0)
	, y_offset(0)
	, height(0)
	, width(0)
	, do_rectify(false)  {
	}

	uint32_t x_offset;
	uint32_t y_offset;
	uint32_t height;
	uint32_t width;
	uint8_t  do_rectify;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::RegionOfInterest> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::RegionOfInterest const> ConstPtr;

}; // struct RegionOfInterest

typedef boost::shared_ptr< ::cartographer_generic_msgs::RegionOfInterest > RegionOfInterestPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::RegionOfInterest const> RegionOfInterestConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_REGIONOFINTEREST_H
