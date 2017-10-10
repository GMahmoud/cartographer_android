#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTFIELD_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTFIELD_H

#include <string>
#include <vector>
#include <map>


namespace cartographer_generic_msgs
{
struct PointField
{

	PointField()
	: name()
	, offset(0)
	, datatype(0)
	, count(0)  {
	}


	std::string name;
	uint32_t offset;
	uint8_t  datatype;
	uint32_t count;

	enum { INT8 = 1u };
	enum { UINT8 = 2u };
	enum { INT16 = 3u };
	enum { UINT16 = 4u };
	enum { INT32 = 5u };
	enum { UINT32 = 6u };
	enum { FLOAT32 = 7u };
	enum { FLOAT64 = 8u };

	typedef boost::shared_ptr< ::cartographer_generic_msgs::PointField > Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::PointField const> ConstPtr;

}; // struct PointField

typedef boost::shared_ptr< ::cartographer_generic_msgs::PointField > PointFieldPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::PointField const> PointFieldConstPtr;


} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTFIELD_H
