#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_MARKERARRAY_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_MARKERARRAY_H


#include <string>
#include <vector>
#include <map>

#include "cartographer_generic_msgs/Marker.h"

namespace cartographer_generic_msgs
{

struct MarkerArray
{

  MarkerArray()
    : markers()  {
    }

   std::vector< ::cartographer_generic_msgs::Marker> markers;

  typedef boost::shared_ptr< ::cartographer_generic_msgs::MarkerArray > Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::MarkerArray const> ConstPtr;

}; // struct MarkerArray

typedef boost::shared_ptr< ::cartographer_generic_msgs::MarkerArray > MarkerArrayPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::MarkerArray const> MarkerArrayConstPtr;

} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_MARKERARRAY_H
