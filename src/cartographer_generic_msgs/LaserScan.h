#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_LASERSCAN_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_LASERSCAN_H


#include <string>
#include <vector>
#include <map>

#include "Header.h"

namespace cartographer_generic_msgs
{

struct LaserScan
{
  LaserScan()
    : header()
    , angle_min(0.0)
    , angle_max(0.0)
    , angle_increment(0.0)
    , time_increment(0.0)
    , scan_time(0.0)
    , range_min(0.0)
    , range_max(0.0)
    , ranges()
    , intensities()  {
    }

   Header header;
   float angle_min;
   float angle_max;
   float angle_increment;
   float time_increment;
   float scan_time;
   float range_min;
   float range_max;
   std::vector<float> ranges;
   std::vector<float> intensities;

   typedef boost::shared_ptr< ::cartographer_generic_msgs::LaserScan> Ptr;
   typedef boost::shared_ptr< ::cartographer_generic_msgs::LaserScan const> ConstPtr;

}; // struct LaserScan

typedef boost::shared_ptr< ::cartographer_generic_msgs::LaserScan > LaserScanPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::LaserScan const> LaserScanConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MESSAGE_LASERSCAN_H
