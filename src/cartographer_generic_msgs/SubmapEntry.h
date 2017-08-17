#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPENTRY_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPENTRY_H

#include <string>
#include <vector>
#include <map>

#include "cartographer_generic_msgs/Pose.h"

namespace cartographer_generic_msgs
{

struct SubmapEntry
{

  SubmapEntry()
    : trajectory_id(0)
    , submap_index(0)
    , submap_version(0)
    , pose()  {
    }

  int32_t trajectory_id;
   int32_t submap_index;
   int32_t submap_version;
   Pose pose;


  typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapEntry > Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapEntry const> ConstPtr;

}; // struct SubmapEntry

typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapEntry > SubmapEntryPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapEntry const> SubmapEntryConstPtr;

}

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPENTRY_H
