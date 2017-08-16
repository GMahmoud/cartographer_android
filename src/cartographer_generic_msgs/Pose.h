#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POSE_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POSE_H

#include <string>
#include <vector>
#include <map>

#include "Point.h"
#include "Quaternion.h"

namespace cartographer_generic_msgs
{

struct Pose
{

  Pose()
    : position()
    , orientation()  {
    }

   ::cartographer_generic_msgs::Point position;
   ::cartographer_generic_msgs::Quaternion orientation;

  typedef boost::shared_ptr< ::cartographer_generic_msgs::Pose> Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::Pose const> ConstPtr;

}; // struct Pose

typedef boost::shared_ptr< ::cartographer_generic_msgs::Pose > PosePtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Pose const> PoseConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POSE_H
