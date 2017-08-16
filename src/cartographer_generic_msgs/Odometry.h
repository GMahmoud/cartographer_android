

#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_ODOMETRY_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_ODOMETRY_H


#include <string>
#include <vector>
#include <map>

#include "Header.h"
#include "PoseWithCovariance.h"
#include "TwistWithCovariance.h"

namespace cartographer_generic_msgs
{

struct Odometry
{

  Odometry()
    : header()
    , child_frame_id()
    , pose()
    , twist()  {
    }


  Header header;
  std::string child_frame_id;
  ::cartographer_generic_msgs::PoseWithCovariance  pose;
  ::cartographer_generic_msgs::TwistWithCovariance twist;

  typedef boost::shared_ptr< ::cartographer_generic_msgs::Odometry > Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::Odometry const> ConstPtr;

}; // struct Odometry

typedef boost::shared_ptr< ::cartographer_generic_msgs::Odometry > OdometryPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Odometry const> OdometryConstPtr;


} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_ODOMETRY_H
