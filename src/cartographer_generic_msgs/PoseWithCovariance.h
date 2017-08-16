#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POSEWITHCOVARIANCE_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POSEWITHCOVARIANCE_H


#include <string>
#include <vector>
#include <map>

#include "Pose.h"
#include <boost/array.hpp>

namespace cartographer_generic_msgs
{

struct PoseWithCovariance
{


  PoseWithCovariance()
    : pose()
    , covariance()  {
      covariance.assign(0.0);
  }

   ::cartographer_generic_msgs::Pose pose;
   boost::array<double, 36> covariance;

  typedef boost::shared_ptr< ::cartographer_generic_msgs::PoseWithCovariance> Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::PoseWithCovariance const> ConstPtr;

}; // struct PoseWithCovariance_

typedef boost::shared_ptr< ::cartographer_generic_msgs::PoseWithCovariance > PoseWithCovariancePtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::PoseWithCovariance const> PoseWithCovarianceConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POSEWITHCOVARIANCE_H
