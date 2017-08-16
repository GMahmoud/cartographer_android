#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_TWISTWITHCOVARIANCE_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_TWISTWITHCOVARIANCE_H


#include <string>
#include <vector>
#include <map>


#include "Twist.h"
#include <boost/array.hpp>

namespace cartographer_generic_msgs
{
struct TwistWithCovariance
{


  TwistWithCovariance()
    : twist()
    , covariance()  {
      covariance.assign(0.0);
  }


  ::cartographer_generic_msgs::Twist twist;
  boost::array<double, 36> covariance;

  typedef boost::shared_ptr< ::cartographer_generic_msgs::TwistWithCovariance> Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::TwistWithCovariance const> ConstPtr;

}; // struct TwistWithCovariance

typedef boost::shared_ptr< ::cartographer_generic_msgs::TwistWithCovariance > TwistWithCovariancePtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::TwistWithCovariance const> TwistWithCovarianceConstPtr;

} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_TWISTWITHCOVARIANCE_H
