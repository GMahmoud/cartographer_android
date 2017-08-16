#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_TWIST_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_TWIST_H


#include <string>
#include <vector>
#include <map>


#include "Vector3.h"

namespace cartographer_generic_msgs
{

struct Twist
{


  Twist()
    : linear()
    , angular()  {
    }


   ::cartographer_generic_msgs::Vector3 linear;
   ::cartographer_generic_msgs::Vector3 angular;

  typedef boost::shared_ptr< ::cartographer_generic_msgs::Twist > Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::Twist const> ConstPtr;

}; // struct Twist


typedef boost::shared_ptr< ::cartographer_generic_msgs::Twist > TwistPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Twist const> TwistConstPtr;


} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_TWIST_H
