#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_VECTOR3_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_VECTOR3_H


#include <string>
#include <vector>
#include <map>

namespace cartographer_generic_msgs
{

struct Vector3
{


  Vector3()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }

   double x;
   double y;
   double z;

  typedef boost::shared_ptr< ::cartographer_generic_msgs::Vector3 > Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::Vector3 const> ConstPtr;

}; // struct Vector3

typedef boost::shared_ptr< ::cartographer_generic_msgs::Vector3 > Vector3Ptr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Vector3 const> Vector3ConstPtr;


} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_VECTOR3_H
