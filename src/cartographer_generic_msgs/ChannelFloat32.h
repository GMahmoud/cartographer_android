#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_CHANNELFLOAT32_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_CHANNELFLOAT32_H

#include <string>
#include <vector>
#include <map>

namespace cartographer_generic_msgs
{
struct ChannelFloat32
{
  ChannelFloat32()
    : name()
    , values()  {
    }


   std::string name;

  std::vector<float> values;




  typedef boost::shared_ptr< ::cartographer_generic_msgs::ChannelFloat32 > Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::ChannelFloat32 const> ConstPtr;

}; // struct ChannelFloat32

typedef boost::shared_ptr< ::cartographer_generic_msgs::ChannelFloat32 > ChannelFloat32Ptr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::ChannelFloat32 const> ChannelFloat32ConstPtr;

} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_CHANNELFLOAT32_H
