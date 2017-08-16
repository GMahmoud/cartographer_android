#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SENSORTOPICS_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SENSORTOPICS_H


#include <string>
#include <vector>
#include <map>

namespace cartographer_generic_msgs
{
struct SensorTopics
{

  SensorTopics()
    : laser_scan_topic()
    , multi_echo_laser_scan_topic()
    , point_cloud2_topic()
    , imu_topic()
    , odometry_topic()  {
    }

   std::string laser_scan_topic;
   std::string multi_echo_laser_scan_topic;
   std::string point_cloud2_topic;
   std::string imu_topic;
   std::string odometry_topic;

  typedef boost::shared_ptr< ::cartographer_generic_msgs::SensorTopics > Ptr;
  typedef boost::shared_ptr< ::cartographer_generic_msgs::SensorTopics const> ConstPtr;

}; // struct SensorTopics

typedef boost::shared_ptr< ::cartographer_generic_msgs::SensorTopics > SensorTopicsPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::SensorTopics const> SensorTopicsConstPtr;


} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SENSORTOPICS_H
