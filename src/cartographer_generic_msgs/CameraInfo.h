#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_CAMERAINFO_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_CAMERAINFO_H


#include <string>
#include <vector>
#include <map>

#include "cartographer_generic_msgs/Header.h"
#include "cartographer_generic_msgs/RegionOfInterest.h"

namespace cartographer_generic_msgs
{

struct CameraInfo
{
	CameraInfo()
	: header()
	, height(0)
	, width(0)
	, distortion_model()
	, D()
	, K()
	, R()
	, P()
	, binning_x(0)
	, binning_y(0)
	, roi()  {
		K.assign(0.0);

		R.assign(0.0);

		P.assign(0.0);
	}

	Header header;
	uint32_t height;
	uint32_t width;
	std::string distortion_model;
	std::vector<double>  D;
	boost::array<double, 9>  K;
	boost::array<double, 9>  R;
	boost::array<double, 12> P;
	uint32_t binning_x;
	uint32_t binning_y;
	RegionOfInterest roi;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::CameraInfo > Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::CameraInfo const> ConstPtr;

}; // struct CameraInfo


typedef boost::shared_ptr< ::cartographer_generic_msgs::CameraInfo > CameraInfoPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::CameraInfo const> CameraInfoConstPtr;

} // namespace cartographer_generic_msgs


#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_CAMERAINFO_H
