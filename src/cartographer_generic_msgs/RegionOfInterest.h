/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_REGIONOFINTEREST_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_REGIONOFINTEREST_H

#include <string>
#include <vector>
#include <map>

namespace cartographer_generic_msgs
{
struct RegionOfInterest
{
	RegionOfInterest()
	: x_offset(0)
	, y_offset(0)
	, height(0)
	, width(0)
	, do_rectify(false)  {
	}

	uint32_t x_offset;
	uint32_t y_offset;
	uint32_t height;
	uint32_t width;
	uint8_t  do_rectify;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::RegionOfInterest> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::RegionOfInterest const> ConstPtr;

}; // struct RegionOfInterest

typedef boost::shared_ptr< ::cartographer_generic_msgs::RegionOfInterest > RegionOfInterestPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::RegionOfInterest const> RegionOfInterestConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_REGIONOFINTEREST_H
