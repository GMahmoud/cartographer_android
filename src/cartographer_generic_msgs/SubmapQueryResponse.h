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

#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPQUERYRESPONSE_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPQUERYRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include "cartographer_generic_msgs/Pose.h"

namespace cartographer_generic_msgs
{
struct SubmapQueryResponse
{

	SubmapQueryResponse()
	: submap_version(0)
	, cells()
	, width(0)
	, height(0)
	, resolution(0.0)
	, slice_pose()
	, error_message()  {
	}

	int32_t submap_version;
	std::vector<uint8_t>  cells;
	int32_t width;
	int32_t  height;
	double resolution;
	Pose slice_pose;
	std::string error_message;


	typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapQueryResponse > Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapQueryResponse const> ConstPtr;

}; // struct SubmapQueryResponse

typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapQueryResponse > SubmapQueryResponsePtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::SubmapQueryResponse const> SubmapQueryResponseConstPtr;

}

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_SUBMAPQUERYRESPONSE_H
