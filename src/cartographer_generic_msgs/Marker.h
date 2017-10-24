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

#ifndef CARTOGRAPHER_GENERIC_MSGS_MSGS_MESSAGE_MARKER_H
#define CARTOGRAPHER_GENERIC_MSGS_MSGS_MESSAGE_MARKER_H

#include <string>
#include <vector>
#include <map>

#include "cartographer_generic_msgs/Header.h"
#include "cartographer_generic_msgs/Pose.h"
#include "cartographer_generic_msgs/Vector3.h"

namespace cartographer_generic_msgs
{

struct Marker
{

	Marker()
	: header()
	, ns()
	, id(0)
	, type(0)
	, action(0)
	, pose()
	, scale()
	, poses()

	{
	}

	Header header;
	std::string ns;
	int32_t id;
	int32_t  type;
	int32_t action;
	Pose pose;
	Vector3 scale;
	std::vector< Pose>  poses;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::Marker> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::Marker const> ConstPtr;

}; // struct Marker

typedef boost::shared_ptr< ::cartographer_generic_msgs::Marker > MarkerPtr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::Marker const> MarkerConstPtr;

} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MSGS_MESSAGE_MARKER_H
