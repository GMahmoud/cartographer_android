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

#ifndef CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD2_H
#define CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD2_H

#include <string>
#include <vector>
#include <map>

#include <cartographer_generic_msgs/Header.h>
#include <cartographer_generic_msgs/PointField.h>

namespace cartographer_generic_msgs
{

struct PointCloud2
{

	PointCloud2()
	: header()
	, height(0)
	, width(0)
	, fields()
	, is_bigendian(false)
	, point_step(0)
	, row_step(0)
	, data()
	, is_dense(false)  {
	}

	Header header;
	uint32_t  height;
	uint32_t  width;
	std::vector< PointField >  fields;
	uint8_t is_bigendian;
	uint32_t  point_step;
	uint32_t row_step;
	std::vector<uint8_t >  data;
	uint8_t is_dense;

	typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud2> Ptr;
	typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud2 const> ConstPtr;

}; // struct PointCloud

typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud2 > PointCloud2Ptr;
typedef boost::shared_ptr< ::cartographer_generic_msgs::PointCloud2 const> PointCloud2ConstPtr;


} // namespace cartographer_generic_msgs

#endif // CARTOGRAPHER_GENERIC_MSGS_MESSAGE_POINTCLOUD2_H
