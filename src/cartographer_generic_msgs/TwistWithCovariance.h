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
