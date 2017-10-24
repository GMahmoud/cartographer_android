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

#ifndef CARTOGRAPHER_GENERIC_MSGS_POINTCLOUD2ITERATOR_H
#define CARTOGRAPHER_GENERIC_MSGS_POINTCLOUD2ITERATOR_H

#include "cartographer_generic_msgs/PointCloud2.h"
#include <string>
#include <vector>

namespace cartographer_generic_msgs
{

namespace impl
{
template<typename T, typename TT, typename U, typename C, template <typename> class V>
class PointCloud2IteratorBase
{
public:
  PointCloud2IteratorBase();
  PointCloud2IteratorBase(C &cloud_msg, const std::string &field_name);
  V<T>& operator =(const V<T>& iter);
  TT& operator [](size_t i) const;
  TT& operator *() const;
  V<T>& operator ++();
  V<T> operator +(int i);
  V<T>& operator +=(int i);
  bool operator !=(const V<T>& iter) const;
  V<T> end() const;

private:
  int set_field(const PointCloud2 &cloud_msg, const std::string &field_name);

  int point_step_;
  U* data_char_;
  TT* data_;
  TT* data_end_;
  bool is_bigendian_;
};
} // namespace impl

template<typename T>
class PointCloud2Iterator : public impl::PointCloud2IteratorBase<T, T, unsigned char, PointCloud2, PointCloud2Iterator>
{
public:
  PointCloud2Iterator(PointCloud2 &cloud_msg, const std::string &field_name) :
    impl::PointCloud2IteratorBase<T, T, unsigned char, PointCloud2, PointCloud2Iterator>::PointCloud2IteratorBase(cloud_msg, field_name) {}
};

} // namespace cartographer_generic_msgs

#include "cartographer_generic_msgs/impl/PointCloud2Iterator.h"
#endif // CARTOGRAPHER_GENERIC_MSGS_POINTCLOUD2ITERATOR_H
