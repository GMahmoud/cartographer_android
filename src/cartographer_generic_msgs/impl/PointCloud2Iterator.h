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

#ifndef CARTOGRAPHER_GENERIC_MSGS_IMPL_POINTCLOUD2ITERATOR_H
#define CARTOGRAPHER_GENERIC_MSGS_IMPL_POINTCLOUD2ITERATOR_H

#include <string>
#include <vector>
#include "cartographer_generic_msgs/PointCloud2.h"


namespace cartographer_generic_msgs
{
namespace impl
{

template<typename T, typename TT, typename U, typename C, template <typename> class V>
PointCloud2IteratorBase<T, TT, U, C, V>::PointCloud2IteratorBase() : data_char_(0), data_(0), data_end_(0)
{
}


template<typename T, typename TT, typename U, typename C, template <typename> class V>
PointCloud2IteratorBase<T, TT, U, C, V>::PointCloud2IteratorBase(C &cloud_msg, const std::string &field_name)
{
  int offset = set_field(cloud_msg, field_name);

  data_char_ = &(cloud_msg.data.front()) + offset;
  data_ = reinterpret_cast<TT*>(data_char_);
  data_end_ = reinterpret_cast<TT*>(&(cloud_msg.data.back()) + 1 + offset);
}


template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator =(const V<T> &iter)
{
  if (this != &iter)
  {
    point_step_ = iter.point_step_;
    data_char_ = iter.data_char_;
    data_ = iter.data_;
    data_end_ = iter.data_end_;
    is_bigendian_ = iter.is_bigendian_;
  }

  return *this;
}


template<typename T, typename TT, typename U, typename C, template <typename> class V>
TT& PointCloud2IteratorBase<T, TT, U, C, V>::operator [](size_t i) const
{
  return *(data_ + i);
}

template<typename T, typename TT, typename U, typename C, template <typename> class V>
TT& PointCloud2IteratorBase<T, TT, U, C, V>::operator *() const
{
  return *data_;
}

template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator ++()
{
  data_char_ += point_step_;
  data_ = reinterpret_cast<TT*>(data_char_);
  return *static_cast<V<T>*>(this);
}


template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T> PointCloud2IteratorBase<T, TT, U, C, V>::operator +(int i)
{
  V<T> res = *static_cast<V<T>*>(this);

  res.data_char_ += i*point_step_;
  res.data_ = reinterpret_cast<TT*>(res.data_char_);

  return res;
}

template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator +=(int i)
{
  data_char_ += i*point_step_;
  data_ = reinterpret_cast<TT*>(data_char_);
  return *static_cast<V<T>*>(this);
}

template<typename T, typename TT, typename U, typename C, template <typename> class V>
bool PointCloud2IteratorBase<T, TT, U, C, V>::operator !=(const V<T>& iter) const
{
  return iter.data_ != data_;
}


template<typename T, typename TT, typename U, typename C, template <typename> class V>
V<T> PointCloud2IteratorBase<T, TT, U, C, V>::end() const
{
  V<T> res = *static_cast<const V<T>*>(this);
  res.data_ = data_end_;
  return res;
}

template<typename T, typename TT, typename U, typename C, template <typename> class V>
int PointCloud2IteratorBase<T, TT, U, C, V>::set_field(const PointCloud2 &cloud_msg, const std::string &field_name)
{
  is_bigendian_ = cloud_msg.is_bigendian;
  point_step_ = cloud_msg.point_step;
  // make sure the channel is valid
  std::vector<PointField>::const_iterator field_iter = cloud_msg.fields.begin(), field_end =
      cloud_msg.fields.end();
  while ((field_iter != field_end) && (field_iter->name != field_name))
    ++field_iter;

  if (field_iter == field_end) {
    // Handle the special case of r,g,b,a (we assume they are understood as the channels of an rgb or rgba field)
    if ((field_name == "r") || (field_name == "g") || (field_name == "b") || (field_name == "a"))
    {
      // Check that rgb or rgba is present
      field_iter = cloud_msg.fields.begin();
      while ((field_iter != field_end) && (field_iter->name != "rgb") && (field_iter->name != "rgba"))
        ++field_iter;
      if (field_iter == field_end)
        throw std::runtime_error("Field " + field_name + " does not exist");
      if (field_name == "r")
      {
        if (is_bigendian_)
          return field_iter->offset + 1;
        else
          return field_iter->offset + 2;
      }
      if (field_name == "g")
      {
        if (is_bigendian_)
          return field_iter->offset + 2;
        else
          return field_iter->offset + 1;
      }
      if (field_name == "b")
      {
        if (is_bigendian_)
          return field_iter->offset + 3;
        else
          return field_iter->offset + 0;
      }
      if (field_name == "a")
      {
        if (is_bigendian_)
          return field_iter->offset + 0;
        else
          return field_iter->offset + 3;
      }
    } else
      throw std::runtime_error("Field " + field_name + " does not exist");
  }

  return field_iter->offset;
}

} // namespace impl
} // namespace cartographer_generic_msgs

#endif //CARTOGRAPHER_GENERIC_MSGS_IMPL_POINTCLOUD2ITERATOR_H
