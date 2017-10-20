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
