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
