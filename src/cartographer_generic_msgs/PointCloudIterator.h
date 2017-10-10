#ifndef CARTOGRAPHER_GENERIC_MSGS_POINTCLOUDITERATOR_H
#define CARTOGRAPHER_GENERIC_MSGS_POINTCLOUDITERATOR_H

#include "cartographer_generic_msgs/PointCloud.h"
#include <string>
#include <vector>

namespace cartographer_generic_msgs
{

namespace impl
{
template<typename T, typename TT, typename U, typename C, template <typename> class V>
class PointCloudIteratorBase
{
public:
  PointCloudIteratorBase();
  PointCloudIteratorBase(C &cloud_msg, const std::string &field_name);
  V<T>& operator =(const V<T>& iter);
  TT& operator [](size_t i) const;
  TT& operator *() const;
  V<T>& operator ++();
  V<T> operator +(int i);
  V<T>& operator +=(int i);
  bool operator !=(const V<T>& iter) const;
  V<T> end() const;

private:
  int set_field(const PointCloud &cloud_msg, const std::string &field_name);

  int point_step_;
  U* data_char_;
  TT* data_;
  TT* data_end_;
  bool is_bigendian_;
};
} // namespace impl

template<typename T>
class PointCloudIterator : public impl::PointCloudIteratorBase<T, T, unsigned char, PointCloud, PointCloudIterator>
{
public:
  PointCloudIterator(PointCloud &cloud_msg, const std::string &field_name) :
    impl::PointCloudIteratorBase<T, T, unsigned char, PointCloud, PointCloudIterator>::PointCloudIteratorBase(cloud_msg, field_name) {}
};

} // namespace cartographer_generic_msgs

#include "cartographer_generic_msgs/impl/PointCloudIterator.h"
#endif // CARTOGRAPHER_GENERIC_MSGS_POINTCLOUDITERATOR_H
