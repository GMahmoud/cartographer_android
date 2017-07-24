// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.proto

#ifndef PROTOBUF_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto__INCLUDED
#define PROTOBUF_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {
namespace proto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto();
void protobuf_AssignDesc_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto();
void protobuf_ShutdownFile_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto();

class FastCorrelativeScanMatcherOptions;

// ===================================================================

class FastCorrelativeScanMatcherOptions : public ::google::protobuf::Message {
 public:
  FastCorrelativeScanMatcherOptions();
  virtual ~FastCorrelativeScanMatcherOptions();

  FastCorrelativeScanMatcherOptions(const FastCorrelativeScanMatcherOptions& from);

  inline FastCorrelativeScanMatcherOptions& operator=(const FastCorrelativeScanMatcherOptions& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const FastCorrelativeScanMatcherOptions& default_instance();

  void Swap(FastCorrelativeScanMatcherOptions* other);

  // implements Message ----------------------------------------------

  FastCorrelativeScanMatcherOptions* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const FastCorrelativeScanMatcherOptions& from);
  void MergeFrom(const FastCorrelativeScanMatcherOptions& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional int32 branch_and_bound_depth = 2;
  inline bool has_branch_and_bound_depth() const;
  inline void clear_branch_and_bound_depth();
  static const int kBranchAndBoundDepthFieldNumber = 2;
  inline ::google::protobuf::int32 branch_and_bound_depth() const;
  inline void set_branch_and_bound_depth(::google::protobuf::int32 value);

  // optional int32 full_resolution_depth = 8;
  inline bool has_full_resolution_depth() const;
  inline void clear_full_resolution_depth();
  static const int kFullResolutionDepthFieldNumber = 8;
  inline ::google::protobuf::int32 full_resolution_depth() const;
  inline void set_full_resolution_depth(::google::protobuf::int32 value);

  // optional int32 rotational_histogram_size = 3;
  inline bool has_rotational_histogram_size() const;
  inline void clear_rotational_histogram_size();
  static const int kRotationalHistogramSizeFieldNumber = 3;
  inline ::google::protobuf::int32 rotational_histogram_size() const;
  inline void set_rotational_histogram_size(::google::protobuf::int32 value);

  // optional double min_rotational_score = 4;
  inline bool has_min_rotational_score() const;
  inline void clear_min_rotational_score();
  static const int kMinRotationalScoreFieldNumber = 4;
  inline double min_rotational_score() const;
  inline void set_min_rotational_score(double value);

  // optional double linear_xy_search_window = 5;
  inline bool has_linear_xy_search_window() const;
  inline void clear_linear_xy_search_window();
  static const int kLinearXySearchWindowFieldNumber = 5;
  inline double linear_xy_search_window() const;
  inline void set_linear_xy_search_window(double value);

  // optional double linear_z_search_window = 6;
  inline bool has_linear_z_search_window() const;
  inline void clear_linear_z_search_window();
  static const int kLinearZSearchWindowFieldNumber = 6;
  inline double linear_z_search_window() const;
  inline void set_linear_z_search_window(double value);

  // optional double angular_search_window = 7;
  inline bool has_angular_search_window() const;
  inline void clear_angular_search_window();
  static const int kAngularSearchWindowFieldNumber = 7;
  inline double angular_search_window() const;
  inline void set_angular_search_window(double value);

  // @@protoc_insertion_point(class_scope:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
 private:
  inline void set_has_branch_and_bound_depth();
  inline void clear_has_branch_and_bound_depth();
  inline void set_has_full_resolution_depth();
  inline void clear_has_full_resolution_depth();
  inline void set_has_rotational_histogram_size();
  inline void clear_has_rotational_histogram_size();
  inline void set_has_min_rotational_score();
  inline void clear_has_min_rotational_score();
  inline void set_has_linear_xy_search_window();
  inline void clear_has_linear_xy_search_window();
  inline void set_has_linear_z_search_window();
  inline void clear_has_linear_z_search_window();
  inline void set_has_angular_search_window();
  inline void clear_has_angular_search_window();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::int32 branch_and_bound_depth_;
  ::google::protobuf::int32 full_resolution_depth_;
  double min_rotational_score_;
  double linear_xy_search_window_;
  double linear_z_search_window_;
  double angular_search_window_;
  ::google::protobuf::int32 rotational_histogram_size_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(7 + 31) / 32];

  friend void  protobuf_AddDesc_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto();
  friend void protobuf_AssignDesc_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto();
  friend void protobuf_ShutdownFile_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto();

  void InitAsDefaultInstance();
  static FastCorrelativeScanMatcherOptions* default_instance_;
};
// ===================================================================


// ===================================================================

// FastCorrelativeScanMatcherOptions

// optional int32 branch_and_bound_depth = 2;
inline bool FastCorrelativeScanMatcherOptions::has_branch_and_bound_depth() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void FastCorrelativeScanMatcherOptions::set_has_branch_and_bound_depth() {
  _has_bits_[0] |= 0x00000001u;
}
inline void FastCorrelativeScanMatcherOptions::clear_has_branch_and_bound_depth() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void FastCorrelativeScanMatcherOptions::clear_branch_and_bound_depth() {
  branch_and_bound_depth_ = 0;
  clear_has_branch_and_bound_depth();
}
inline ::google::protobuf::int32 FastCorrelativeScanMatcherOptions::branch_and_bound_depth() const {
  return branch_and_bound_depth_;
}
inline void FastCorrelativeScanMatcherOptions::set_branch_and_bound_depth(::google::protobuf::int32 value) {
  set_has_branch_and_bound_depth();
  branch_and_bound_depth_ = value;
}

// optional int32 full_resolution_depth = 8;
inline bool FastCorrelativeScanMatcherOptions::has_full_resolution_depth() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void FastCorrelativeScanMatcherOptions::set_has_full_resolution_depth() {
  _has_bits_[0] |= 0x00000002u;
}
inline void FastCorrelativeScanMatcherOptions::clear_has_full_resolution_depth() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void FastCorrelativeScanMatcherOptions::clear_full_resolution_depth() {
  full_resolution_depth_ = 0;
  clear_has_full_resolution_depth();
}
inline ::google::protobuf::int32 FastCorrelativeScanMatcherOptions::full_resolution_depth() const {
  return full_resolution_depth_;
}
inline void FastCorrelativeScanMatcherOptions::set_full_resolution_depth(::google::protobuf::int32 value) {
  set_has_full_resolution_depth();
  full_resolution_depth_ = value;
}

// optional int32 rotational_histogram_size = 3;
inline bool FastCorrelativeScanMatcherOptions::has_rotational_histogram_size() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void FastCorrelativeScanMatcherOptions::set_has_rotational_histogram_size() {
  _has_bits_[0] |= 0x00000004u;
}
inline void FastCorrelativeScanMatcherOptions::clear_has_rotational_histogram_size() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void FastCorrelativeScanMatcherOptions::clear_rotational_histogram_size() {
  rotational_histogram_size_ = 0;
  clear_has_rotational_histogram_size();
}
inline ::google::protobuf::int32 FastCorrelativeScanMatcherOptions::rotational_histogram_size() const {
  return rotational_histogram_size_;
}
inline void FastCorrelativeScanMatcherOptions::set_rotational_histogram_size(::google::protobuf::int32 value) {
  set_has_rotational_histogram_size();
  rotational_histogram_size_ = value;
}

// optional double min_rotational_score = 4;
inline bool FastCorrelativeScanMatcherOptions::has_min_rotational_score() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void FastCorrelativeScanMatcherOptions::set_has_min_rotational_score() {
  _has_bits_[0] |= 0x00000008u;
}
inline void FastCorrelativeScanMatcherOptions::clear_has_min_rotational_score() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void FastCorrelativeScanMatcherOptions::clear_min_rotational_score() {
  min_rotational_score_ = 0;
  clear_has_min_rotational_score();
}
inline double FastCorrelativeScanMatcherOptions::min_rotational_score() const {
  return min_rotational_score_;
}
inline void FastCorrelativeScanMatcherOptions::set_min_rotational_score(double value) {
  set_has_min_rotational_score();
  min_rotational_score_ = value;
}

// optional double linear_xy_search_window = 5;
inline bool FastCorrelativeScanMatcherOptions::has_linear_xy_search_window() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void FastCorrelativeScanMatcherOptions::set_has_linear_xy_search_window() {
  _has_bits_[0] |= 0x00000010u;
}
inline void FastCorrelativeScanMatcherOptions::clear_has_linear_xy_search_window() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void FastCorrelativeScanMatcherOptions::clear_linear_xy_search_window() {
  linear_xy_search_window_ = 0;
  clear_has_linear_xy_search_window();
}
inline double FastCorrelativeScanMatcherOptions::linear_xy_search_window() const {
  return linear_xy_search_window_;
}
inline void FastCorrelativeScanMatcherOptions::set_linear_xy_search_window(double value) {
  set_has_linear_xy_search_window();
  linear_xy_search_window_ = value;
}

// optional double linear_z_search_window = 6;
inline bool FastCorrelativeScanMatcherOptions::has_linear_z_search_window() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void FastCorrelativeScanMatcherOptions::set_has_linear_z_search_window() {
  _has_bits_[0] |= 0x00000020u;
}
inline void FastCorrelativeScanMatcherOptions::clear_has_linear_z_search_window() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void FastCorrelativeScanMatcherOptions::clear_linear_z_search_window() {
  linear_z_search_window_ = 0;
  clear_has_linear_z_search_window();
}
inline double FastCorrelativeScanMatcherOptions::linear_z_search_window() const {
  return linear_z_search_window_;
}
inline void FastCorrelativeScanMatcherOptions::set_linear_z_search_window(double value) {
  set_has_linear_z_search_window();
  linear_z_search_window_ = value;
}

// optional double angular_search_window = 7;
inline bool FastCorrelativeScanMatcherOptions::has_angular_search_window() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void FastCorrelativeScanMatcherOptions::set_has_angular_search_window() {
  _has_bits_[0] |= 0x00000040u;
}
inline void FastCorrelativeScanMatcherOptions::clear_has_angular_search_window() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void FastCorrelativeScanMatcherOptions::clear_angular_search_window() {
  angular_search_window_ = 0;
  clear_has_angular_search_window();
}
inline double FastCorrelativeScanMatcherOptions::angular_search_window() const {
  return angular_search_window_;
}
inline void FastCorrelativeScanMatcherOptions::set_angular_search_window(double value) {
  set_has_angular_search_window();
  angular_search_window_ = value;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto__INCLUDED
