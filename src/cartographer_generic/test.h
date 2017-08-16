#ifndef SRC_CARTOGRAPHER_GENERIC_TEST_H_
#define SRC_CARTOGRAPHER_GENERIC_TEST_H_
#include "cartographer_generic/test_heritage.h"
//#include <google/protobuf/message_lite.h>
/*#include <google/protobuf/message.h>*/
//#include "cartographer/mapping/trajectory_builder.h"
//#include "cartographer/sensor/imu_data.h"
//#include "cartographer/transform/rigid_transform.h"
//#include "cartographer/transform/transform.h"
//#include "tf_bridge.h"
//#include "cartographer_generic_msgs/LaserScan.h"
//#include "cartographer_generic_msgs/Odometry.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"


#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/sparse_pose_graph/proto/optimization_problem_options.pb.h"
#include "cartographer/sensor/imu_data.h"

namespace cartographer {
namespace mapping_3d {
namespace sparse_pose_graph {

struct NodeDataTest {
  common::Time time;
  transform::Rigid3d point_cloud_pose;
};

struct SubmapDataTest {
  transform::Rigid3d pose;
};

// Implements the SPA loop closure method.
class test {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint;

  enum class FixZ { kYes, kNo };

  test(
      const mapping::sparse_pose_graph::proto::OptimizationProblemOptions&
          options,
      FixZ fix_z);
  ~test();

  test(const test&) = delete;
  test& operator=(const test&) = delete;

  void AddImuData(int trajectory_id, common::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);
  void AddTrajectoryNode(int trajectory_id, common::Time time,
                         const transform::Rigid3d& point_cloud_pose);
  void AddSubmap(int trajectory_id, const transform::Rigid3d& submap_pose);

  void SetMaxNumIterations(int32 max_num_iterations);

  // Computes the optimized poses.
  void Solve(const std::vector<Constraint>& constraints,
             const std::set<int>& frozen_trajectories);

  const std::vector<std::vector<NodeDataTest>>& node_data() const;
  const std::vector<std::vector<SubmapDataTest>>& submap_data() const;

 private:
  struct TrajectoryDataTest {
    double gravity_constant = 9.8;
    std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
  };

  mapping::sparse_pose_graph::proto::OptimizationProblemOptions options_;
  FixZ fix_z_;
  std::vector<std::deque<sensor::ImuData>> imu_data_;
  std::vector<std::vector<NodeDataTest>> node_data_;
  std::vector<std::vector<SubmapDataTest>> submap_data_;
  std::vector<TrajectoryDataTest> trajectory_data_;
};

}  // namespace sparse_pose_graph
}  // namespace mapping_3d
}  // namespace cartographer
#endif /* SRC_CARTOGRAPHER_GENERIC_TEST_H_ */











//ProbabilityGrid ComputeCroppedProbabilityGrid(
//		const ProbabilityGrid& probability_grid);
//
//proto::SubmapsOptions CreateSubmapsOptions(
//		common::LuaParameterDictionary* parameter_dictionary);
//class test : public mapping::Submap {
//
//public:
//	test(const MapLimits& limits, const Eigen::Vector2f& origin);
//	explicit test(const mapping::proto::Submap2D& proto);
//
//	void ToProto(mapping::proto::Submap* proto) const override;
//
//	const ProbabilityGrid& probability_grid() const { return probability_grid_; }
//	bool finished() const { return finished_; }
//
//	void ToResponseProto(
//			const transform::Rigid3d& global_submap_pose,
//			mapping::proto::SubmapQuery::Response* response) const override;
//
//	// Insert 'range_data' into this submap using 'range_data_inserter'. The
//	// submap must not be finished yet.
//	void InsertRangeData(const sensor::RangeData& range_data,
//			const RangeDataInserter& range_data_inserter);
//	void Finish();
//
//private:
//	ProbabilityGrid probability_grid_;
//	bool finished_ = false;
//};
//
//
//class ActiveTests {
//public:
//	explicit ActiveTests(const proto::SubmapsOptions& options);
//
//	ActiveTests(const ActiveTests&) = delete;
//	ActiveTests& operator=(const ActiveTests&) = delete;
//
//	// Returns the index of the newest initialized Submap which can be
//	// used for scan-to-map matching.
//	int matching_index() const;
//
//	// Inserts 'range_data' into the Submap collection.
//	void InsertRangeData(const sensor::RangeData& range_data);
//
//	std::vector<std::shared_ptr<test>> submaps() const;
//
//private:
//	void FinishSubmap();
//	void AddSubmap(const Eigen::Vector2f& origin);
//
//	const proto::SubmapsOptions options_;
//	int matching_submap_index_ = 0;
//	std::vector<std::shared_ptr<test>> submaps_;
//	RangeDataInserter range_data_inserter_;
//};
