/*
 * test_header.h
 *
 *  Created on: Aug 7, 2017
 *      Author: maghob
 */

#ifndef SRC_CARTOGRAPHER_GENERIC_TEST_HERITAGE_H_
#define SRC_CARTOGRAPHER_GENERIC_TEST_HERITAGE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/proto/submaps_options.pb.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include <glog/logging.h>
using namespace std;

namespace cartographer {
namespace mapping {

class  test_heritage {

public:
	test_heritage(const transform::Rigid3d& local_pose): local_pose_(local_pose)
{
		LOG(INFO) << local_pose_.DebugString();
		LOG(INFO) << "YES";
}
	// ADDED BY MAH
	virtual ~test_heritage(){};

	virtual void ToProto(proto::Submap* proto) const = 0;

	// Local SLAM pose of this submap.
	transform::Rigid3d local_pose() const { return local_pose_; }

	 // Fills data into the 'response'.
/*	  virtual void ToResponseProto(
	      const transform::Rigid3d& global_submap_pose,
	      proto::SubmapQuery::Response* response) const = 0;*/

	// Number of RangeData inserted.
	int num_range_data() const { return num_range_data_; }

protected:
	void SetNumRangeData(const int num_range_data) {
		num_range_data_ = num_range_data;
	}


private:
	const ::cartographer::transform::Rigid3d local_pose_;
	int num_range_data_ = 0;
};

}  // namespace
}


#endif /* SRC_CARTOGRAPHER_GENERIC_TEST_HERITAGE_H_ */
