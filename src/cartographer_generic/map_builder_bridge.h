#ifndef CARTOGRAPHER_GENERIC_MAP_BUILDER_BRIDGE_H_
#define CARTOGRAPHER_GENERIC_MAP_BUILDER_BRIDGE_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_generic/node_options.h"
#include "cartographer_generic/sensor_bridge.h"
#include "cartographer_generic/tf_bridge.h"
#include "cartographer_generic/trajectory_options.h"

namespace cartographer_generic {

class MapBuilderBridge {
 public:
  struct TrajectoryState {
    cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate;
    cartographer::transform::Rigid3d local_to_map;
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;
  };

  MapBuilderBridge(const NodeOptions& node_options/*, tf2_ros::Buffer* tf_buffer*/);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  //void LoadMap(const std::string& map_filename);
  int AddTrajectory(const std::unordered_set<string>& expected_sensor_ids,
                    const TrajectoryOptions& trajectory_options);
  void FinishTrajectory(int trajectory_id);

  //cartographer_generic_msgs::SubmapList GetSubmapList();
  //std::unordered_map<int, TrajectoryState> GetTrajectoryStates();
  //visualization_msgs::MarkerArray GetTrajectoryNodeList();
  //visualization_msgs::MarkerArray GetConstraintList();

  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  const NodeOptions node_options_;
  cartographer::mapping::MapBuilder map_builder_;
  //tf2_ros::Buffer* const tf_buffer_;

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
};

}  // namespace cartographer_generic

#endif  // CARTOGRAPHER_GENERIC_MAP_BUILDER_BRIDGE_H_
