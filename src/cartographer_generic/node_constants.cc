/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_generic/node_constants.h"

#include "glog/logging.h"
#include "cartographer/to_string.h"

namespace cartographer_generic {

std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   const int num_topics) {
  //CHECK_GE(num_topics, 0);
  if (num_topics == 1) {
    return {topic};
  }
  std::vector<std::string> topics;
  topics.reserve(num_topics);
  for (int i = 0; i < num_topics; ++i) {
    topics.emplace_back(topic + "_" + to_string(i + 1));
  }
  return topics;
}

std::vector<double>  GetPoseEstimate(){
	return pose_estimate;
}

void SetPoseEstimate(double* pose_ros){
	pose_estimate.clear();
	for(int i=0; i<7; i++)
			pose_estimate.push_back(pose_ros[i]);
}

}  // namespace cartographer_generic
