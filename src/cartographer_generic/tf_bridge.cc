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

#include "cartographer/common/make_unique.h"

#include "cartographer_generic/msg_conversion.h"
#include "cartographer_generic/tf_bridge.h"

namespace cartographer_generic {

TfBridge::TfBridge(const string& tracking_frame,
		const double lookup_transform_timeout_sec/*,
		const tf2_ros::Buffer* buffer*/)
: tracking_frame_(tracking_frame),
  lookup_transform_timeout_sec_(lookup_transform_timeout_sec)/*,
  buffer_(buffer)*/ {}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
		const ::cartographer::common::Time time, const string& frame_id) const {

	//::ros::Duration timeout(lookup_transform_timeout_sec_);

	//std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking; NOT USED !!!
	//	try {
	//		//TODO Time_out
	//		//    const ::ros::Time latest_tf_time =
	//		//        buffer_
	//		//            ->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.),
	//		//                              timeout)
	//		//            .header.stamp;
	//		//    const ::ros::Time requested_time = ToRos(time);
	//		//    if (latest_tf_time >= requested_time) {
	//		//      // We already have newer data, so we do not wait. Otherwise, we would wait
	//		//      // for the full 'timeout' even if we ask for data that is too old.
	//		//      timeout = ::ros::Duration(0.);
	//		//	}
	//		return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id, requested_time, timeout)));
	//	} catch (const tf2::TransformException& ex) {
	//		LOG(WARNING) << ex.what();
	//	}
	//	return nullptr;

//	if(frame_id == "buddy_tablet"){
//		::cartographer::transform::Rigid3<double>::Vector  Translation(0.107459,-0.00871557,0.412804);
//		::cartographer::transform::Rigid3<double>::Quaternion Rotation(0.00289275,-0.0662549,-0.0435233,0.996849);
//		::cartographer::transform::Rigid3<double> Transform (Translation, Rotation);
//		std::stringstream ss;
//		ss <<"Transform " << tracking_frame_ << "to " <<  frame_id << ": { Pos(" << Transform.translation().x() << "," << Transform.translation().y()  << "," <<  Transform.translation().z() << ")";
//		ss<< "Orie(" << Transform.rotation().x() << "," << Transform.rotation().y()  << "," <<  Transform.rotation().z()  << "," << Transform.rotation().w() << ")";
//		LOG(INFO) << ss.str() ;
//		return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(Transform);
//	}
//	else {
//		::cartographer::transform::Rigid3<double> Transform = ::cartographer::transform::Rigid3d::Identity();
//		std::stringstream ss;
//		ss <<"Transform " << tracking_frame_ << "to " <<  frame_id << ": { Pos(" << Transform.translation().x() << "," << Transform.translation().y()  << "," <<  Transform.translation().z() << ")";
//		ss<< "Orie(" << Transform.rotation().x() << "," << Transform.rotation().y()  << "," <<  Transform.rotation().z()  << "," << Transform.rotation().w() << ")";
//		LOG(INFO) << ss.str() ;
//		return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(Transform);
//	}
	::cartographer::transform::Rigid3<double> Transform = ::cartographer::transform::Rigid3d::Identity();
	return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(Transform);
}

}  // namespace cartographer_generic
