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

#include "cartographer/io/xray_points_processor.h"

#include <cmath>
#include <string>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/detect_floors.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/transform/transform.h"

#include "cartographer/to_string.h"

namespace cartographer {
namespace io {
namespace {

struct PixelData {
  size_t num_occupied_cells_in_column = 0;
  double mean_r = 0.;
  double mean_g = 0.;
  double mean_b = 0.;
};

using PixelDataMatrix =
    Eigen::Matrix<PixelData, Eigen::Dynamic, Eigen::Dynamic>;

double Mix(const double a, const double b, const double t) {
  return a * (1. - t) + t * b;
}

// Write 'mat' as a pleasing-to-look-at PNG into 'filename'
Image IntoImage(const PixelDataMatrix& mat) {
  Image image(mat.cols(), mat.rows());
  float max = std::numeric_limits<float>::min();
  for (int y = 0; y < mat.rows(); ++y) {
    for (int x = 0; x < mat.cols(); ++x) {
      const PixelData& cell = mat(y, x);
      if (cell.num_occupied_cells_in_column == 0.) {
        continue;
      }
      max = std::max<float>(max, std::log(cell.num_occupied_cells_in_column));
    }
  }

  for (int y = 0; y < mat.rows(); ++y) {
    for (int x = 0; x < mat.cols(); ++x) {
      const PixelData& cell = mat(y, x);
      if (cell.num_occupied_cells_in_column == 0.) {
        image.SetPixel(x, y, {{255, 255, 255}});
        continue;
      }

      // We use a logarithmic weighting for how saturated a pixel will be. The
      // basic idea here was that walls (full height) are fully saturated, but
      // details like chairs and tables are still well visible.
      const float saturation =
          std::log(cell.num_occupied_cells_in_column) / max;
      double mean_r_in_column = (cell.mean_r / 255.);
      double mean_g_in_column = (cell.mean_g / 255.);
      double mean_b_in_column = (cell.mean_b / 255.);

      double mix_r = Mix(1., mean_r_in_column, saturation);
      double mix_g = Mix(1., mean_g_in_column, saturation);
      double mix_b = Mix(1., mean_b_in_column, saturation);

      const uint8_t r = common::RoundToInt(mix_r * 255.);
      const uint8_t g = common::RoundToInt(mix_g * 255.);
      const uint8_t b = common::RoundToInt(mix_b * 255.);
      image.SetPixel(x, y, {{r, g, b}});
    }
  }
  return image;
}

bool ContainedIn(const common::Time& time,
                 const std::vector<mapping::Timespan>& timespans) {
  for (const mapping::Timespan& timespan : timespans) {
    if (timespan.start <= time && time <= timespan.end) {
      return true;
    }
  }
  return false;
}

}  // namespace

XRayPointsProcessor::XRayPointsProcessor(
    const double voxel_size, const transform::Rigid3f& transform,
    const std::vector<mapping::Floor>& floors, const string& output_filename,
    FileWriterFactory file_writer_factory, PointsProcessor* const next)
    : next_(next),
      file_writer_factory_(file_writer_factory),
      floors_(floors),
      output_filename_(output_filename),
      transform_(transform) {
  for (size_t i = 0; i < (floors_.empty() ? 1 : floors.size()); ++i) {
    aggregations_.emplace_back(
        Aggregation{mapping_3d::HybridGridBase<bool>(voxel_size), {}});
  }
}

std::unique_ptr<XRayPointsProcessor> XRayPointsProcessor::FromDictionary(
    const mapping::proto::Trajectory& trajectory,
    FileWriterFactory file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  std::vector<mapping::Floor> floors;
  if (dictionary->HasKey("separate_floors") &&
      dictionary->GetBool("separate_floors")) {
    floors = mapping::DetectFloors(trajectory);
  }

  return common::make_unique<XRayPointsProcessor>(
      dictionary->GetDouble("voxel_size"),
      transform::FromDictionary(dictionary->GetDictionary("transform").get())
          .cast<float>(),
      floors, dictionary->GetString("filename"), file_writer_factory, next);
}

void XRayPointsProcessor::WriteVoxels(const Aggregation& aggregation,
                                      FileWriter* const file_writer) {
  if (bounding_box_.isEmpty()) {
    LOG(WARNING) << "Not writing output: bounding box is empty.";
    return;
  }

  // Returns the (x, y) pixel of the given 'index'.
  const auto voxel_index_to_pixel = [this](const Eigen::Array3i& index) {
    // We flip the y axis, since matrices rows are counted from the top.
    return Eigen::Array2i(bounding_box_.max()[1] - index[1],
                          bounding_box_.max()[2] - index[2]);
  };

  // Hybrid grid uses X: forward, Y: left, Z: up.
  // For the screen we are using. X: right, Y: up
  const int xsize = bounding_box_.sizes()[1] + 1;
  const int ysize = bounding_box_.sizes()[2] + 1;
  PixelDataMatrix pixel_data_matrix = PixelDataMatrix(ysize, xsize);
  for (mapping_3d::HybridGridBase<bool>::Iterator it(aggregation.voxels);
       !it.Done(); it.Next()) {
    const Eigen::Array3i cell_index = it.GetCellIndex();
    const Eigen::Array2i pixel = voxel_index_to_pixel(cell_index);
    PixelData& pixel_data = pixel_data_matrix(pixel.y(), pixel.x());
    const auto& column_data = aggregation.column_data.at(
        std::make_pair(cell_index[1], cell_index[2]));
    pixel_data.mean_r = column_data.sum_r / column_data.count;
    pixel_data.mean_g = column_data.sum_g / column_data.count;
    pixel_data.mean_b = column_data.sum_b / column_data.count;
    ++pixel_data.num_occupied_cells_in_column;
  }
  Image image = IntoImage(pixel_data_matrix);

  // TODO(hrapp): Draw trajectories here.

  image.WritePng(file_writer);
  CHECK(file_writer->Close());
}

void XRayPointsProcessor::Insert(const PointsBatch& batch,
                                 Aggregation* const aggregation) {
  constexpr Color kDefaultColor = {{0, 0, 0}};
  for (size_t i = 0; i < batch.points.size(); ++i) {
    const Eigen::Vector3f camera_point = transform_ * batch.points[i];
    const Eigen::Array3i cell_index =
        aggregation->voxels.GetCellIndex(camera_point);
    *aggregation->voxels.mutable_value(cell_index) = true;
    bounding_box_.extend(cell_index.matrix());
    ColumnData& column_data =
        aggregation->column_data[std::make_pair(cell_index[1], cell_index[2])];
    const auto& color =
        batch.colors.empty() ? kDefaultColor : batch.colors.at(i);
    column_data.sum_r += color[0];
    column_data.sum_g += color[1];
    column_data.sum_b += color[2];
    ++column_data.count;
  }
}

void XRayPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (floors_.empty()) {
    CHECK_EQ(aggregations_.size(), 1);
    Insert(*batch, &aggregations_[0]);
  } else {
    for (size_t i = 0; i < floors_.size(); ++i) {
      if (!ContainedIn(batch->time, floors_[i].timespans)) {
        continue;
      }
      Insert(*batch, &aggregations_[i]);
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult XRayPointsProcessor::Flush() {
  if (floors_.empty()) {
    CHECK_EQ(aggregations_.size(), 1);
    WriteVoxels(aggregations_[0],
                file_writer_factory_(output_filename_ + ".png").get());
  } else {
    for (size_t i = 0; i < floors_.size(); ++i) {
      WriteVoxels(
          aggregations_[i],
          file_writer_factory_(output_filename_ + to_string(i) + ".png")
              .get());
    }
  }

  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "X-Ray generation must be configured to occur after any "
                    "stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer
