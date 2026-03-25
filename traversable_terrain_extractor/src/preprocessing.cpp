#include "traversable_terrain_extractor/preprocessing.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace traversable_terrain {

Preprocessing::Preprocessing(const PreprocessingParams& params)
  : params_(params) {}

void Preprocessing::setParams(const PreprocessingParams& params) {
  params_ = params;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
Preprocessing::process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  if (!input || input->empty()) {
    return std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  // Pipeline: PassThrough(Z) → VoxelGrid → DistanceFilter → SOR
  auto filtered = passthroughFilter(input);
  filtered = voxelDownsample(filtered);
  filtered = distanceFilter(filtered);
  if (params_.enable_sor && filtered->size() > 10) {
    filtered = statisticalOutlierRemoval(filtered);
  }
  return filtered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
Preprocessing::passthroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(params_.min_z, params_.max_z);
  pass.filter(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
Preprocessing::voxelDownsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
  voxel.filter(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
Preprocessing::distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  output->reserve(input->size());

  float min_d2 = params_.min_distance * params_.min_distance;
  float max_d2 = params_.max_distance * params_.max_distance;

  for (const auto& pt : *input) {
    float d2 = pt.x * pt.x + pt.y * pt.y;  // 2D distance (horizontal)
    if (d2 >= min_d2 && d2 <= max_d2) {
      output->push_back(pt);
    }
  }

  output->width = output->size();
  output->height = 1;
  output->is_dense = true;
  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
Preprocessing::statisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(input);
  sor.setMeanK(params_.sor_mean_k);
  sor.setStddevMulThresh(params_.sor_std_thresh);
  sor.filter(*output);
  return output;
}

}  // namespace traversable_terrain
