#include "traversable_terrain_extractor/preprocessing.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cmath>
#include <limits>

namespace traversable_terrain {

Preprocessing::Preprocessing(const PreprocessingParams& params)
  : params_(params) {}

void Preprocessing::setParams(const PreprocessingParams& params) {
  params_ = params;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
Preprocessing::process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  if (!input || input->empty()) {
    RCLCPP_WARN(logger_, "[Preprocessing] Input cloud is null or empty");
    return std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  // Print input stats (always shown, helps identify upstream issues)
  if (params_.debug_mode) {
    auto stats = computeStats(input);
    RCLCPP_INFO(logger_,
      "[Preprocessing] INPUT  : %zu pts | z=[%.2f, %.2f] mean=%.2f | "
      "xy_dist=[%.2f, %.2f] | params z=[%.2f,%.2f] dist=[%.2f,%.2f]",
      stats.count, stats.z_min, stats.z_max, stats.z_mean,
      stats.xy_dist_min, stats.xy_dist_max,
      params_.min_z, params_.max_z,
      params_.min_distance, params_.max_distance);
  }

  // ---- Step 1: PassThrough(Z) ----
  auto after_passthrough = passthroughFilter(input);
  if (params_.debug_mode) {
    RCLCPP_INFO(logger_,
      "[Preprocessing] Step1 PassThrough(z=[%.2f,%.2f]): %zu → %zu pts%s",
      params_.min_z, params_.max_z,
      input->size(), after_passthrough->size(),
      after_passthrough->empty() ? "  *** EMPTY ***" : "");
  }
  if (after_passthrough->empty()) {
    auto stats = computeStats(input);
    RCLCPP_ERROR(logger_,
      "[Preprocessing] PassThrough eliminated ALL points! "
      "Cloud z range=[%.2f, %.2f], filter z=[%.2f, %.2f]. "
      "Fix: adjust min_z/max_z in terrain_params.yaml",
      stats.z_min, stats.z_max, params_.min_z, params_.max_z);
    return after_passthrough;
  }

  // ---- Step 2: VoxelGrid ----
  auto after_voxel = voxelDownsample(after_passthrough);
  if (params_.debug_mode) {
    RCLCPP_INFO(logger_,
      "[Preprocessing] Step2 VoxelGrid(leaf=%.3fm): %zu → %zu pts%s",
      params_.voxel_size,
      after_passthrough->size(), after_voxel->size(),
      after_voxel->empty() ? "  *** EMPTY ***" : "");
  }
  if (after_voxel->empty()) {
    RCLCPP_ERROR(logger_,
      "[Preprocessing] VoxelGrid eliminated ALL points! "
      "leaf_size=%.3f m, input had %zu pts. "
      "Fix: reduce voxel_size in terrain_params.yaml",
      params_.voxel_size, after_passthrough->size());
    return after_voxel;
  }

  // ---- Step 3: Distance Filter ----
  auto after_dist = distanceFilter(after_voxel);
  if (params_.debug_mode) {
    auto stats = computeStats(after_voxel);
    RCLCPP_INFO(logger_,
      "[Preprocessing] Step3 DistFilter(xy=[%.2f,%.2f]m): %zu → %zu pts%s"
      " | cloud xy_dist=[%.2f, %.2f]",
      params_.min_distance, params_.max_distance,
      after_voxel->size(), after_dist->size(),
      after_dist->empty() ? "  *** EMPTY ***" : "",
      stats.xy_dist_min, stats.xy_dist_max);
  }
  if (after_dist->empty()) {
    auto stats = computeStats(after_voxel);
    RCLCPP_ERROR(logger_,
      "[Preprocessing] DistanceFilter eliminated ALL points! "
      "Cloud xy_dist=[%.2f, %.2f], filter=[%.2f, %.2f]. "
      "Fix: adjust min_distance/max_distance in terrain_params.yaml",
      stats.xy_dist_min, stats.xy_dist_max,
      params_.min_distance, params_.max_distance);
    return after_dist;
  }

  // ---- Step 4: SOR ----
  if (!params_.enable_sor) {
    if (params_.debug_mode) {
      RCLCPP_INFO(logger_, "[Preprocessing] Step4 SOR: disabled, final=%zu pts",
                  after_dist->size());
    }
    return after_dist;
  }
  if (after_dist->size() <= 10) {
    RCLCPP_WARN(logger_,
      "[Preprocessing] Step4 SOR: skipped (only %zu pts, need >10)",
      after_dist->size());
    return after_dist;
  }

  auto after_sor = statisticalOutlierRemoval(after_dist);
  if (params_.debug_mode) {
    RCLCPP_INFO(logger_,
      "[Preprocessing] Step4 SOR(k=%d, std=%.2f): %zu → %zu pts%s",
      params_.sor_mean_k, params_.sor_std_thresh,
      after_dist->size(), after_sor->size(),
      after_sor->empty() ? "  *** EMPTY ***" : "");
  }
  if (after_sor->empty()) {
    RCLCPP_ERROR(logger_,
      "[Preprocessing] SOR eliminated ALL points! "
      "k=%d, std_thresh=%.2f, input=%zu pts. "
      "Fix: increase sor_std_thresh or decrease sor_mean_k, or set enable_sor=false",
      params_.sor_mean_k, params_.sor_std_thresh, after_dist->size());
    return after_sor;
  }

  if (params_.debug_mode) {
    RCLCPP_INFO(logger_, "[Preprocessing] DONE: %zu → %zu pts (%.1f%% kept)",
      input->size(), after_sor->size(),
      100.0f * after_sor->size() / input->size());
  }

  return after_sor;
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

Preprocessing::CloudStats
Preprocessing::computeStats(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const {
  CloudStats stats{};
  stats.count = cloud->size();

  if (cloud->empty()) {
    stats.z_min = stats.z_max = stats.z_mean = 0.0f;
    stats.xy_dist_min = stats.xy_dist_max = 0.0f;
    return stats;
  }

  stats.z_min = std::numeric_limits<float>::max();
  stats.z_max = std::numeric_limits<float>::lowest();
  stats.xy_dist_min = std::numeric_limits<float>::max();
  stats.xy_dist_max = 0.0f;

  double z_sum = 0.0;
  for (const auto& pt : *cloud) {
    stats.z_min = std::min(stats.z_min, pt.z);
    stats.z_max = std::max(stats.z_max, pt.z);
    z_sum += pt.z;

    float xy_dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    stats.xy_dist_min = std::min(stats.xy_dist_min, xy_dist);
    stats.xy_dist_max = std::max(stats.xy_dist_max, xy_dist);
  }
  stats.z_mean = static_cast<float>(z_sum / cloud->size());

  return stats;
}

}  // namespace traversable_terrain
