#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <limits>

namespace traversable_terrain {

struct PreprocessingParams {
  // Voxel grid
  float voxel_size = 0.1f;

  // Distance filter
  float min_distance = 0.5f;
  float max_distance = 50.0f;

  // Z range (passthrough)
  float min_z = -5.0f;
  float max_z = 10.0f;

  // Statistical outlier removal
  int sor_mean_k = 10;
  float sor_std_thresh = 1.0f;
  bool enable_sor = true;

  // Debug: print per-step point counts and cloud statistics
  bool debug_mode = false;
};

class Preprocessing {
public:
  explicit Preprocessing(const PreprocessingParams& params = PreprocessingParams{});

  void setParams(const PreprocessingParams& params);

  // Set ROS2 logger for debug output
  void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

  // Full preprocessing pipeline
  // Returns filtered cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const;

private:
  // Individual steps
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  passthroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const;

  pcl::PointCloud<pcl::PointXYZI>::Ptr
  voxelDownsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const;

  pcl::PointCloud<pcl::PointXYZI>::Ptr
  distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const;

  pcl::PointCloud<pcl::PointXYZI>::Ptr
  statisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const;

  // Debug: compute cloud statistics (z range, xy range)
  struct CloudStats {
    float z_min, z_max, z_mean;
    float xy_dist_min, xy_dist_max;
    size_t count;
  };
  CloudStats computeStats(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const;

  PreprocessingParams params_;
  rclcpp::Logger logger_ = rclcpp::get_logger("preprocessing");
};

}  // namespace traversable_terrain
