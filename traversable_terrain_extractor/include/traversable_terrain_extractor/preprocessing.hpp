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

  // Z range — relative to sensor position (set via sensor_z in process())
  // PassThrough will use [sensor_z - z_offset_below, sensor_z + z_offset_above]
  float z_offset_below = 3.0f;   // meters below sensor to keep
  float z_offset_above = 1.0f;   // meters above sensor to keep

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
  // sensor_z: current sensor height in map frame (from odometry)
  // PassThrough Z window = [sensor_z - z_offset_below, sensor_z + z_offset_above]
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
          float sensor_z = 0.0f) const;

private:
  // Individual steps
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  passthroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
                    float z_min, float z_max) const;

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
