#pragma once

#include "traversable_terrain_extractor/types.hpp"
#include "traversable_terrain_extractor/elevation_grid.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

namespace traversable_terrain {

struct FeatureComputerParams {
  float grid_resolution = 0.3f;

  // Stair detection
  float stair_min_step_height = 0.10f;    // meters
  float stair_max_step_height = 0.25f;    // meters
  int stair_min_steps = 2;                // minimum steps to classify as stairs
  float stair_search_length = 3.0f;       // meters, search distance

  // Ramp detection
  float ramp_min_length = 1.5f;           // meters, minimum ramp length
  float ramp_min_slope_deg = 5.0f;        // minimum slope for ramp
  float ramp_max_slope_deg = 25.0f;       // maximum slope for ramp
  float ramp_slope_consistency = 5.0f;    // max slope variation (degrees)

  // Point density
  int min_points_per_cell = 2;

  // Debug: print per-frame feature distribution (normal_z, slope, roughness, etc.)
  bool debug_mode = false;
};

class FeatureComputer {
public:
  explicit FeatureComputer(const FeatureComputerParams& params = FeatureComputerParams{});

  void setParams(const FeatureComputerParams& params);

  void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

  // Compute all features on the elevation grid
  // Modifies grid cells in-place
  void compute(ElevationGrid& grid,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
               const pcl::PointCloud<pcl::Normal>::Ptr& normals) const;

private:
  void computeCellFeatures(ElevationGrid& grid) const;
  void detectStairs(ElevationGrid& grid) const;
  void detectRamps(ElevationGrid& grid) const;

  // Debug: log feature distribution histogram across all valid cells
  void logFeatureStats(const ElevationGrid& grid) const;

  FeatureComputerParams params_;
  rclcpp::Logger logger_ = rclcpp::get_logger("feature_computer");
};

}  // namespace traversable_terrain
