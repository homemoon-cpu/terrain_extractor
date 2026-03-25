#pragma once

#include "traversable_terrain_extractor/types.hpp"
#include "traversable_terrain_extractor/elevation_grid.hpp"
#include <rclcpp/rclcpp.hpp>

namespace traversable_terrain {

struct TerrainClassifierParams {
  // Ground thresholds
  float ground_max_slope_deg = 15.0f;
  float ground_max_roughness = 0.05f;
  float ground_max_height_var = 0.03f;
  float ground_max_curvature = 0.1f;

  // Stair thresholds
  float stair_min_slope_deg = 20.0f;
  float stair_max_slope_deg = 50.0f;

  // Ramp thresholds
  float ramp_min_slope_deg = 5.0f;
  float ramp_max_slope_deg = 25.0f;

  // Traversability
  float max_traversable_slope_deg = 30.0f;
  bool stairs_traversable = true;
  bool ramps_traversable = true;

  // Minimum observations for classification
  int min_observations = 2;

  // Debug: log per-cell rejection reason + per-frame statistics
  bool debug_mode = false;
};

class TerrainClassifier {
public:
  explicit TerrainClassifier(const TerrainClassifierParams& params = TerrainClassifierParams{});

  void setParams(const TerrainClassifierParams& params);

  void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

  // Classify all cells in the elevation grid
  void classify(ElevationGrid& grid) const;

  // Classify a single cell
  TerrainClass classifyCell(const ElevationCell& cell) const;

  // Determine traversability for a given classification
  bool isTraversable(TerrainClass tc, float slope_deg) const;

private:
  TerrainClassifierParams params_;
  rclcpp::Logger logger_ = rclcpp::get_logger("terrain_classifier");
};

}  // namespace traversable_terrain
