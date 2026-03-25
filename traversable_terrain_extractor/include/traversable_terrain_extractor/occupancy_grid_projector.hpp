#pragma once

#include "traversable_terrain_extractor/types.hpp"
#include "traversable_terrain_extractor/elevation_grid.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Dense>

namespace traversable_terrain {

struct OccupancyGridProjectorParams {
  float resolution = 0.2f;     // Grid resolution in meters
  float range = 100.0f;        // Half-width of the grid
  bool robot_centric = false;  // If true, center on robot position
  std::string frame_id = "map";
};

class OccupancyGridProjector {
public:
  explicit OccupancyGridProjector(
    const OccupancyGridProjectorParams& params = OccupancyGridProjectorParams{});

  void setParams(const OccupancyGridProjectorParams& params);

  // Project elevation grid to 2D occupancy grid
  nav_msgs::msg::OccupancyGrid
  project(const ElevationGrid& elevation_grid,
          const Eigen::Vector3f& robot_position = Eigen::Vector3f::Zero()) const;

  // Convert terrain class to occupancy value
  static int8_t terrainToOccupancy(TerrainClass tc);

private:
  OccupancyGridProjectorParams params_;
};

}  // namespace traversable_terrain
