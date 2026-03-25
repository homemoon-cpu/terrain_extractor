#include "traversable_terrain_extractor/occupancy_grid_projector.hpp"

#include <cmath>
#include <algorithm>

namespace traversable_terrain {

OccupancyGridProjector::OccupancyGridProjector(
  const OccupancyGridProjectorParams& params)
  : params_(params) {}

void OccupancyGridProjector::setParams(const OccupancyGridProjectorParams& params) {
  params_ = params;
}

nav_msgs::msg::OccupancyGrid
OccupancyGridProjector::project(
  const ElevationGrid& elevation_grid,
  const Eigen::Vector3f& robot_position) const {

  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = params_.frame_id;

  const float half_range = params_.range;
  const float res = params_.resolution;
  const int grid_size = static_cast<int>(2.0f * half_range / res);

  grid_msg.info.resolution = res;
  grid_msg.info.width = grid_size;
  grid_msg.info.height = grid_size;

  // Origin: bottom-left corner
  float origin_x, origin_y;
  if (params_.robot_centric) {
    origin_x = robot_position.x() - half_range;
    origin_y = robot_position.y() - half_range;
  } else {
    origin_x = -half_range;
    origin_y = -half_range;
  }

  grid_msg.info.origin.position.x = origin_x;
  grid_msg.info.origin.position.y = origin_y;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  // Initialize all cells to -1 (unknown)
  grid_msg.data.assign(grid_size * grid_size, -1);

  // Project elevation grid cells onto the occupancy grid
  for (const auto& [cell_idx, cell] : elevation_grid.cells()) {
    Eigen::Vector2f world_pos = elevation_grid.cellToWorld(cell_idx);

    // Map to grid coordinates
    int gx = static_cast<int>((world_pos.x() - origin_x) / res);
    int gy = static_cast<int>((world_pos.y() - origin_y) / res);

    if (gx < 0 || gx >= grid_size || gy < 0 || gy >= grid_size) {
      continue;
    }

    int grid_idx = gy * grid_size + gx;
    int8_t value = terrainToOccupancy(cell.classification);

    // If multiple elevation cells map to same grid cell, use worst case
    if (grid_msg.data[grid_idx] == -1) {
      grid_msg.data[grid_idx] = value;
    } else {
      grid_msg.data[grid_idx] = std::max(grid_msg.data[grid_idx], value);
    }
  }

  return grid_msg;
}

int8_t OccupancyGridProjector::terrainToOccupancy(TerrainClass tc) {
  switch (tc) {
    case TerrainClass::GROUND:           return 0;
    case TerrainClass::RAMP:             return 25;
    case TerrainClass::STAIRS:           return 50;
    case TerrainClass::NON_TRAVERSABLE:  return 100;
    case TerrainClass::UNKNOWN:
    default:                             return -1;
  }
}

}  // namespace traversable_terrain
