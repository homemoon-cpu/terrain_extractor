#include "traversable_terrain_extractor/feature_computer.hpp"

#include <cmath>
#include <queue>
#include <set>
#include <algorithm>
#include <Eigen/Dense>

namespace traversable_terrain {

FeatureComputer::FeatureComputer(const FeatureComputerParams& params)
  : params_(params) {}

void FeatureComputer::setParams(const FeatureComputerParams& params) {
  params_ = params;
}

void FeatureComputer::compute(ElevationGrid& grid,
                              const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                              const pcl::PointCloud<pcl::Normal>::Ptr& normals) const {
  if (!cloud || cloud->empty()) return;

  // 1. Insert points and normals into the elevation grid
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& pt = (*cloud)[i];
    if (normals && i < normals->size()) {
      const auto& n = (*normals)[i];
      if (std::isfinite(n.normal_x) && std::isfinite(n.normal_y) && std::isfinite(n.normal_z)) {
        grid.insertPointWithNormal(pt.x, pt.y, pt.z,
                                   Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z));
      } else {
        grid.insertPoint(pt.x, pt.y, pt.z);
      }
    } else {
      grid.insertPoint(pt.x, pt.y, pt.z);
    }
  }

  // 2. Compute per-cell features (slope, roughness, curvature)
  computeCellFeatures(grid);

  // 3. Detect stairs and ramps
  detectStairs(grid);
  detectRamps(grid);
}

void FeatureComputer::computeCellFeatures(ElevationGrid& grid) const {
  for (auto& [idx, cell] : grid.cellsMutable()) {
    if (cell.point_count < static_cast<uint32_t>(params_.min_points_per_cell)) {
      continue;
    }

    // Slope: angle between mean normal and vertical (degrees)
    float normal_z = std::abs(cell.mean_normal.z());
    normal_z = std::min(1.0f, std::max(-1.0f, normal_z));  // clamp
    cell.mean_slope = std::acos(normal_z) * 180.0f / M_PI;

    // Roughness: use height variance as proxy
    // (standard deviation of z within the cell)
    cell.roughness = std::sqrt(std::max(0.0f, cell.height_var));

    // Curvature: estimated from neighbor height differences
    auto neighbors = grid.getNeighbors(idx);
    if (!neighbors.empty()) {
      float curvature_sum = 0.0f;
      int valid_neighbors = 0;
      for (const auto& nidx : neighbors) {
        const auto& ncell = grid.getCell(nidx);
        if (ncell.point_count >= static_cast<uint32_t>(params_.min_points_per_cell)) {
          float dz = std::abs(ncell.mean_z - cell.mean_z);
          curvature_sum += dz;
          valid_neighbors++;
        }
      }
      if (valid_neighbors > 0) {
        cell.curvature = curvature_sum / (valid_neighbors * grid.resolution());
      }
    }
  }
}

void FeatureComputer::detectStairs(ElevationGrid& grid) const {
  // For each cell, search along gradient direction for periodic height jumps
  const float step_min = params_.stair_min_step_height;
  const float step_max = params_.stair_max_step_height;
  const int min_steps = params_.stair_min_steps;
  const float search_length = params_.stair_search_length;
  const float res = grid.resolution();
  const int search_cells = static_cast<int>(search_length / res);

  for (auto& [idx, cell] : grid.cellsMutable()) {
    if (cell.point_count < static_cast<uint32_t>(params_.min_points_per_cell)) {
      continue;
    }

    // Compute gradient direction from neighbors
    auto neighbors = grid.getNeighbors(idx);
    if (neighbors.size() < 2) continue;

    Eigen::Vector2f gradient = Eigen::Vector2f::Zero();
    for (const auto& nidx : neighbors) {
      const auto& ncell = grid.getCell(nidx);
      if (ncell.point_count < static_cast<uint32_t>(params_.min_points_per_cell)) continue;
      float dz = ncell.mean_z - cell.mean_z;
      gradient += dz * Eigen::Vector2f(
        static_cast<float>(nidx.x - idx.x),
        static_cast<float>(nidx.y - idx.y));
    }

    if (gradient.norm() < 1e-6f) continue;
    gradient.normalize();

    // Walk along gradient direction, counting step-like height changes
    int step_count = 0;
    float prev_z = cell.mean_z;

    for (int s = 1; s <= search_cells; ++s) {
      CellIndex next{
        idx.x + static_cast<int>(std::round(gradient.x() * s)),
        idx.y + static_cast<int>(std::round(gradient.y() * s))
      };

      if (!grid.hasCell(next)) continue;
      const auto& next_cell = grid.getCell(next);
      if (next_cell.point_count < static_cast<uint32_t>(params_.min_points_per_cell)) continue;

      float dz = std::abs(next_cell.mean_z - prev_z);
      if (dz >= step_min && dz <= step_max) {
        step_count++;
        prev_z = next_cell.mean_z;
      }
    }

    if (step_count >= min_steps) {
      cell.stair_detected = true;
      // Also mark nearby cells along the stair path
    }
  }
}

void FeatureComputer::detectRamps(ElevationGrid& grid) const {
  // Region growing: find connected regions with consistent slope
  const float min_slope = params_.ramp_min_slope_deg;
  const float max_slope = params_.ramp_max_slope_deg;
  const float consistency = params_.ramp_slope_consistency;
  const float min_length = params_.ramp_min_length;
  const float res = grid.resolution();
  const int min_cells = static_cast<int>(min_length / res);

  struct CellIndexCmp {
    bool operator()(const CellIndex& a, const CellIndex& b) const {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    }
  };
  std::set<CellIndex, CellIndexCmp> visited;

  for (auto& [idx, cell] : grid.cellsMutable()) {
    if (cell.point_count < static_cast<uint32_t>(params_.min_points_per_cell)) continue;
    if (cell.mean_slope < min_slope || cell.mean_slope > max_slope) continue;

    // Check if already visited
    if (visited.count(idx)) continue;

    // Region growing from this cell
    std::queue<CellIndex> queue;
    std::vector<CellIndex> region;
    queue.push(idx);
    visited.insert(idx);

    while (!queue.empty()) {
      CellIndex current = queue.front();
      queue.pop();
      region.push_back(current);

      auto neighbors = grid.getNeighbors(current);
      for (const auto& nidx : neighbors) {
        if (visited.count(nidx)) continue;
        const auto& ncell = grid.getCell(nidx);
        if (ncell.point_count < static_cast<uint32_t>(params_.min_points_per_cell)) continue;

        // Check slope is within ramp range and consistent
        if (ncell.mean_slope >= min_slope && ncell.mean_slope <= max_slope) {
          const auto& cur_cell = grid.getCell(current);
          if (std::abs(ncell.mean_slope - cur_cell.mean_slope) < consistency) {
            visited.insert(nidx);
            queue.push(nidx);
          }
        }
      }
    }

    // If region is large enough, mark as ramp
    if (static_cast<int>(region.size()) >= min_cells) {
      for (const auto& ridx : region) {
        grid.getCellMutable(ridx).ramp_detected = true;
      }
    }
  }
}

}  // namespace traversable_terrain
