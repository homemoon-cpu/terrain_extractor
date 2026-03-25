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

  // 4. Debug statistics
  if (params_.debug_mode) {
    logFeatureStats(grid);
  }
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

void FeatureComputer::logFeatureStats(const ElevationGrid& grid) const {
  // Collect per-cell feature values for valid cells
  struct Stats {
    float min_val =  std::numeric_limits<float>::max();
    float max_val =  std::numeric_limits<float>::lowest();
    double sum = 0.0;
    uint32_t count = 0;

    void add(float v) {
      min_val = std::min(min_val, v);
      max_val = std::max(max_val, v);
      sum += v;
      count++;
    }
    float mean() const { return count > 0 ? static_cast<float>(sum / count) : 0.f; }
  };

  Stats normal_z_stats, slope_stats, roughness_stats, height_var_stats, curvature_stats;

  // Buckets for normal_z histogram: [-1,-0.5), [-0.5,0), [0,0.5), [0.5,0.8), [0.8,1.0]
  uint32_t nz_neg = 0, nz_low = 0, nz_mid = 0, nz_high = 0, nz_very_high = 0;

  uint32_t valid_cells = 0, invalid_cells = 0;

  for (const auto& [idx, cell] : grid.cells()) {
    if (cell.point_count < static_cast<uint32_t>(params_.min_points_per_cell)) {
      invalid_cells++;
      continue;
    }
    valid_cells++;

    float nz = cell.mean_normal.z();
    normal_z_stats.add(nz);
    slope_stats.add(cell.mean_slope);
    roughness_stats.add(cell.roughness);
    height_var_stats.add(cell.height_var);
    curvature_stats.add(cell.curvature);

    if      (nz < -0.5f) nz_neg++;
    else if (nz <  0.0f) nz_low++;
    else if (nz <  0.5f) nz_mid++;
    else if (nz <  0.8f) nz_high++;
    else                 nz_very_high++;
  }

  RCLCPP_INFO(logger_,
    "[FeatureComputer] Cells: valid=%u  skipped(low pts)=%u",
    valid_cells, invalid_cells);

  RCLCPP_INFO(logger_,
    "[FeatureComputer] normal_z  : min=%.3f  max=%.3f  mean=%.3f\n"
    "  histogram: nz<-0.5:%u  [-0.5,0):%u  [0,0.5):%u  [0.5,0.8):%u  >=0.8:%u\n"
    "  NOTE: ground normals should be in [0.5,1.0]. "
    "If most are low → viewpoint or search_radius issue.",
    normal_z_stats.min_val, normal_z_stats.max_val, normal_z_stats.mean(),
    nz_neg, nz_low, nz_mid, nz_high, nz_very_high);

  RCLCPP_INFO(logger_,
    "[FeatureComputer] slope(°)   : min=%.2f  max=%.2f  mean=%.2f  "
    "(ground threshold=%.2f°)",
    slope_stats.min_val, slope_stats.max_val, slope_stats.mean(),
    params_.ramp_max_slope_deg);  // reference only

  RCLCPP_INFO(logger_,
    "[FeatureComputer] roughness  : min=%.4f  max=%.4f  mean=%.4f\n"
    "             height_var: min=%.4f  max=%.4f  mean=%.4f\n"
    "             curvature : min=%.4f  max=%.4f  mean=%.4f",
    roughness_stats.min_val,   roughness_stats.max_val,   roughness_stats.mean(),
    height_var_stats.min_val,  height_var_stats.max_val,  height_var_stats.mean(),
    curvature_stats.min_val,   curvature_stats.max_val,   curvature_stats.mean());

  // Warn if normal_z distribution is suspicious
  if (valid_cells > 0) {
    float frac_bad = static_cast<float>(nz_neg + nz_low + nz_mid) / valid_cells;
    if (frac_bad > 0.5f) {
      RCLCPP_WARN(logger_,
        "[FeatureComputer] %.0f%% of cells have normal_z < 0.5 — "
        "normals are not pointing upward! Check: "
        "(1) viewpoint should be sensor position not map origin, "
        "(2) increase normal_estimation.search_radius, "
        "(3) ensure ground points are not mixed with walls in same cell.",
        frac_bad * 100.f);
    }
  }
}

}  // namespace traversable_terrain
