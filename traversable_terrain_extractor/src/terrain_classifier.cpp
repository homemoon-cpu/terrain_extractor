#include "traversable_terrain_extractor/terrain_classifier.hpp"

#include <cmath>

namespace traversable_terrain {

TerrainClassifier::TerrainClassifier(const TerrainClassifierParams& params)
  : params_(params) {}

void TerrainClassifier::setParams(const TerrainClassifierParams& params) {
  params_ = params;
}

void TerrainClassifier::classify(ElevationGrid& grid) const {
  for (auto& [idx, cell] : grid.cellsMutable()) {
    cell.classification = classifyCell(cell);
    cell.traversable = isTraversable(cell.classification, cell.mean_slope);
  }
}

TerrainClass TerrainClassifier::classifyCell(const ElevationCell& cell) const {
  // Priority-based classification

  // 1. Insufficient data → UNKNOWN
  if (cell.point_count < static_cast<uint32_t>(params_.min_observations)) {
    return TerrainClass::UNKNOWN;
  }

  // 2. Stair pattern detected → STAIRS
  if (cell.stair_detected) {
    if (cell.mean_slope >= params_.stair_min_slope_deg &&
        cell.mean_slope <= params_.stair_max_slope_deg) {
      return TerrainClass::STAIRS;
    }
  }

  // 3. Ramp region detected → RAMP
  if (cell.ramp_detected) {
    if (cell.mean_slope >= params_.ramp_min_slope_deg &&
        cell.mean_slope <= params_.ramp_max_slope_deg) {
      return TerrainClass::RAMP;
    }
  }

  // 4. Ground thresholds met → GROUND
  if (cell.mean_slope <= params_.ground_max_slope_deg &&
      cell.roughness <= params_.ground_max_roughness &&
      cell.height_var <= params_.ground_max_height_var &&
      cell.curvature <= params_.ground_max_curvature) {
    return TerrainClass::GROUND;
  }

  // 5. Otherwise → NON_TRAVERSABLE
  return TerrainClass::NON_TRAVERSABLE;
}

bool TerrainClassifier::isTraversable(TerrainClass tc, float slope_deg) const {
  switch (tc) {
    case TerrainClass::GROUND:
      return slope_deg <= params_.max_traversable_slope_deg;
    case TerrainClass::STAIRS:
      return params_.stairs_traversable;
    case TerrainClass::RAMP:
      return params_.ramps_traversable &&
             slope_deg <= params_.max_traversable_slope_deg;
    case TerrainClass::UNKNOWN:
    case TerrainClass::NON_TRAVERSABLE:
    default:
      return false;
  }
}

}  // namespace traversable_terrain
