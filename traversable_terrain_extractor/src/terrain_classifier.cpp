#include "traversable_terrain_extractor/terrain_classifier.hpp"

#include <cmath>
#include <algorithm>

namespace traversable_terrain {

TerrainClassifier::TerrainClassifier(const TerrainClassifierParams& params)
  : params_(params) {}

void TerrainClassifier::setParams(const TerrainClassifierParams& params) {
  params_ = params;
}

void TerrainClassifier::classify(ElevationGrid& grid) const {
  if (!params_.debug_mode) {
    // Fast path: no stats collection
    for (auto& [idx, cell] : grid.cellsMutable()) {
      cell.classification = classifyCell(cell);
      cell.traversable = isTraversable(cell.classification, cell.mean_slope);
    }
    return;
  }

  // ── Debug path: collect statistics ──────────────────────────────────────
  uint32_t cnt_unknown = 0, cnt_ground = 0, cnt_stairs = 0,
           cnt_ramp = 0, cnt_non_trav = 0;

  // Track why cells fall into NON_TRAVERSABLE
  uint32_t reject_slope = 0, reject_roughness = 0,
           reject_height_var = 0, reject_curvature = 0;

  // Track actual feature ranges for ground-candidate cells
  // (cells that passed point_count check but failed ground thresholds)
  float max_slope_seen = 0.0f, max_rough_seen = 0.0f,
        max_hvar_seen = 0.0f,  max_curv_seen = 0.0f;

  for (auto& [idx, cell] : grid.cellsMutable()) {
    cell.classification = classifyCell(cell);
    cell.traversable = isTraversable(cell.classification, cell.mean_slope);

    switch (cell.classification) {
      case TerrainClass::UNKNOWN:          cnt_unknown++;   break;
      case TerrainClass::GROUND:           cnt_ground++;    break;
      case TerrainClass::STAIRS:           cnt_stairs++;    break;
      case TerrainClass::RAMP:             cnt_ramp++;      break;
      case TerrainClass::NON_TRAVERSABLE:  cnt_non_trav++;  break;
    }

    // For cells with enough points that are NOT ground/stairs/ramp,
    // record which threshold(s) they exceeded
    if (cell.classification == TerrainClass::NON_TRAVERSABLE &&
        cell.point_count >= static_cast<uint32_t>(params_.min_observations)) {

      max_slope_seen = std::max(max_slope_seen, cell.mean_slope);
      max_rough_seen = std::max(max_rough_seen, cell.roughness);
      max_hvar_seen  = std::max(max_hvar_seen,  cell.height_var);
      max_curv_seen  = std::max(max_curv_seen,  cell.curvature);

      if (cell.mean_slope  > params_.ground_max_slope_deg)  reject_slope++;
      if (cell.roughness   > params_.ground_max_roughness)  reject_roughness++;
      if (cell.height_var  > params_.ground_max_height_var) reject_height_var++;
      if (cell.curvature   > params_.ground_max_curvature)  reject_curvature++;
    }
  }

  uint32_t total = cnt_unknown + cnt_ground + cnt_stairs + cnt_ramp + cnt_non_trav;

  RCLCPP_INFO(logger_,
    "[Classifier] Cells: total=%u | GROUND=%u  RAMP=%u  STAIRS=%u"
    "  NON_TRAV=%u  UNKNOWN=%u",
    total, cnt_ground, cnt_ramp, cnt_stairs, cnt_non_trav, cnt_unknown);

  if (cnt_non_trav > 0) {
    RCLCPP_WARN(logger_,
      "[Classifier] NON_TRAVERSABLE rejection breakdown "
      "(cells can fail multiple thresholds):\n"
      "  slope    > %.2f°  : %u cells  (max seen: %.3f°)\n"
      "  roughness> %.4f  : %u cells  (max seen: %.4f)\n"
      "  height_var>%.4f  : %u cells  (max seen: %.4f)\n"
      "  curvature> %.4f  : %u cells  (max seen: %.4f)\n"
      "  Thresholds: slope=%.2f  rough=%.4f  hvar=%.4f  curv=%.4f",
      params_.ground_max_slope_deg,  reject_slope,     max_slope_seen,
      params_.ground_max_roughness,  reject_roughness, max_rough_seen,
      params_.ground_max_height_var, reject_height_var,max_hvar_seen,
      params_.ground_max_curvature,  reject_curvature, max_curv_seen,
      params_.ground_max_slope_deg, params_.ground_max_roughness,
      params_.ground_max_height_var, params_.ground_max_curvature);
  }

  if (cnt_ground == 0 && total > cnt_unknown) {
    RCLCPP_ERROR(logger_,
      "[Classifier] *** ZERO GROUND CELLS *** — all %u valid cells are NON_TRAVERSABLE. "
      "Dominant rejection: slope=%u rough=%u hvar=%u curv=%u. "
      "Loosen the corresponding threshold in terrain_params.yaml",
      cnt_non_trav, reject_slope, reject_roughness, reject_height_var, reject_curvature);
  }
}

TerrainClass TerrainClassifier::classifyCell(const ElevationCell& cell) const {
  // 1. Insufficient data → UNKNOWN
  if (cell.point_count < static_cast<uint32_t>(params_.min_observations)) {
    return TerrainClass::UNKNOWN;
  }

  // 2. Stair pattern detected → STAIRS
  if (cell.stair_detected &&
      cell.mean_slope >= params_.stair_min_slope_deg &&
      cell.mean_slope <= params_.stair_max_slope_deg) {
    return TerrainClass::STAIRS;
  }

  // 3. Ramp region detected → RAMP
  if (cell.ramp_detected &&
      cell.mean_slope >= params_.ramp_min_slope_deg &&
      cell.mean_slope <= params_.ramp_max_slope_deg) {
    return TerrainClass::RAMP;
  }

  // 4. Ground thresholds → GROUND
  if (cell.mean_slope <= params_.ground_max_slope_deg &&
      cell.roughness   <= params_.ground_max_roughness &&
      cell.height_var  <= params_.ground_max_height_var &&
      cell.curvature   <= params_.ground_max_curvature) {
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
