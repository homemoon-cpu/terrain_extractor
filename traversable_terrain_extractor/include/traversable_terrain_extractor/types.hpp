#pragma once

#include <cstdint>
#include <cmath>
#include <Eigen/Dense>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>

namespace traversable_terrain {

// ========== Terrain classification enum ==========
enum class TerrainClass : uint8_t {
  UNKNOWN = 0,
  GROUND = 1,
  STAIRS = 2,
  RAMP = 3,
  NON_TRAVERSABLE = 4
};

inline const char* terrainClassToString(TerrainClass tc) {
  switch (tc) {
    case TerrainClass::UNKNOWN:          return "UNKNOWN";
    case TerrainClass::GROUND:           return "GROUND";
    case TerrainClass::STAIRS:           return "STAIRS";
    case TerrainClass::RAMP:             return "RAMP";
    case TerrainClass::NON_TRAVERSABLE:  return "NON_TRAVERSABLE";
    default:                             return "INVALID";
  }
}

// ========== Custom point type extending XYZI ==========
struct EIGEN_ALIGN16 PointXYZITerrain {
  PCL_ADD_POINT4D;
  float intensity;
  float slope;
  float roughness;
  float curvature;
  float height_variance;
  uint8_t terrain_class;
  uint8_t traversable;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

// ========== Elevation grid cell ==========
struct ElevationCell {
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  float mean_z = 0.0f;
  float height_var = 0.0f;
  float mean_slope = 0.0f;
  float roughness = 0.0f;
  float curvature = 0.0f;
  uint32_t point_count = 0;
  Eigen::Vector3f mean_normal = Eigen::Vector3f::UnitZ();
  TerrainClass classification = TerrainClass::UNKNOWN;
  bool traversable = false;

  // For incremental mean computation
  float sum_z = 0.0f;
  float sum_z_sq = 0.0f;

  // Normal accumulation
  Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();

  // Stair/ramp detection flags
  bool stair_detected = false;
  bool ramp_detected = false;

  void addPoint(float z) {
    point_count++;
    sum_z += z;
    sum_z_sq += z * z;
    min_z = std::min(min_z, z);
    max_z = std::max(max_z, z);
    mean_z = sum_z / static_cast<float>(point_count);
    if (point_count > 1) {
      height_var = (sum_z_sq / point_count) - (mean_z * mean_z);
      height_var = std::max(0.0f, height_var);  // Numerical safety
    }
  }

  void addNormal(const Eigen::Vector3f& n) {
    normal_sum += n;
    mean_normal = normal_sum.normalized();
  }

  float heightRange() const {
    return (point_count > 0) ? (max_z - min_z) : 0.0f;
  }

  void reset() {
    *this = ElevationCell{};
  }
};

}  // namespace traversable_terrain

// ========== PCL point type registration ==========
POINT_CLOUD_REGISTER_POINT_STRUCT(
  traversable_terrain::PointXYZITerrain,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, slope, slope)
  (float, roughness, roughness)
  (float, curvature, curvature)
  (float, height_variance, height_variance)
  (uint8_t, terrain_class, terrain_class)
  (uint8_t, traversable, traversable)
)
