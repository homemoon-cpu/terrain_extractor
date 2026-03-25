#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

namespace traversable_terrain {

enum class GroundSegMethod {
  RANSAC,
  PATCHWORK
};

struct GroundSegmentationParams {
  GroundSegMethod method = GroundSegMethod::PATCHWORK;

  // RANSAC params
  float ransac_distance_threshold = 0.15f;
  int ransac_max_iterations = 200;
  float ransac_normal_z_min = 0.85f;  // cos(~32°)

  // Patchwork params
  int pw_num_rings = 4;
  int pw_num_sectors = 16;
  float pw_max_range = 50.0f;
  float pw_min_range = 0.5f;
  float pw_uprightness_thresh = 0.7f;  // cos(~45°) for normal z-component
  float pw_elevation_thresh = 0.5f;    // max height diff from fitted plane
  float pw_flatness_thresh = 0.02f;    // planarity threshold (eigenvalue ratio)
  int pw_min_points_per_patch = 5;
};

struct GroundSegmentationResult {
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground;
  pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground;
  pcl::ModelCoefficients::Ptr ground_plane;  // For RANSAC mode
};

class GroundSegmentation {
public:
  explicit GroundSegmentation(const GroundSegmentationParams& params = GroundSegmentationParams{});

  void setParams(const GroundSegmentationParams& params);

  GroundSegmentationResult
  segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const;

private:
  GroundSegmentationResult
  segmentRANSAC(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const;

  GroundSegmentationResult
  segmentPatchwork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const;

  GroundSegmentationParams params_;
};

}  // namespace traversable_terrain
