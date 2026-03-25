#include "traversable_terrain_extractor/ground_segmentation.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

namespace traversable_terrain {

GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params)
  : params_(params) {}

void GroundSegmentation::setParams(const GroundSegmentationParams& params) {
  params_ = params;
}

GroundSegmentationResult
GroundSegmentation::segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  if (!input || input->empty()) {
    GroundSegmentationResult result;
    result.ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    result.non_ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    result.ground_plane = std::make_shared<pcl::ModelCoefficients>();
    return result;
  }

  switch (params_.method) {
    case GroundSegMethod::RANSAC:
      return segmentRANSAC(input);
    case GroundSegMethod::PATCHWORK:
    default:
      return segmentPatchwork(input);
  }
}

GroundSegmentationResult
GroundSegmentation::segmentRANSAC(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  GroundSegmentationResult result;
  result.ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  result.non_ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  result.ground_plane = std::make_shared<pcl::ModelCoefficients>();

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(params_.ransac_distance_threshold);
  seg.setMaxIterations(params_.ransac_max_iterations);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle(std::acos(params_.ransac_normal_z_min));

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.setInputCloud(input);
  seg.segment(*inliers, *result.ground_plane);

  if (inliers->indices.empty()) {
    *result.non_ground = *input;
    return result;
  }

  // Extract ground
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*result.ground);

  // Extract non-ground
  extract.setNegative(true);
  extract.filter(*result.non_ground);

  return result;
}

GroundSegmentationResult
GroundSegmentation::segmentPatchwork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  GroundSegmentationResult result;
  result.ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  result.non_ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  result.ground_plane = std::make_shared<pcl::ModelCoefficients>();

  const int num_rings = params_.pw_num_rings;
  const int num_sectors = params_.pw_num_sectors;
  const float max_range = params_.pw_max_range;
  const float min_range = params_.pw_min_range;
  const int total_patches = num_rings * num_sectors;

  // Compute ring boundaries (equal area rings)
  std::vector<float> ring_bounds(num_rings + 1);
  ring_bounds[0] = min_range;
  float area_per_ring = (max_range * max_range - min_range * min_range) / num_rings;
  for (int r = 1; r <= num_rings; ++r) {
    ring_bounds[r] = std::sqrt(ring_bounds[r - 1] * ring_bounds[r - 1] + area_per_ring);
  }

  // Bin points into patches
  struct Patch {
    std::vector<int> indices;
  };
  std::vector<Patch> patches(total_patches);

  for (size_t i = 0; i < input->size(); ++i) {
    const auto& pt = (*input)[i];
    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    if (dist < min_range || dist >= max_range) {
      result.non_ground->push_back(pt);
      continue;
    }

    // Find ring
    int ring = -1;
    for (int r = 0; r < num_rings; ++r) {
      if (dist >= ring_bounds[r] && dist < ring_bounds[r + 1]) {
        ring = r;
        break;
      }
    }
    if (ring < 0) {
      result.non_ground->push_back(pt);
      continue;
    }

    // Find sector
    float angle = std::atan2(pt.y, pt.x);  // [-pi, pi]
    if (angle < 0) angle += 2.0f * M_PI;
    int sector = static_cast<int>(angle / (2.0f * M_PI) * num_sectors);
    sector = std::min(sector, num_sectors - 1);

    patches[ring * num_sectors + sector].indices.push_back(static_cast<int>(i));
  }

  // Process each patch: fit plane and check uprightness
  for (int p = 0; p < total_patches; ++p) {
    const auto& patch = patches[p];
    if (static_cast<int>(patch.indices.size()) < params_.pw_min_points_per_patch) {
      for (int idx : patch.indices) {
        result.non_ground->push_back((*input)[idx]);
      }
      continue;
    }

    // Compute centroid and covariance for PCA-based plane fit
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int idx : patch.indices) {
      const auto& pt = (*input)[idx];
      centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
    }
    centroid /= static_cast<float>(patch.indices.size());

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (int idx : patch.indices) {
      const auto& pt = (*input)[idx];
      Eigen::Vector3f diff(pt.x - centroid.x(), pt.y - centroid.y(), pt.z - centroid.z());
      covariance += diff * diff.transpose();
    }
    covariance /= static_cast<float>(patch.indices.size());

    // Eigen decomposition - smallest eigenvalue's eigenvector is the normal
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f normal = solver.eigenvectors().col(0);  // Smallest eigenvalue
    float smallest_eigenvalue = solver.eigenvalues()(0);

    // Ensure normal points upward
    if (normal.z() < 0) normal = -normal;

    // Check uprightness: how vertical is the normal?
    float uprightness = normal.z();  // dot with (0,0,1)

    // Check flatness: ratio of smallest to middle eigenvalue
    float middle_eigenvalue = solver.eigenvalues()(1);
    float flatness = (middle_eigenvalue > 1e-6f)
                       ? smallest_eigenvalue / middle_eigenvalue
                       : 0.0f;

    bool is_ground_patch = (uprightness >= params_.pw_uprightness_thresh) &&
                           (flatness < params_.pw_flatness_thresh);

    if (is_ground_patch) {
      // Additional check: points should be close to fitted plane
      float plane_d = -normal.dot(centroid);
      for (int idx : patch.indices) {
        const auto& pt = (*input)[idx];
        float dist_to_plane = std::abs(
          normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + plane_d);
        if (dist_to_plane < params_.pw_elevation_thresh) {
          result.ground->push_back((*input)[idx]);
        } else {
          result.non_ground->push_back((*input)[idx]);
        }
      }
    } else {
      for (int idx : patch.indices) {
        result.non_ground->push_back((*input)[idx]);
      }
    }
  }

  // Set cloud metadata
  result.ground->width = result.ground->size();
  result.ground->height = 1;
  result.ground->is_dense = true;
  result.non_ground->width = result.non_ground->size();
  result.non_ground->height = 1;
  result.non_ground->is_dense = true;

  return result;
}

}  // namespace traversable_terrain
