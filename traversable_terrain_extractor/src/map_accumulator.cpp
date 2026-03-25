#include "traversable_terrain_extractor/map_accumulator.hpp"

#include <cmath>

namespace traversable_terrain {

MapAccumulator::MapAccumulator(const MapAccumulatorParams& params)
  : params_(params) {}

void MapAccumulator::setParams(const MapAccumulatorParams& params) {
  params_ = params;
}

void MapAccumulator::accumulate(const pcl::PointCloud<PointXYZITerrain>::Ptr& cloud) {
  if (!cloud || cloud->empty()) return;

  for (const auto& pt : *cloud) {
    Eigen::Vector3f pos(pt.x, pt.y, pt.z);
    VoxelKey key = worldToVoxel(pos);

    auto it = voxel_map_.find(key);
    if (it == voxel_map_.end()) {
      // New voxel
      AccumulatedVoxel voxel;
      voxel.position = pos;
      voxel.intensity = pt.intensity;
      voxel.slope = pt.slope;
      voxel.roughness = pt.roughness;
      voxel.curvature = pt.curvature;
      voxel.height_variance = pt.height_variance;
      voxel.terrain_class = pt.terrain_class;
      voxel.traversable = pt.traversable;
      voxel.observation_count = 1;
      voxel_map_[key] = voxel;
    } else {
      // EMA update existing voxel
      auto& v = it->second;
      float alpha = params_.ema_alpha;
      float beta = 1.0f - alpha;

      v.position = beta * v.position + alpha * pos;
      v.intensity = beta * v.intensity + alpha * pt.intensity;
      v.slope = beta * v.slope + alpha * pt.slope;
      v.roughness = beta * v.roughness + alpha * pt.roughness;
      v.curvature = beta * v.curvature + alpha * pt.curvature;
      v.height_variance = beta * v.height_variance + alpha * pt.height_variance;

      // For terrain class, use majority vote / latest if observation count is high enough
      v.observation_count++;
      if (v.observation_count >= static_cast<uint32_t>(params_.min_observations)) {
        v.terrain_class = pt.terrain_class;
        v.traversable = pt.traversable;
      }
    }
  }
}

void MapAccumulator::prune(const Eigen::Vector3f& center) {
  float r2 = params_.sliding_window_radius * params_.sliding_window_radius;

  for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
    float dx = it->second.position.x() - center.x();
    float dy = it->second.position.y() - center.y();
    float dist2 = dx * dx + dy * dy;
    if (dist2 > r2) {
      it = voxel_map_.erase(it);
    } else {
      ++it;
    }
  }

  // Also enforce max points limit
  if (voxel_map_.size() > params_.max_points) {
    // Remove voxels with lowest observation count
    // Simple approach: just remove excess from the iterator
    size_t excess = voxel_map_.size() - params_.max_points;
    auto it = voxel_map_.begin();
    while (excess > 0 && it != voxel_map_.end()) {
      if (it->second.observation_count < static_cast<uint32_t>(params_.min_observations)) {
        it = voxel_map_.erase(it);
        excess--;
      } else {
        ++it;
      }
    }
  }
}

pcl::PointCloud<PointXYZITerrain>::Ptr MapAccumulator::getAccumulatedCloud() const {
  auto cloud = std::make_shared<pcl::PointCloud<PointXYZITerrain>>();
  cloud->reserve(voxel_map_.size());

  for (const auto& [key, voxel] : voxel_map_) {
    PointXYZITerrain pt;
    pt.x = voxel.position.x();
    pt.y = voxel.position.y();
    pt.z = voxel.position.z();
    pt.intensity = voxel.intensity;
    pt.slope = voxel.slope;
    pt.roughness = voxel.roughness;
    pt.curvature = voxel.curvature;
    pt.height_variance = voxel.height_variance;
    pt.terrain_class = voxel.terrain_class;
    pt.traversable = voxel.traversable;
    cloud->push_back(pt);
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

pcl::PointCloud<PointXYZITerrain>::Ptr MapAccumulator::getTraversableCloud() const {
  auto cloud = std::make_shared<pcl::PointCloud<PointXYZITerrain>>();

  for (const auto& [key, voxel] : voxel_map_) {
    if (voxel.traversable) {
      PointXYZITerrain pt;
      pt.x = voxel.position.x();
      pt.y = voxel.position.y();
      pt.z = voxel.position.z();
      pt.intensity = voxel.intensity;
      pt.slope = voxel.slope;
      pt.roughness = voxel.roughness;
      pt.curvature = voxel.curvature;
      pt.height_variance = voxel.height_variance;
      pt.terrain_class = voxel.terrain_class;
      pt.traversable = voxel.traversable;
      cloud->push_back(pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void MapAccumulator::clear() {
  voxel_map_.clear();
}

VoxelKey MapAccumulator::worldToVoxel(const Eigen::Vector3f& point) const {
  float inv = 1.0f / params_.voxel_size;
  return VoxelKey{
    static_cast<int>(std::floor(point.x() * inv)),
    static_cast<int>(std::floor(point.y() * inv)),
    static_cast<int>(std::floor(point.z() * inv))
  };
}

}  // namespace traversable_terrain
