#pragma once

#include "traversable_terrain_extractor/types.hpp"
#include <unordered_map>
#include <Eigen/Dense>

namespace traversable_terrain {

struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const {
    auto h1 = std::hash<int>{}(k.x);
    auto h2 = std::hash<int>{}(k.y);
    auto h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 11) ^ (h3 << 22);
  }
};

struct AccumulatedVoxel {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  float intensity = 0.0f;
  float slope = 0.0f;
  float roughness = 0.0f;
  float curvature = 0.0f;
  float height_variance = 0.0f;
  uint8_t terrain_class = 0;
  uint8_t traversable = 0;
  uint32_t observation_count = 0;
};

struct MapAccumulatorParams {
  float voxel_size = 0.15f;
  float sliding_window_radius = 80.0f;
  size_t max_points = 5000000;
  float ema_alpha = 0.3f;  // EMA smoothing coefficient
  int min_observations = 2;  // Min observations for final classification
};

class MapAccumulator {
public:
  explicit MapAccumulator(const MapAccumulatorParams& params = MapAccumulatorParams{});

  void setParams(const MapAccumulatorParams& params);

  // Accumulate a classified point cloud into the map
  void accumulate(const pcl::PointCloud<PointXYZITerrain>::Ptr& cloud);

  // Prune points outside the sliding window
  void prune(const Eigen::Vector3f& center);

  // Get the full accumulated cloud
  pcl::PointCloud<PointXYZITerrain>::Ptr getAccumulatedCloud() const;

  // Get only traversable points
  pcl::PointCloud<PointXYZITerrain>::Ptr getTraversableCloud() const;

  // Get total point count
  size_t size() const { return voxel_map_.size(); }

  // Clear
  void clear();

  // Const access to voxel map for save service
  const std::unordered_map<VoxelKey, AccumulatedVoxel, VoxelKeyHash>& voxelMap() const {
    return voxel_map_;
  }

private:
  VoxelKey worldToVoxel(const Eigen::Vector3f& point) const;

  MapAccumulatorParams params_;
  std::unordered_map<VoxelKey, AccumulatedVoxel, VoxelKeyHash> voxel_map_;
};

}  // namespace traversable_terrain
