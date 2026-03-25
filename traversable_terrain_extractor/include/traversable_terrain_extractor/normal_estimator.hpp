#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <Eigen/Dense>

namespace traversable_terrain {

struct NormalEstimatorParams {
  float search_radius = 0.3f;
  int omp_num_threads = 4;
  // Viewpoint for consistent normal orientation (0,0,+inf -> normals point up)
  float viewpoint_x = 0.0f;
  float viewpoint_y = 0.0f;
  float viewpoint_z = 1000.0f;
};

class NormalEstimator {
public:
  explicit NormalEstimator(const NormalEstimatorParams& params = NormalEstimatorParams{});

  void setParams(const NormalEstimatorParams& params);

  // Estimate normals for the input cloud
  // sensor_pos: actual sensor position in map frame — used as viewpoint so
  //             normals are flipped to consistently point away from sensor.
  //             Pass robot_position from odometry each frame.
  pcl::PointCloud<pcl::Normal>::Ptr
  estimate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
           const Eigen::Vector3f& sensor_pos = Eigen::Vector3f(0.f, 0.f, 1000.f)) const;

private:
  NormalEstimatorParams params_;
};

}  // namespace traversable_terrain
