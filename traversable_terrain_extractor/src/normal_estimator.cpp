#include "traversable_terrain_extractor/normal_estimator.hpp"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

namespace traversable_terrain {

NormalEstimator::NormalEstimator(const NormalEstimatorParams& params)
  : params_(params) {}

void NormalEstimator::setParams(const NormalEstimatorParams& params) {
  params_ = params;
}

pcl::PointCloud<pcl::Normal>::Ptr
NormalEstimator::estimate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
                          const Eigen::Vector3f& sensor_pos) const {
  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();

  if (!input || input->empty()) {
    return normals;
  }

  pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
  ne.setNumberOfThreads(params_.omp_num_threads);
  ne.setInputCloud(input);
  ne.setRadiusSearch(params_.search_radius);
  // Use actual sensor position as viewpoint so normals consistently
  // point toward the sensor (away from surface). This is critical for
  // correct slope computation — a fixed viewpoint causes misorientation
  // when the robot moves far from the map origin.
  ne.setViewPoint(sensor_pos.x(), sensor_pos.y(), sensor_pos.z());

  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
  ne.setSearchMethod(tree);
  ne.compute(*normals);

  return normals;
}

}  // namespace traversable_terrain
