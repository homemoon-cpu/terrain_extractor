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
NormalEstimator::estimate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();

  if (!input || input->empty()) {
    return normals;
  }

  pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
  ne.setNumberOfThreads(params_.omp_num_threads);
  ne.setInputCloud(input);
  ne.setRadiusSearch(params_.search_radius);
  ne.setViewPoint(params_.viewpoint_x, params_.viewpoint_y, params_.viewpoint_z);

  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
  ne.setSearchMethod(tree);
  ne.compute(*normals);

  return normals;
}

}  // namespace traversable_terrain
