#pragma once

#include <memory>
#include <shared_mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "traversable_terrain_extractor/types.hpp"
#include "traversable_terrain_extractor/preprocessing.hpp"
#include "traversable_terrain_extractor/ground_segmentation.hpp"
#include "traversable_terrain_extractor/normal_estimator.hpp"
#include "traversable_terrain_extractor/feature_computer.hpp"
#include "traversable_terrain_extractor/terrain_classifier.hpp"
#include "traversable_terrain_extractor/elevation_grid.hpp"
#include "traversable_terrain_extractor/map_accumulator.hpp"
#include "traversable_terrain_extractor/occupancy_grid_projector.hpp"
#include "traversable_terrain_extractor/srv/save_traversable_map.hpp"

namespace traversable_terrain {

class TerrainExtractorNode : public rclcpp::Node {
public:
  explicit TerrainExtractorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Callbacks
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Service
  void saveMapCallback(
    const std::shared_ptr<traversable_terrain_extractor::srv::SaveTraversableMap::Request> request,
    std::shared_ptr<traversable_terrain_extractor::srv::SaveTraversableMap::Response> response);

  // Helper: build classified PointXYZITerrain cloud from grid
  pcl::PointCloud<PointXYZITerrain>::Ptr buildClassifiedCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    const ElevationGrid& grid) const;

  // Helper: build visualization markers
  visualization_msgs::msg::MarkerArray buildTerrainMarkers(
    const ElevationGrid& grid) const;

  // Helper: save PCD file
  bool savePCD(const std::string& path, bool traversable_only) const;

  // Helper: save occupancy grid as PGM+YAML
  bool saveOccupancyGrid(const std::string& pgm_path, const std::string& yaml_path,
                         const nav_msgs::msg::OccupancyGrid& grid) const;

  // Helper: save CSV
  bool saveCSV(const std::string& path, bool traversable_only) const;

  // Declare and load parameters
  void declareParameters();
  void loadParameters();

  // Pipeline components
  std::unique_ptr<Preprocessing> preprocessing_;
  std::unique_ptr<GroundSegmentation> ground_segmentation_;
  std::unique_ptr<NormalEstimator> normal_estimator_;
  std::unique_ptr<FeatureComputer> feature_computer_;
  std::unique_ptr<TerrainClassifier> terrain_classifier_;
  std::unique_ptr<MapAccumulator> map_accumulator_;
  std::unique_ptr<OccupancyGridProjector> occupancy_projector_;

  // Elevation grid (per-frame, rebuilt each callback)
  ElevationGrid elevation_grid_;

  // Thread safety
  mutable std::shared_mutex map_mutex_;

  // Robot state
  Eigen::Vector3f robot_position_ = Eigen::Vector3f::Zero();
  bool odom_received_ = false;  // Guard: don't process until first odom arrives
  std::mutex odom_mutex_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr classified_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr normal_debug_pub_;  // normal_z colored XYZRGB
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Service<traversable_terrain_extractor::srv::SaveTraversableMap>::SharedPtr save_service_;

  // Config
  std::string map_frame_id_ = "map";
  std::string default_save_dir_ = "/tmp/traversable_maps";
};

}  // namespace traversable_terrain
