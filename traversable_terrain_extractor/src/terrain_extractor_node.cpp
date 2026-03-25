#include "traversable_terrain_extractor/terrain_extractor_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace traversable_terrain {

TerrainExtractorNode::TerrainExtractorNode(const rclcpp::NodeOptions& options)
  : Node("terrain_extractor", options) {

  declareParameters();
  loadParameters();

  // Initialize pipeline components
  preprocessing_ = std::make_unique<Preprocessing>();
  preprocessing_->setLogger(this->get_logger());
  ground_segmentation_ = std::make_unique<GroundSegmentation>();
  normal_estimator_ = std::make_unique<NormalEstimator>();
  feature_computer_ = std::make_unique<FeatureComputer>();
  terrain_classifier_ = std::make_unique<TerrainClassifier>();
  map_accumulator_ = std::make_unique<MapAccumulator>();
  occupancy_projector_ = std::make_unique<OccupancyGridProjector>();

  // Apply parameters to all components
  loadParameters();

  // Subscribers
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("input_cloud_topic").as_string(),
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&TerrainExtractorNode::cloudCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->get_parameter("input_odom_topic").as_string(),
    rclcpp::SensorDataQoS(),
    std::bind(&TerrainExtractorNode::odomCallback, this, std::placeholders::_1));

  // Publishers
  traversable_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/traversable_cloud", 10);
  classified_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/classified_cloud", 10);
  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/ground_cloud", 10);
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "~/occupancy_grid", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/terrain_markers", 10);

  // Service
  save_service_ = this->create_service<traversable_terrain_extractor::srv::SaveTraversableMap>(
    "~/save_map",
    std::bind(&TerrainExtractorNode::saveMapCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Terrain Extractor Node initialized");
}

void TerrainExtractorNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  auto start_time = std::chrono::steady_clock::now();

  // Convert ROS message to PCL cloud
  auto input_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *input_cloud);

  if (input_cloud->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Received empty cloud");
    return;
  }

  // === 1. Preprocessing ===
  // Pass current sensor Z so PassThrough uses relative offsets
  float sensor_z = 0.0f;
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    sensor_z = robot_position_.z();
  }
  auto filtered = preprocessing_->process(input_cloud, sensor_z);
  if (filtered->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Cloud empty after preprocessing");
    return;
  }

  // === 2. Ground Segmentation ===
  auto seg_result = ground_segmentation_->segment(filtered);

  // Publish ground cloud for debugging
  if (ground_pub_->get_subscription_count() > 0 && seg_result.ground && !seg_result.ground->empty()) {
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*seg_result.ground, ground_msg);
    ground_msg.header = msg->header;
    ground_pub_->publish(ground_msg);
  }

  // === 3. Normal Estimation (on all filtered points) ===
  auto normals = normal_estimator_->estimate(filtered);

  // === 4. Feature Computation ===
  // Create per-frame elevation grid
  ElevationGrid frame_grid(feature_computer_ ? 0.3f : 0.3f);
  feature_computer_->compute(frame_grid, filtered, normals);

  // === 5. Terrain Classification ===
  terrain_classifier_->classify(frame_grid);

  // === 6. Build classified cloud ===
  auto classified_cloud = buildClassifiedCloud(filtered, normals, frame_grid);

  // === 7. Map Accumulation (thread-safe) ===
  {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    map_accumulator_->accumulate(classified_cloud);

    // Get robot position and prune
    Eigen::Vector3f pos;
    {
      std::lock_guard<std::mutex> odom_lock(odom_mutex_);
      pos = robot_position_;
    }
    map_accumulator_->prune(pos);

    // Update stored elevation grid for occupancy projection
    elevation_grid_ = frame_grid;
  }

  // === 8. Publish results ===
  auto now = this->now();

  // Classified cloud
  if (classified_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 classified_msg;
    pcl::toROSMsg(*classified_cloud, classified_msg);
    classified_msg.header.stamp = now;
    classified_msg.header.frame_id = map_frame_id_;
    classified_pub_->publish(classified_msg);
  }

  // Traversable cloud (from accumulated map for completeness)
  if (traversable_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<PointXYZITerrain>::Ptr trav_cloud;
    {
      std::shared_lock<std::shared_mutex> lock(map_mutex_);
      trav_cloud = map_accumulator_->getTraversableCloud();
    }
    sensor_msgs::msg::PointCloud2 trav_msg;
    pcl::toROSMsg(*trav_cloud, trav_msg);
    trav_msg.header.stamp = now;
    trav_msg.header.frame_id = map_frame_id_;
    traversable_pub_->publish(trav_msg);
  }

  // Occupancy grid
  if (grid_pub_->get_subscription_count() > 0) {
    Eigen::Vector3f pos;
    {
      std::lock_guard<std::mutex> odom_lock(odom_mutex_);
      pos = robot_position_;
    }
    auto occ_grid = occupancy_projector_->project(frame_grid, pos);
    occ_grid.header.stamp = now;
    grid_pub_->publish(occ_grid);
  }

  // Terrain markers
  if (marker_pub_->get_subscription_count() > 0) {
    auto markers = buildTerrainMarkers(frame_grid);
    marker_pub_->publish(markers);
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time).count();

  RCLCPP_DEBUG(this->get_logger(),
               "Pipeline took %ld ms | Points: %zu → %zu | Cells: %zu | Map: %zu",
               elapsed_ms, input_cloud->size(), filtered->size(),
               frame_grid.size(), map_accumulator_->size());
}

void TerrainExtractorNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  robot_position_.x() = static_cast<float>(msg->pose.pose.position.x);
  robot_position_.y() = static_cast<float>(msg->pose.pose.position.y);
  robot_position_.z() = static_cast<float>(msg->pose.pose.position.z);
}

pcl::PointCloud<PointXYZITerrain>::Ptr
TerrainExtractorNode::buildClassifiedCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
  const pcl::PointCloud<pcl::Normal>::Ptr& normals,
  const ElevationGrid& grid) const {

  auto cloud = std::make_shared<pcl::PointCloud<PointXYZITerrain>>();
  cloud->reserve(input->size());

  for (size_t i = 0; i < input->size(); ++i) {
    const auto& pt = (*input)[i];
    auto cell_idx = grid.worldToCell(pt.x, pt.y);
    const auto& cell = grid.getCell(cell_idx);

    PointXYZITerrain out_pt;
    out_pt.x = pt.x;
    out_pt.y = pt.y;
    out_pt.z = pt.z;
    out_pt.intensity = pt.intensity;
    out_pt.slope = cell.mean_slope;
    out_pt.roughness = cell.roughness;
    out_pt.curvature = cell.curvature;
    out_pt.height_variance = cell.height_var;
    out_pt.terrain_class = static_cast<uint8_t>(cell.classification);
    out_pt.traversable = cell.traversable ? 1 : 0;
    cloud->push_back(out_pt);
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

visualization_msgs::msg::MarkerArray
TerrainExtractorNode::buildTerrainMarkers(const ElevationGrid& grid) const {
  visualization_msgs::msg::MarkerArray markers;

  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear_marker.header.frame_id = map_frame_id_;
  markers.markers.push_back(clear_marker);

  // Color map for terrain classes
  auto getColor = [](TerrainClass tc) -> std::array<float, 4> {
    switch (tc) {
      case TerrainClass::GROUND:          return {0.0f, 0.8f, 0.0f, 0.6f};  // Green
      case TerrainClass::STAIRS:          return {0.0f, 0.5f, 1.0f, 0.8f};  // Blue
      case TerrainClass::RAMP:            return {1.0f, 0.8f, 0.0f, 0.8f};  // Yellow
      case TerrainClass::NON_TRAVERSABLE: return {1.0f, 0.0f, 0.0f, 0.6f};  // Red
      case TerrainClass::UNKNOWN:
      default:                            return {0.5f, 0.5f, 0.5f, 0.3f};  // Gray
    }
  };

  int id = 0;
  for (const auto& [cell_idx, cell] : grid.cells()) {
    if (cell.point_count < 2) continue;

    auto world_pos = grid.cellToWorld(cell_idx);
    auto color = getColor(cell.classification);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.ns = "terrain";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = world_pos.x();
    marker.pose.position.y = world_pos.y();
    marker.pose.position.z = cell.mean_z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = grid.resolution();
    marker.scale.y = grid.resolution();
    marker.scale.z = std::max(0.05f, cell.heightRange());
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    markers.markers.push_back(marker);
  }

  return markers;
}

void TerrainExtractorNode::saveMapCallback(
  const std::shared_ptr<traversable_terrain_extractor::srv::SaveTraversableMap::Request> request,
  std::shared_ptr<traversable_terrain_extractor::srv::SaveTraversableMap::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Save map request received");

  // Determine output directory
  std::string output_dir = request->output_directory.empty()
    ? default_save_dir_ : request->output_directory;

  // Create directory if needed
  try {
    std::filesystem::create_directories(output_dir);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Failed to create directory: ") + e.what();
    return;
  }

  // Generate timestamp for filenames
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
  std::string timestamp = ss.str();

  response->success = true;
  uint32_t total = 0, traversable = 0;

  // Count points
  {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    for (const auto& [key, voxel] : map_accumulator_->voxelMap()) {
      total++;
      if (voxel.traversable) traversable++;
    }
  }

  // Save PCD
  if (request->save_pcd) {
    std::string pcd_path = output_dir + "/terrain_" + timestamp + ".pcd";
    if (savePCD(pcd_path, request->traversable_only)) {
      response->pcd_path = pcd_path;
      RCLCPP_INFO(this->get_logger(), "PCD saved: %s", pcd_path.c_str());
    } else {
      response->success = false;
      response->message += "PCD save failed. ";
    }
  }

  // Save OccupancyGrid (PGM + YAML)
  if (request->save_occupancy_grid) {
    std::string pgm_path = output_dir + "/terrain_" + timestamp + ".pgm";
    std::string yaml_path = output_dir + "/terrain_" + timestamp + ".yaml";

    nav_msgs::msg::OccupancyGrid occ_grid;
    {
      std::shared_lock<std::shared_mutex> lock(map_mutex_);
      occ_grid = occupancy_projector_->project(elevation_grid_);
    }

    if (saveOccupancyGrid(pgm_path, yaml_path, occ_grid)) {
      response->grid_pgm_path = pgm_path;
      response->grid_yaml_path = yaml_path;
      RCLCPP_INFO(this->get_logger(), "Grid saved: %s, %s",
                  pgm_path.c_str(), yaml_path.c_str());
    } else {
      response->success = false;
      response->message += "Grid save failed. ";
    }
  }

  // Save CSV
  if (request->save_csv) {
    std::string csv_path = output_dir + "/terrain_" + timestamp + ".csv";
    if (saveCSV(csv_path, request->traversable_only)) {
      response->csv_path = csv_path;
      RCLCPP_INFO(this->get_logger(), "CSV saved: %s", csv_path.c_str());
    } else {
      response->success = false;
      response->message += "CSV save failed. ";
    }
  }

  response->total_points = total;
  response->traversable_points = traversable;

  if (response->success) {
    response->message = "Map saved successfully";
  }
}

bool TerrainExtractorNode::savePCD(const std::string& path, bool traversable_only) const {
  try {
    pcl::PointCloud<PointXYZITerrain>::Ptr cloud;
    {
      std::shared_lock<std::shared_mutex> lock(map_mutex_);
      cloud = traversable_only
        ? map_accumulator_->getTraversableCloud()
        : map_accumulator_->getAccumulatedCloud();
    }

    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "No points to save");
      return false;
    }

    pcl::io::savePCDFileBinaryCompressed(path, *cloud);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "PCD save error: %s", e.what());
    return false;
  }
}

bool TerrainExtractorNode::saveOccupancyGrid(
  const std::string& pgm_path, const std::string& yaml_path,
  const nav_msgs::msg::OccupancyGrid& grid) const {
  try {
    int width = grid.info.width;
    int height = grid.info.height;

    if (width == 0 || height == 0) {
      RCLCPP_WARN(this->get_logger(), "Empty grid, nothing to save");
      return false;
    }

    // Write PGM (P5 binary format)
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm.is_open()) return false;

    pgm << "P5\n" << width << " " << height << "\n255\n";

    for (int y = height - 1; y >= 0; --y) {  // Flip Y for image
      for (int x = 0; x < width; ++x) {
        int8_t val = grid.data[y * width + x];
        uint8_t pixel;
        if (val < 0) {
          pixel = 205;  // Unknown (gray)
        } else {
          // 0 = white (free), 100 = black (occupied)
          pixel = static_cast<uint8_t>(255 - std::min(100, static_cast<int>(val)) * 255 / 100);
        }
        pgm.write(reinterpret_cast<char*>(&pixel), 1);
      }
    }
    pgm.close();

    // Write YAML (nav2_map_server compatible)
    std::ofstream yaml(yaml_path);
    if (!yaml.is_open()) return false;

    // Extract just the filename for the image path
    std::string pgm_filename = std::filesystem::path(pgm_path).filename().string();

    yaml << "image: " << pgm_filename << "\n";
    yaml << "mode: trinary\n";
    yaml << "resolution: " << grid.info.resolution << "\n";
    yaml << "origin: ["
         << grid.info.origin.position.x << ", "
         << grid.info.origin.position.y << ", "
         << "0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.25\n";
    yaml.close();

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Grid save error: %s", e.what());
    return false;
  }
}

bool TerrainExtractorNode::saveCSV(const std::string& path, bool traversable_only) const {
  try {
    std::ofstream csv(path);
    if (!csv.is_open()) return false;

    csv << "x,y,z,intensity,slope,roughness,curvature,height_variance,terrain_class,traversable\n";

    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    for (const auto& [key, voxel] : map_accumulator_->voxelMap()) {
      if (traversable_only && !voxel.traversable) continue;

      csv << voxel.position.x() << ","
          << voxel.position.y() << ","
          << voxel.position.z() << ","
          << voxel.intensity << ","
          << voxel.slope << ","
          << voxel.roughness << ","
          << voxel.curvature << ","
          << voxel.height_variance << ","
          << static_cast<int>(voxel.terrain_class) << ","
          << static_cast<int>(voxel.traversable) << "\n";
    }

    csv.close();
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "CSV save error: %s", e.what());
    return false;
  }
}

void TerrainExtractorNode::declareParameters() {
  // General
  this->declare_parameter("map_frame_id", "map");
  this->declare_parameter("default_save_directory", "/tmp/traversable_maps");
  this->declare_parameter("input_cloud_topic", "/lio_sam/mapping/cloud_registered");
  this->declare_parameter("input_odom_topic", "/lio_sam/mapping/odometry");

  // Preprocessing
  this->declare_parameter("preprocessing.voxel_size", 0.1);
  this->declare_parameter("preprocessing.min_distance", 0.5);
  this->declare_parameter("preprocessing.max_distance", 50.0);
  this->declare_parameter("preprocessing.z_offset_below", 3.0);  // meters below sensor
  this->declare_parameter("preprocessing.z_offset_above", 1.0);  // meters above sensor
  this->declare_parameter("preprocessing.sor_mean_k", 10);
  this->declare_parameter("preprocessing.sor_std_thresh", 1.0);
  this->declare_parameter("preprocessing.enable_sor", true);
  this->declare_parameter("preprocessing.debug_mode", false);

  // Ground segmentation
  this->declare_parameter("ground_segmentation.method", "patchwork");
  this->declare_parameter("ground_segmentation.ransac_distance_threshold", 0.15);
  this->declare_parameter("ground_segmentation.ransac_max_iterations", 200);
  this->declare_parameter("ground_segmentation.ransac_normal_z_min", 0.85);
  this->declare_parameter("ground_segmentation.pw_num_rings", 4);
  this->declare_parameter("ground_segmentation.pw_num_sectors", 16);
  this->declare_parameter("ground_segmentation.pw_max_range", 50.0);
  this->declare_parameter("ground_segmentation.pw_min_range", 0.5);
  this->declare_parameter("ground_segmentation.pw_uprightness_thresh", 0.7);
  this->declare_parameter("ground_segmentation.pw_elevation_thresh", 0.5);
  this->declare_parameter("ground_segmentation.pw_flatness_thresh", 0.02);
  this->declare_parameter("ground_segmentation.pw_min_points_per_patch", 5);

  // Normal estimation
  this->declare_parameter("normal_estimation.search_radius", 0.3);
  this->declare_parameter("normal_estimation.omp_num_threads", 4);

  // Feature computation
  this->declare_parameter("feature_computation.grid_resolution", 0.3);
  this->declare_parameter("feature_computation.stair_min_step_height", 0.10);
  this->declare_parameter("feature_computation.stair_max_step_height", 0.25);
  this->declare_parameter("feature_computation.stair_min_steps", 2);
  this->declare_parameter("feature_computation.stair_search_length", 3.0);
  this->declare_parameter("feature_computation.ramp_min_length", 1.5);
  this->declare_parameter("feature_computation.ramp_min_slope_deg", 5.0);
  this->declare_parameter("feature_computation.ramp_max_slope_deg", 25.0);
  this->declare_parameter("feature_computation.ramp_slope_consistency", 5.0);
  this->declare_parameter("feature_computation.min_points_per_cell", 2);

  // Terrain classification
  this->declare_parameter("terrain_classification.ground_max_slope_deg", 15.0);
  this->declare_parameter("terrain_classification.ground_max_roughness", 0.05);
  this->declare_parameter("terrain_classification.ground_max_height_var", 0.03);
  this->declare_parameter("terrain_classification.ground_max_curvature", 0.1);
  this->declare_parameter("terrain_classification.stair_min_slope_deg", 20.0);
  this->declare_parameter("terrain_classification.stair_max_slope_deg", 50.0);
  this->declare_parameter("terrain_classification.ramp_min_slope_deg", 5.0);
  this->declare_parameter("terrain_classification.ramp_max_slope_deg", 25.0);
  this->declare_parameter("terrain_classification.max_traversable_slope_deg", 30.0);
  this->declare_parameter("terrain_classification.stairs_traversable", true);
  this->declare_parameter("terrain_classification.ramps_traversable", true);
  this->declare_parameter("terrain_classification.min_observations", 2);

  // Map accumulator
  this->declare_parameter("map_accumulator.voxel_size", 0.15);
  this->declare_parameter("map_accumulator.sliding_window_radius", 80.0);
  this->declare_parameter("map_accumulator.max_points", 5000000);
  this->declare_parameter("map_accumulator.ema_alpha", 0.3);
  this->declare_parameter("map_accumulator.min_observations", 2);

  // Occupancy grid
  this->declare_parameter("occupancy_grid.resolution", 0.2);
  this->declare_parameter("occupancy_grid.range", 100.0);
  this->declare_parameter("occupancy_grid.robot_centric", false);
}

void TerrainExtractorNode::loadParameters() {
  map_frame_id_ = this->get_parameter("map_frame_id").as_string();
  default_save_dir_ = this->get_parameter("default_save_directory").as_string();

  // Preprocessing
  if (preprocessing_) {
    PreprocessingParams pp;
    pp.voxel_size = static_cast<float>(this->get_parameter("preprocessing.voxel_size").as_double());
    pp.min_distance = static_cast<float>(this->get_parameter("preprocessing.min_distance").as_double());
    pp.max_distance = static_cast<float>(this->get_parameter("preprocessing.max_distance").as_double());
    pp.z_offset_below = static_cast<float>(this->get_parameter("preprocessing.z_offset_below").as_double());
    pp.z_offset_above = static_cast<float>(this->get_parameter("preprocessing.z_offset_above").as_double());
    pp.sor_mean_k = this->get_parameter("preprocessing.sor_mean_k").as_int();
    pp.sor_std_thresh = static_cast<float>(this->get_parameter("preprocessing.sor_std_thresh").as_double());
    pp.enable_sor = this->get_parameter("preprocessing.enable_sor").as_bool();
    pp.debug_mode = this->get_parameter("preprocessing.debug_mode").as_bool();
    preprocessing_->setParams(pp);
  }

  // Ground segmentation
  if (ground_segmentation_) {
    GroundSegmentationParams gsp;
    std::string method = this->get_parameter("ground_segmentation.method").as_string();
    gsp.method = (method == "ransac") ? GroundSegMethod::RANSAC : GroundSegMethod::PATCHWORK;
    gsp.ransac_distance_threshold = static_cast<float>(this->get_parameter("ground_segmentation.ransac_distance_threshold").as_double());
    gsp.ransac_max_iterations = this->get_parameter("ground_segmentation.ransac_max_iterations").as_int();
    gsp.ransac_normal_z_min = static_cast<float>(this->get_parameter("ground_segmentation.ransac_normal_z_min").as_double());
    gsp.pw_num_rings = this->get_parameter("ground_segmentation.pw_num_rings").as_int();
    gsp.pw_num_sectors = this->get_parameter("ground_segmentation.pw_num_sectors").as_int();
    gsp.pw_max_range = static_cast<float>(this->get_parameter("ground_segmentation.pw_max_range").as_double());
    gsp.pw_min_range = static_cast<float>(this->get_parameter("ground_segmentation.pw_min_range").as_double());
    gsp.pw_uprightness_thresh = static_cast<float>(this->get_parameter("ground_segmentation.pw_uprightness_thresh").as_double());
    gsp.pw_elevation_thresh = static_cast<float>(this->get_parameter("ground_segmentation.pw_elevation_thresh").as_double());
    gsp.pw_flatness_thresh = static_cast<float>(this->get_parameter("ground_segmentation.pw_flatness_thresh").as_double());
    gsp.pw_min_points_per_patch = this->get_parameter("ground_segmentation.pw_min_points_per_patch").as_int();
    ground_segmentation_->setParams(gsp);
  }

  // Normal estimation
  if (normal_estimator_) {
    NormalEstimatorParams nep;
    nep.search_radius = static_cast<float>(this->get_parameter("normal_estimation.search_radius").as_double());
    nep.omp_num_threads = this->get_parameter("normal_estimation.omp_num_threads").as_int();
    normal_estimator_->setParams(nep);
  }

  // Feature computation
  if (feature_computer_) {
    FeatureComputerParams fcp;
    fcp.grid_resolution = static_cast<float>(this->get_parameter("feature_computation.grid_resolution").as_double());
    fcp.stair_min_step_height = static_cast<float>(this->get_parameter("feature_computation.stair_min_step_height").as_double());
    fcp.stair_max_step_height = static_cast<float>(this->get_parameter("feature_computation.stair_max_step_height").as_double());
    fcp.stair_min_steps = this->get_parameter("feature_computation.stair_min_steps").as_int();
    fcp.stair_search_length = static_cast<float>(this->get_parameter("feature_computation.stair_search_length").as_double());
    fcp.ramp_min_length = static_cast<float>(this->get_parameter("feature_computation.ramp_min_length").as_double());
    fcp.ramp_min_slope_deg = static_cast<float>(this->get_parameter("feature_computation.ramp_min_slope_deg").as_double());
    fcp.ramp_max_slope_deg = static_cast<float>(this->get_parameter("feature_computation.ramp_max_slope_deg").as_double());
    fcp.ramp_slope_consistency = static_cast<float>(this->get_parameter("feature_computation.ramp_slope_consistency").as_double());
    fcp.min_points_per_cell = this->get_parameter("feature_computation.min_points_per_cell").as_int();
    feature_computer_->setParams(fcp);
  }

  // Terrain classification
  if (terrain_classifier_) {
    TerrainClassifierParams tcp;
    tcp.ground_max_slope_deg = static_cast<float>(this->get_parameter("terrain_classification.ground_max_slope_deg").as_double());
    tcp.ground_max_roughness = static_cast<float>(this->get_parameter("terrain_classification.ground_max_roughness").as_double());
    tcp.ground_max_height_var = static_cast<float>(this->get_parameter("terrain_classification.ground_max_height_var").as_double());
    tcp.ground_max_curvature = static_cast<float>(this->get_parameter("terrain_classification.ground_max_curvature").as_double());
    tcp.stair_min_slope_deg = static_cast<float>(this->get_parameter("terrain_classification.stair_min_slope_deg").as_double());
    tcp.stair_max_slope_deg = static_cast<float>(this->get_parameter("terrain_classification.stair_max_slope_deg").as_double());
    tcp.ramp_min_slope_deg = static_cast<float>(this->get_parameter("terrain_classification.ramp_min_slope_deg").as_double());
    tcp.ramp_max_slope_deg = static_cast<float>(this->get_parameter("terrain_classification.ramp_max_slope_deg").as_double());
    tcp.max_traversable_slope_deg = static_cast<float>(this->get_parameter("terrain_classification.max_traversable_slope_deg").as_double());
    tcp.stairs_traversable = this->get_parameter("terrain_classification.stairs_traversable").as_bool();
    tcp.ramps_traversable = this->get_parameter("terrain_classification.ramps_traversable").as_bool();
    tcp.min_observations = this->get_parameter("terrain_classification.min_observations").as_int();
    terrain_classifier_->setParams(tcp);
  }

  // Map accumulator
  if (map_accumulator_) {
    MapAccumulatorParams map;
    map.voxel_size = static_cast<float>(this->get_parameter("map_accumulator.voxel_size").as_double());
    map.sliding_window_radius = static_cast<float>(this->get_parameter("map_accumulator.sliding_window_radius").as_double());
    map.max_points = this->get_parameter("map_accumulator.max_points").as_int();
    map.ema_alpha = static_cast<float>(this->get_parameter("map_accumulator.ema_alpha").as_double());
    map.min_observations = this->get_parameter("map_accumulator.min_observations").as_int();
    map_accumulator_->setParams(map);
  }

  // Occupancy grid
  if (occupancy_projector_) {
    OccupancyGridProjectorParams ogp;
    ogp.resolution = static_cast<float>(this->get_parameter("occupancy_grid.resolution").as_double());
    ogp.range = static_cast<float>(this->get_parameter("occupancy_grid.range").as_double());
    ogp.robot_centric = this->get_parameter("occupancy_grid.robot_centric").as_bool();
    ogp.frame_id = map_frame_id_;
    occupancy_projector_->setParams(ogp);
  }
}

}  // namespace traversable_terrain
