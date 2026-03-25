#include <rclcpp/rclcpp.hpp>
#include "traversable_terrain_extractor/terrain_extractor_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<traversable_terrain::TerrainExtractorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
