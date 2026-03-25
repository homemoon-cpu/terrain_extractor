#pragma once

#include "traversable_terrain_extractor/types.hpp"
#include <unordered_map>
#include <vector>
#include <cmath>
#include <functional>
#include <shared_mutex>
#include <Eigen/Dense>

namespace traversable_terrain {

// Hash for grid cell indices
struct CellIndex {
  int x;
  int y;

  bool operator==(const CellIndex& other) const {
    return x == other.x && y == other.y;
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    // Cantor pairing function
    auto h1 = std::hash<int>{}(idx.x);
    auto h2 = std::hash<int>{}(idx.y);
    return h1 ^ (h2 << 16) ^ (h2 >> 16);
  }
};

class ElevationGrid {
public:
  explicit ElevationGrid(float resolution = 0.3f);

  // Reset all cells
  void clear();

  // Get cell index for a world coordinate
  CellIndex worldToCell(float x, float y) const;

  // Get world coordinate center for a cell index
  Eigen::Vector2f cellToWorld(const CellIndex& idx) const;

  // Insert a point into the grid
  void insertPoint(float x, float y, float z);

  // Insert a point with normal
  void insertPointWithNormal(float x, float y, float z,
                             const Eigen::Vector3f& normal);

  // Get cell (const reference, returns empty cell if not found)
  const ElevationCell& getCell(const CellIndex& idx) const;

  // Get mutable cell reference (creates if not exists)
  ElevationCell& getCellMutable(const CellIndex& idx);

  // Check if cell exists
  bool hasCell(const CellIndex& idx) const;

  // Get all occupied cells
  const std::unordered_map<CellIndex, ElevationCell, CellIndexHash>& cells() const;

  // Get mutable cells map
  std::unordered_map<CellIndex, ElevationCell, CellIndexHash>& cellsMutable();

  // Get neighbor cell indices (8-connectivity)
  std::vector<CellIndex> getNeighbors(const CellIndex& idx) const;

  // Number of occupied cells
  size_t size() const;

  float resolution() const { return resolution_; }

private:
  float resolution_;
  float inv_resolution_;
  std::unordered_map<CellIndex, ElevationCell, CellIndexHash> cells_;
  static const ElevationCell empty_cell_;
};

}  // namespace traversable_terrain
