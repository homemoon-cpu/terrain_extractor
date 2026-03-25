#include "traversable_terrain_extractor/elevation_grid.hpp"

namespace traversable_terrain {

const ElevationCell ElevationGrid::empty_cell_ = ElevationCell{};

ElevationGrid::ElevationGrid(float resolution)
  : resolution_(resolution),
    inv_resolution_(1.0f / resolution) {}

void ElevationGrid::clear() {
  cells_.clear();
}

CellIndex ElevationGrid::worldToCell(float x, float y) const {
  return CellIndex{
    static_cast<int>(std::floor(x * inv_resolution_)),
    static_cast<int>(std::floor(y * inv_resolution_))
  };
}

Eigen::Vector2f ElevationGrid::cellToWorld(const CellIndex& idx) const {
  return Eigen::Vector2f(
    (static_cast<float>(idx.x) + 0.5f) * resolution_,
    (static_cast<float>(idx.y) + 0.5f) * resolution_
  );
}

void ElevationGrid::insertPoint(float x, float y, float z) {
  auto idx = worldToCell(x, y);
  cells_[idx].addPoint(z);
}

void ElevationGrid::insertPointWithNormal(float x, float y, float z,
                                          const Eigen::Vector3f& normal) {
  auto idx = worldToCell(x, y);
  auto& cell = cells_[idx];
  cell.addPoint(z);
  cell.addNormal(normal);
}

const ElevationCell& ElevationGrid::getCell(const CellIndex& idx) const {
  auto it = cells_.find(idx);
  if (it != cells_.end()) {
    return it->second;
  }
  return empty_cell_;
}

ElevationCell& ElevationGrid::getCellMutable(const CellIndex& idx) {
  return cells_[idx];
}

bool ElevationGrid::hasCell(const CellIndex& idx) const {
  return cells_.find(idx) != cells_.end();
}

const std::unordered_map<CellIndex, ElevationCell, CellIndexHash>&
ElevationGrid::cells() const {
  return cells_;
}

std::unordered_map<CellIndex, ElevationCell, CellIndexHash>&
ElevationGrid::cellsMutable() {
  return cells_;
}

std::vector<CellIndex> ElevationGrid::getNeighbors(const CellIndex& idx) const {
  std::vector<CellIndex> neighbors;
  neighbors.reserve(8);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      CellIndex neighbor{idx.x + dx, idx.y + dy};
      if (hasCell(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
}

size_t ElevationGrid::size() const {
  return cells_.size();
}

}  // namespace traversable_terrain
