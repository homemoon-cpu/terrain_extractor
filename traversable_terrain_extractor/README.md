# Traversable Terrain Extractor

ROS2 package for real-time traversable terrain extraction from LiDAR point clouds. Classifies ground, stairs, ramps, and non-traversable areas. Designed to work with **LIO-SAM** and **Livox Mid-360**.

## Overview

在 LIO-SAM 建图过程中，实时从激光雷达点云中提取可通行区域（地面、楼梯、坡道），构建可通行地图，并提供保存服务。输出可直接用于 nav2 导航。

### Processing Pipeline

```
PointCloud2 (LIO-SAM registered)
    │
    ▼
┌─────────────────┐
│  Preprocessing   │  PassThrough(Z) → VoxelGrid(0.1m) → DistanceFilter → SOR
└────────┬────────┘
         ▼
┌─────────────────┐
│ Ground Segment.  │  Patchwork (default) or RANSAC
└────────┬────────┘
         ▼
┌─────────────────┐
│ Normal Estimate  │  pcl::NormalEstimationOMP, radius=0.3m
└────────┬────────┘
         ▼
┌─────────────────┐
│ Feature Compute  │  Slope, Roughness, Curvature, HeightVar
│                  │  + Stair Detection (periodic height jumps)
│                  │  + Ramp Detection (region growing)
└────────┬────────┘
         ▼
┌─────────────────┐
│ Classification   │  GROUND / STAIRS / RAMP / NON_TRAVERSABLE / UNKNOWN
└────────┬────────┘
         ▼
┌─────────────────┐
│ Map Accumulator  │  Voxel hash + EMA smoothing + sliding window (80m)
└────────┬────────┘
         ▼
┌─────────────────┐
│ OccupancyGrid    │  3D → 2D projection, nav2 compatible
└─────────────────┘
```

## Terrain Classes

| Class | Value | OccupancyGrid | Color (RViz) |
|-------|-------|---------------|--------------|
| GROUND | 1 | 0 (free) | 🟢 Green |
| STAIRS | 2 | 50 | 🔵 Blue |
| RAMP | 3 | 25 | 🟡 Yellow |
| NON_TRAVERSABLE | 4 | 100 (occupied) | 🔴 Red |
| UNKNOWN | 0 | -1 | ⚪ Gray |

## Dependencies

- ROS2 (Humble / Iron / Jazzy)
- PCL (>= 1.12)
- Eigen3
- OpenMP
- LIO-SAM (for input point clouds)

## Build

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone git@github.com:homemoon-cpu/terrain_extractor.git

# Build
cd ~/ros2_ws
colcon build --packages-select traversable_terrain_extractor

# Source
source install/setup.bash
```

## Usage

### Launch

```bash
# Default launch
ros2 launch traversable_terrain_extractor terrain_extractor.launch.py

# With rosbag replay (simulation time)
ros2 launch traversable_terrain_extractor terrain_extractor.launch.py use_sim_time:=true

# Custom config file
ros2 launch traversable_terrain_extractor terrain_extractor.launch.py \
  params_file:=/path/to/custom_params.yaml
```

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| **Subscribe** | | |
| `/lio_sam/mapping/cloud_registered` | `PointCloud2` | LIO-SAM registered point cloud |
| `/lio_sam/mapping/odometry` | `Odometry` | Robot odometry |
| **Publish** | | |
| `~/traversable_cloud` | `PointCloud2` | Traversable points (accumulated) |
| `~/classified_cloud` | `PointCloud2` | All points with terrain labels |
| `~/ground_cloud` | `PointCloud2` | Ground-only points (debug) |
| `~/occupancy_grid` | `OccupancyGrid` | 2D traversability grid |
| `~/terrain_markers` | `MarkerArray` | RViz color-coded visualization |

### Save Map Service

```bash
# Save all formats
ros2 service call /terrain_extractor/save_map \
  traversable_terrain_extractor/srv/SaveTraversableMap \
  "{save_pcd: true, save_occupancy_grid: true, save_csv: true, traversable_only: false}"

# Save only traversable points as PCD
ros2 service call /terrain_extractor/save_map \
  traversable_terrain_extractor/srv/SaveTraversableMap \
  "{save_pcd: true, save_occupancy_grid: false, save_csv: false, traversable_only: true}"

# Custom output directory
ros2 service call /terrain_extractor/save_map \
  traversable_terrain_extractor/srv/SaveTraversableMap \
  "{output_directory: '/home/user/maps', save_pcd: true, save_occupancy_grid: true, save_csv: false, traversable_only: false}"
```

Output files (timestamped):
- `terrain_YYYYMMDD_HHMMSS.pcd` — 3D annotated point cloud
- `terrain_YYYYMMDD_HHMMSS.pgm` + `.yaml` — 2D occupancy grid (nav2 compatible)
- `terrain_YYYYMMDD_HHMMSS.csv` — Feature CSV for analysis

The PGM+YAML output can be directly loaded by `nav2_map_server`:
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/tmp/traversable_maps/terrain_20260325_143000.yaml
```

## Configuration

All parameters are configurable via `config/terrain_params.yaml`. Key parameters:

### Preprocessing
| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_size` | 0.1 m | Downsampling resolution |
| `min_distance` / `max_distance` | 0.5 / 50.0 m | Horizontal range filter |
| `min_z` / `max_z` | -5.0 / 10.0 m | Vertical range filter |

### Ground Segmentation
| Parameter | Default | Description |
|-----------|---------|-------------|
| `method` | `patchwork` | `patchwork` or `ransac` |
| `pw_num_rings` | 4 | Concentric ring divisions |
| `pw_num_sectors` | 16 | Sector divisions per ring |
| `pw_uprightness_thresh` | 0.7 | Normal z-component threshold (cos 45°) |

### Terrain Classification
| Parameter | Default | Description |
|-----------|---------|-------------|
| `ground_max_slope_deg` | 15° | Max slope for ground |
| `ground_max_roughness` | 0.05 m | Max roughness for ground |
| `max_traversable_slope_deg` | 30° | Overall traversability limit |
| `stairs_traversable` | true | Treat stairs as traversable |
| `ramps_traversable` | true | Treat ramps as traversable |

### Map Accumulator
| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_size` | 0.15 m | Accumulation voxel resolution |
| `sliding_window_radius` | 80 m | Memory management radius |
| `ema_alpha` | 0.3 | Feature smoothing coefficient |

## Package Structure

```
traversable_terrain_extractor/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── terrain_params.yaml
├── launch/
│   └── terrain_extractor.launch.py
├── srv/
│   └── SaveTraversableMap.srv
├── include/traversable_terrain_extractor/
│   ├── types.hpp                    # PointXYZITerrain, TerrainClass, ElevationCell
│   ├── terrain_extractor_node.hpp   # Main node
│   ├── preprocessing.hpp            # VoxelGrid, PassThrough, SOR
│   ├── ground_segmentation.hpp      # RANSAC / Patchwork
│   ├── normal_estimator.hpp         # OMP parallel normal estimation
│   ├── feature_computer.hpp         # Slope, roughness, stair/ramp detection
│   ├── terrain_classifier.hpp       # Threshold-based classification
│   ├── elevation_grid.hpp           # Sparse elevation grid
│   ├── map_accumulator.hpp          # Incremental voxel map
│   └── occupancy_grid_projector.hpp # 3D → 2D projection
└── src/
    ├── main.cpp
    └── *.cpp                        # Implementation files
```

## Livox Mid-360 Notes

- Non-repetitive scanning → single frame is sparse, multi-frame accumulation is essential
- LIO-SAM handles undistortion and registration, outputs clean map-frame clouds
- Normal estimation uses larger search radius (0.3m) to compensate for sparsity
- `min_observations >= 2` required before final classification

## Performance

Target: 5 Hz processing at ~20K points/frame

| Stage | Budget |
|-------|--------|
| Preprocessing | < 5 ms |
| Ground Segmentation | < 15 ms |
| Normal Estimation | < 20 ms |
| Feature Computation | < 10 ms |
| Classification | < 2 ms |
| Map Accumulation | < 10 ms |
| Grid Projection | < 5 ms |
| **Total** | **< 67 ms (budget 200 ms)** |

## License

MIT
