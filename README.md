# iceberg_nav

Navigation and perception package for AUV-based iceberg circumnavigation. Converts sonar data into point clouds, probabilistic occupancy maps, and autonomy-level path and waypoint commands.

## Dependencies

```bash
sudo apt-get install ros-jazzy-tf2-ros ros-jazzy-cv-bridge ros-jazzy-image-transport libboost-all-dev
pip install scipy scikit-learn
```

Clone this [fork](https://github.com/GSO-soslab/navigation2) of nav2 (required for `costmap.launch.py`).

---

## Nodes
---

### `msis_voxels` — `src/msis_voxels.cpp`

Publishes the MSIS sonar field-of-view as a voxel `Marker` grid. Used downstream by `msis_prob_clouds` to assign beam-directivity weights to incoming points.

**Subscribes**
| Type | Description |
|---|---|
| `SonarEcho` | Sonar beam angle updates |

**Publishes**
| Type | Description |
|---|---|
| `Marker` | 3D voxel grid covering the sonar cone |

---

### `msis_prob_clouds` — `iceberg_nav/msis_prob_clouds.py`

Projects MSIS point clouds into the world frame and assigns each point an occupancy probability based on sinc beam directivity and range. Outputs a probabilistic cloud for voxel map ingestion and a range-filter profile for downstream nodes.

**Subscribes**
| Type | Description |
|---|---|
| `Marker` | Voxel FOV geometry from `msis_voxels` |
| `PointCloud2` | Raw point cloud in sensor frame |

**Publishes**
| Type | Description |
|---|---|
| `PointCloud2` | Probabilistic cloud (intensity = occupancy probability) |
| `Float32MultiArray` | Per-beam range-filter profile |

---

### `voxel_log_odds_visualizer` — `src/voxel_log_odds_visualizer.cpp`

Accumulates probabilistic point clouds into a 3D log-odds occupancy voxel map. Transforms incoming clouds into the world frame, updates each voxel's log-odds score, and publishes the occupied voxels as a `PointCloud2` and a 2D `OccupancyGrid` (horizontal slice).

**Subscribes**
| Type | Description |
|---|---|
| `PointCloud2` | Probabilistic cloud from ISM nodes |
| `Odometry` | Vehicle odometry for map-relative bounds |

**Publishes**
| Type | Description |
|---|---|
| `PointCloud2` | 3D occupied voxels |
| `OccupancyGrid` | 2D top-down occupancy grid |

> Best used alongside [fls_ism](https://github.com/GSO-soslab/fls_ism) and [mbes_ism](https://github.com/GSO-soslab/mbes_ism) — feed their probabilistic point cloud outputs directly into this node for multi-sonar occupancy fusion.

---

### `path_gen` — `iceberg_nav/path_gen.py`

Generates a circumnavigation path around the iceberg from a 2D costmap. Applies Canny edge detection to extract the iceberg boundary, fits a standoff curve at a configurable offset, samples waypoints, and publishes the path. Supports a spiral search mode for initial acquisition.

**Subscribes**
| Type | Description |
|---|---|
| `OccupancyGrid` | 2D costmap |

**Publishes**
| Type | Description |
|---|---|
| `Path` | Sampled circumnavigation path |
| `Float32` | Estimated distance to obstacle |
| `Image` | Debug visualization of detected edge and path |

---

### `wp_admin` — `iceberg_nav/wp_admin.py`

Autonomy state machine. Converts the generated path into MVP waypoints and manages transitions between search, follow, and reacquisition modes based on vehicle state and obstacle distance.

**Subscribes**
| Type | Description |
|---|---|
| `Path` | Circumnavigation path from `path_gen` |
| `Float32` | Distance-to-obstacle estimate |
---
