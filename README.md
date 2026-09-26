## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-FLOAM-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.

## Example Dataset:

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

# benchmark-FLOAM-to-HDMapping

Runs the [FLOAM](https://github.com/wh200720041/floam) LiDAR odometry algorithm
on a ROS 1 bag file and converts the output to an
[HDMapping](https://github.com/MapsHD/HDMapping) session.

FLOAM (*Fast LOAM*, H. Wang et al.) is a lightweight, pure-LiDAR odometry and
mapping method — a computationally efficient LOAM variant using edge/planar
feature extraction with an analytical-Jacobian scan-to-map optimization. No IMU
is used. Scan lines are computed from each point's vertical angle, so plain
`sensor_msgs/PointCloud2` input without a `ring` field works.

## Prerequisites

- Docker
- A ROS 1 bag containing a `sensor_msgs/PointCloud2` LiDAR topic
  (ROS 2 bags are automatically converted to ROS 1 format)

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-FLOAM-to-HDMapping.git --recursive
cd benchmark-FLOAM-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t floam_noetic .
```

This installs:
- Ubuntu 20.04 + ROS 1 Noetic
- Eigen3, PCL, Ceres (system `libceres-dev`)
- FLOAM (compiled from submodule)
- catkin workspace with `floam` and `floam_to_hdmapping`

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-ros1-floam.sh
./docker_session_run-ros1-floam.sh /path/to/input.bag /path/to/output/dir
```

Or with no arguments to use a GUI file selector (requires `zenity`).

FLOAM's parameters are set with environment variables:

| Variable | Default | Meaning |
|----------|---------|---------|
| `LIDAR_TOPIC` | `/velodyne_points` | LiDAR topic name **in the bag** (remapped to FLOAM's input) |
| `SCAN_LINE` | `16` | number of LiDAR scan lines — FLOAM supports only `16`, `32` or `64` |
| `SCAN_PERIOD` | `0.1` | scan period [s] |
| `VERTICAL_ANGLE` | `2.0` | vertical angular resolution [deg] |
| `MAX_DIS` | `90.0` | maximum point distance [m] |
| `MIN_DIS` | `3.0` | minimum point distance [m] |
| `MAP_RESOLUTION` | `0.4` | map voxel resolution [m] |
| `USE_RVIZ` | `1` | RViz live view |
| `PLAY_RATE` | `1.0` | rosbag play rate (lower it if FLOAM cannot keep up) |

Example (Velodyne HDL-64, KITTI-style bag):

```bash
SCAN_LINE=64 LIDAR_TOPIC=/kitti/velo/pointcloud \
  ./docker_session_run-ros1-floam.sh /path/to/input.bag /path/to/output/dir
```

**What happens:**

The script opens a Docker container with a tmux session containing five panes on
window 0 and a `control` window (window 1, the attach target):

| Pane | Role |
|------|------|
| 0 | `roscore` |
| 1 | `roslaunch floam floam_bench.launch` — the three FLOAM nodes (+ RViz live view) |
| 2 | `rosbag record /odom /velodyne_points_filtered /clock` |
| 3 | `rosbag play --clock` — plays your input bag |
| 4 | diagnostics — shows active topics and publishing rates |

When playback finishes, the control window stops the recorder, kills all nodes
and RViz, and exits tmux. A second Docker run then converts the recorded bag
into the HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-FLOAM/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
...
session.json
trajectory_lio_0.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

## Notes on FLOAM

FLOAM publishes:

| Topic | Type | Meaning |
|-------|------|---------|
| `/odom` | `nav_msgs/Odometry` | the 6-DoF body pose in the `map` (world) frame |
| `/velodyne_points_filtered` | `sensor_msgs/PointCloud2` | the filtered input scan in the `base_link` (body) frame |
| `/map` | `sensor_msgs/PointCloud2` | the entire accumulated downsampled map, republished every frame |

The converter records `/odom` + `/velodyne_points_filtered` and transforms each
scan into the world frame with its matching pose — recording `/map` would
duplicate the whole map thousands of times.

**Timestamps:** the benchmark does not change FLOAM's clock (no
`use_sim_time`). FLOAM stamps its outputs with the input scan's header stamp,
so they are already in bag time; `/clock` is recorded as a reference and the
converter translates the result into bag time only if it is not (it reports
which case applied). Because FLOAM's `map → base_link` TF is stamped with
wall-clock time, RViz runs with fixed frame `map`, where the `/map` cloud is
published.

## Contact

januszbedkowski@gmail.com
