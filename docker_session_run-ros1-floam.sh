#!/bin/bash
# Run FLOAM on a rosbag (ROS 1 .bag or ROS 2 bag directory), record output
# topics, then convert the recorded bag to an HDMapping session.
#
# FLOAM publishes the body pose on /odom and the filtered input scan (BODY
# frame) on /velodyne_points_filtered, both stamped with the scan's header
# stamp. We record both, plus /clock as a bag time reference: FLOAM's own clock
# is left untouched and the converter translates the result into bag time if
# needed. The converter transforms every scan into the world frame with its
# matching pose (dense reconstruction). FLOAM's own /map topic republishes the
# entire downsampled map each frame and is NOT recorded.

IMAGE_NAME='floam_noetic'
TMUX_SESSION='ros1_floam'

DATASET_CONTAINER_PATH='/ros_ws/dataset/input.bag'
CONVERTED_BAG_CONTAINER='/tmp/dataset_ros1.bag'
BAG_OUTPUT_CONTAINER='/ros_ws/recordings'

RECORDED_BAG_NAME="recorded-floam.bag"
HDMAPPING_OUT_NAME="output_hdmapping"

# Recorded topics (used by the converter).
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/velodyne_points_filtered}"

# FLOAM parameters (roslaunch args of floam_bench.launch).
# FLOAM subscribes to /velodyne_points; a bag with a different LiDAR topic
# name is remapped on rosbag play via LIDAR_TOPIC.
FLOAM_INPUT_TOPIC='/velodyne_points'
LIDAR_TOPIC="${LIDAR_TOPIC:-$FLOAM_INPUT_TOPIC}"
SCAN_LINE="${SCAN_LINE:-16}"
SCAN_PERIOD="${SCAN_PERIOD:-0.1}"
VERTICAL_ANGLE="${VERTICAL_ANGLE:-2.0}"
MAX_DIS="${MAX_DIS:-90.0}"
MIN_DIS="${MIN_DIS:-3.0}"
MAP_RESOLUTION="${MAP_RESOLUTION:-0.4}"

# RViz on by default — the live view of how FLOAM tracks the dataset.
USE_RVIZ="${USE_RVIZ:-1}"

# Force Mesa software rendering by default so RViz renders even when the host
# GPU driver is not exposed to the container (the libGL "nvidia-drm" / amdgpu case).
LIBGL_SW="${LIBGL_SW:-1}"
if [[ "$LIBGL_SW" == "1" ]]; then LIBGL_ENV="1"; else LIBGL_ENV=""; fi

usage() {
  echo "Usage:"
  echo "  $0 <input.bag-or-ros2bag-dir> <output_dir>"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  LIDAR_TOPIC    - LiDAR topic name inside the bag  (default: /velodyne_points)"
  echo "  SCAN_LINE      - number of LiDAR scan lines, 16|32|64 (default: 16)"
  echo "  SCAN_PERIOD    - scan period in seconds           (default: 0.1)"
  echo "  VERTICAL_ANGLE - vertical angle resolution [deg]  (default: 2.0)"
  echo "  MAX_DIS        - maximum point distance [m]       (default: 90.0)"
  echo "  MIN_DIS        - minimum horizontal distance [m]  (default: 3.0)"
  echo "  MAP_RESOLUTION - map voxel resolution [m]         (default: 0.4)"
  echo "  ODOM_TOPIC     - FLOAM odometry output topic      (default: /odom)"
  echo "  CLOUD_TOPIC    - FLOAM filtered cloud topic       (default: /velodyne_points_filtered)"
  echo "  USE_RVIZ       - 1/0, launch RViz live view       (default: 1)"
  echo "  PLAY_RATE      - rosbag play rate                 (default: 1.0)"
  exit 1
}

echo "=== FLOAM rosbag pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  BAG_OUTPUT_HOST="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --title="Select BAG file (or ROS 2 bag directory)")
  BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -e "$DATASET_HOST_PATH" ]]; then
  echo "Error: input does not exist: $DATASET_HOST_PATH"
  exit 1
fi

mkdir -p "$BAG_OUTPUT_HOST"

DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

# RViz on/off resolved to a roslaunch boolean
RVIZ_ARG=false; [[ "$USE_RVIZ" == "1" ]] && RVIZ_ARG=true

echo "Input           : $DATASET_HOST_PATH"
echo "Output dir      : $BAG_OUTPUT_HOST"
echo "LiDAR topic     : $LIDAR_TOPIC  (FLOAM expects $FLOAM_INPUT_TOPIC)"
echo "scan_line       : $SCAN_LINE"
echo "min/max dis     : $MIN_DIS / $MAX_DIS"
echo "Odom topic      : $ODOM_TOPIC"
echo "Cloud topic     : $CLOUD_TOPIC"

if [[ -d "$DATASET_HOST_PATH" ]]; then
  INPUT_IS_DIR=1
else
  INPUT_IS_DIR=0
fi

xhost +local:docker >/dev/null

# ── Phase 1: run FLOAM + record output topics ────────────────────────────────
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -e USE_RVIZ="$USE_RVIZ" \
  -e LIBGL_ALWAYS_SOFTWARE="$LIBGL_ENV" \
  -e LIDAR_TOPIC="$LIDAR_TOPIC" \
  -e FLOAM_INPUT_TOPIC="$FLOAM_INPUT_TOPIC" \
  -e SCAN_LINE="$SCAN_LINE" \
  -e SCAN_PERIOD="$SCAN_PERIOD" \
  -e VERTICAL_ANGLE="$VERTICAL_ANGLE" \
  -e MAX_DIS="$MAX_DIS" \
  -e MIN_DIS="$MIN_DIS" \
  -e MAP_RESOLUTION="$MAP_RESOLUTION" \
  -e ODOM_TOPIC="$ODOM_TOPIC" \
  -e CLOUD_TOPIC="$CLOUD_TOPIC" \
  -e INPUT_IS_DIR="$INPUT_IS_DIR" \
  -e PLAY_RATE="${PLAY_RATE:-1.0}" \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    source /opt/ros/noetic/setup.bash
    source /ros_ws/devel/setup.bash

    # ── If input is a ROS 2 bag directory, convert to a ROS 1 bag ──────────
    if [[ "$INPUT_IS_DIR" == "1" ]]; then
      echo "[convert] Converting ROS 2 bag to ROS 1 bag format..."
      rm -f '"$CONVERTED_BAG_CONTAINER"'
      rosbags-convert '"$DATASET_CONTAINER_PATH"' --dst '"$CONVERTED_BAG_CONTAINER"' || {
        echo "[convert] ERROR: rosbags-convert failed"; exit 1; }
      ROS1_BAG="'"$CONVERTED_BAG_CONTAINER"'"
    else
      ROS1_BAG="'"$DATASET_CONTAINER_PATH"'"
    fi

    export ROS1_BAG
    echo "[convert] ROS 1 bag ready at: $ROS1_BAG"
    ls -la $ROS1_BAG

    # ── Preflight: FLOAM can only read a sensor_msgs/PointCloud2 LiDAR topic ─
    # Without it FLOAM receives nothing and the run records no output.
    if ! rosbag info "$ROS1_BAG" 2>/dev/null | grep -qE "[[:space:]]$LIDAR_TOPIC[[:space:]]+[0-9]+ msgs[[:space:]]+: sensor_msgs/PointCloud2"; then
      echo "[preflight] ERROR: LiDAR topic $LIDAR_TOPIC (sensor_msgs/PointCloud2) not found in the bag."
      echo "[preflight] PointCloud2 topics in this bag:"
      rosbag info "$ROS1_BAG" 2>/dev/null | grep -E "msgs[[:space:]]+: sensor_msgs/PointCloud2" || echo "  (none)"
      echo "[preflight] For the Bunker DVI dataset use reg-1.bag-pc.bag (LiDAR on /livox/pointcloud),"
      echo "[preflight] or set LIDAR_TOPIC to the LiDAR PointCloud2 topic of your bag."
      exit 42
    fi
    echo "[preflight] LiDAR topic $LIDAR_TOPIC found (sensor_msgs/PointCloud2)"

    # Topic remap arguments for rosbag play, if the bag uses a non-default name
    REMAP_ARGS=""
    if [[ "$LIDAR_TOPIC" != "$FLOAM_INPUT_TOPIC" ]]; then
      REMAP_ARGS="$LIDAR_TOPIC:=$FLOAM_INPUT_TOPIC"
    fi
    export REMAP_ARGS
    echo "[play] rosbag remap args: $REMAP_ARGS"

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANE 0: roscore ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[roscore] starting..."
roscore
'\'' C-m

    # ---------- PANE 1: FLOAM (+ RViz) ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 4
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[floam] launching floam_bench.launch (scan_line='"$SCAN_LINE"', rviz='"$RVIZ_ARG"') ..."
roslaunch floam floam_bench.launch scan_line:='"$SCAN_LINE"' scan_period:='"$SCAN_PERIOD"' vertical_angle:='"$VERTICAL_ANGLE"' max_dis:='"$MAX_DIS"' min_dis:='"$MIN_DIS"' map_resolution:='"$MAP_RESOLUTION"' rviz:='"$RVIZ_ARG"'
'\'' C-m

    # ---------- PANE 2: rosbag record ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 6
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
rm -f '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] start"
rosbag record '"$ODOM_TOPIC"' '"$CLOUD_TOPIC"' /clock -O '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] exit"
'\'' C-m

    # ---------- PANE 3: rosbag play ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 10
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[play] start"
rosbag play --clock --rate ${PLAY_RATE:-1.0} $ROS1_BAG $REMAP_ARGS; tmux wait-for -S BAG_DONE;
echo "[play] done"
'\'' C-m

    # ---------- PANE 4: diagnostics ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 12
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "=== ROS 1 DIAGNOSTICS ==="
echo ""
echo "--- Active topics ---"
rostopic list
echo ""
echo "--- Checking input LiDAR: '"$FLOAM_INPUT_TOPIC"' ---"
timeout 5 rostopic hz '"$FLOAM_INPUT_TOPIC"' 2>&1 &
echo ""
echo "--- Checking FLOAM output: '"$ODOM_TOPIC"' ---"
timeout 5 rostopic hz '"$ODOM_TOPIC"' 2>&1 &
echo ""
echo "--- Checking FLOAM output: '"$CLOUD_TOPIC"' ---"
timeout 5 rostopic hz '"$CLOUD_TOPIC"' 2>&1 &
wait
echo ""
echo "[diag] done — you can type ROS 1 commands here, e.g.:"
echo "  rostopic echo '"$ODOM_TOPIC"'"
'\'' C-m

    # ---------- Control window (window 1) ----------
    # This is the window the user attaches to; the 5 noisy panes are on window 0.
    # It waits for the play pane to signal end of playback, then tears the whole
    # session down.
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[control] waiting for bag playback to finish..."
tmux wait-for BAG_DONE
echo "[control] bag playback finished — shutting down"

# Give FLOAM a moment to process remaining queued scans
sleep 3

# Graceful stop: Ctrl+C to each pane
# Pane layout: 0=roscore, 1=floam+rviz, 2=recorder, 3=play, 4=diag
echo "[control] sending Ctrl+C to all panes..."
tmux send-keys -t '"$TMUX_SESSION"':0.2 C-c
sleep 1
tmux send-keys -t '"$TMUX_SESSION"':0.1 C-c
sleep 1
tmux send-keys -t '"$TMUX_SESSION"':0.0 C-c
sleep 3

# Force-kill by process name
echo "[control] force-killing remaining processes..."
pkill -9 floam            2>/dev/null || true
pkill -9 rviz             2>/dev/null || true
pkill -9 rosmaster        2>/dev/null || true
pkill -9 rosout           2>/dev/null || true
sleep 1

echo "[control] terminating tmux"
tmux kill-server
'\''

    tmux attach -t '"$TMUX_SESSION"'
  '

# The preflight check failed: nothing was recorded, and converting would pick
# up a stale recording from an earlier run.
if [[ $? -eq 42 ]]; then
  echo "=== ABORTED: input check failed, no conversion ==="
  exit 1
fi

# ── Phase 2: convert recorded bag to HDMapping session ────────────────────────
echo "=== Converting recorded bag to HDMapping session ==="

docker run -it --rm \
  --network host \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    set -e
    source /opt/ros/noetic/setup.bash
    source /ros_ws/devel/setup.bash
    rosrun floam_to_hdmapping listener \
      \"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME\" \
      \"$BAG_OUTPUT_CONTAINER/$HDMAPPING_OUT_NAME-floam\" \
      \"$ODOM_TOPIC\" \
      \"$CLOUD_TOPIC\"
  "

echo "=== DONE ==="
