#!/usr/bin/env bash

# Run GPS publishing/logging and synchronized multi-camera acquisition.
set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${PPBV2_IMAGING_WORKSPACE:-$SCRIPT_DIR}"
DATA_ROOT="${PPBV2_DATA_ROOT:-/media/Data/cairlab}"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble setup file was not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -r "$WORKSPACE/install/setup.bash" ]]; then
  echo "Workspace is not built: $WORKSPACE/install/setup.bash" >&2
  echo "Run 'cd $WORKSPACE && colcon build' first." >&2
  exit 1
fi

# ROS setup scripts are not guaranteed to be safe under `set -u`.
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

read -r -p \
  "Enter output folder name (for example Oblock_20260822_0914): " USER_TAG
if [[ ! "$USER_TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "Invalid folder name. Use only letters, numbers, dot, underscore, and dash." >&2
  exit 1
fi

IMAGE_FORMAT="${IMAGE_FORMAT:-}"
if [[ -z "$IMAGE_FORMAT" ]]; then
  read -r -p "Image format [png/jpg/pgm] (default: png): " IMAGE_FORMAT
fi
IMAGE_FORMAT="${IMAGE_FORMAT:-png}"
IMAGE_FORMAT="${IMAGE_FORMAT,,}"
if [[ "$IMAGE_FORMAT" == "jpeg" ]]; then
  IMAGE_FORMAT="jpg"
fi
if [[ "$IMAGE_FORMAT" != "png" && "$IMAGE_FORMAT" != "jpg" && "$IMAGE_FORMAT" != "pgm" ]]; then
  echo "Unsupported image format: $IMAGE_FORMAT (choose png, jpg, or pgm)." >&2
  exit 1
fi

OUTPUT_DIR="$DATA_ROOT/$USER_TAG"
GPS_LOG="$OUTPUT_DIR/gps_log.csv"
mkdir -p "$DATA_ROOT"
if [[ -e "$OUTPUT_DIR" ]]; then
  echo "Refusing to overwrite existing output directory: $OUTPUT_DIR" >&2
  exit 1
fi
mkdir "$OUTPUT_DIR"

# Hardware and acquisition parameters.
ARDUINO_PORT="/dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_F412FA67A978-if01"
ARDUINO_BAUD=9600
EXPOSURE_TIME=250.0
GAIN=5.0
WB_RED=1.34
WB_BLUE=2.98
JPEG_QUALITY=95
PNG_COMPRESS_LEVEL=3

GPS_PORT="${PPBV2_UM982_IMAGING_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
GPS_BAUD=115200
UM982_BASELINE_M=1.28
UM982_BASELINE_TOLERANCE_M=0.1
UM982_ANTENNA_BASELINE_ANGLE_DEG=0.0
UM982_HEADING_OFFSET_DEG=0.0
UM982_PITCH_MULTIPLIER=1.0
UM982_OUTPUT_PERIOD_SEC=0.1

PIDS=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if (( ${#PIDS[@]} > 0 )); then
    kill -TERM "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
  exit "$status"
}

handle_signal() {
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

echo "Save directory: $OUTPUT_DIR"
echo "Image format: $IMAGE_FORMAT"

echo "Launching gps_publisher..."
ros2 run multi_camera_trigger gps_publisher \
  --ros-args \
    -p port:="$GPS_PORT" \
    -p baud:="$GPS_BAUD" \
    -p configure_receiver_on_start:=true \
    -p receiver_output_period_sec:="$UM982_OUTPUT_PERIOD_SEC" \
    -p baseline_m:="$UM982_BASELINE_M" \
    -p baseline_tolerance_m:="$UM982_BASELINE_TOLERANCE_M" \
    -p antenna_baseline_angle_deg:="$UM982_ANTENNA_BASELINE_ANGLE_DEG" \
    -p heading_offset_deg:="$UM982_HEADING_OFFSET_DEG" \
    -p pitch_multiplier:="$UM982_PITCH_MULTIPLIER" &
PIDS+=("$!")

echo "Launching gps_logger..."
ros2 run multi_camera_trigger gps_logger \
  --ros-args \
    -p log_file:="$GPS_LOG" \
    -p overwrite_existing:=false &
PIDS+=("$!")

echo "Launching multi_camera_trigger_node..."
ros2 run multi_camera_trigger multi_camera_trigger_node \
  --ros-args \
    -p output_dir:="$OUTPUT_DIR" \
    -p image_format:="$IMAGE_FORMAT" \
    -p jpeg_quality:="$JPEG_QUALITY" \
    -p png_compress_level:="$PNG_COMPRESS_LEVEL" \
    -p overwrite_existing:=false \
    -p arduino_port:="$ARDUINO_PORT" \
    -p arduino_baud:="$ARDUINO_BAUD" \
    -p exposure_time:="$EXPOSURE_TIME" \
    -p gain:="$GAIN" \
    -p wb_red:="$WB_RED" \
    -p wb_blue:="$WB_BLUE" &
PIDS+=("$!")

echo "All nodes started. Press Ctrl+C to stop."

# A healthy acquisition requires all three nodes. Stop the group if any one
# exits, so a camera failure cannot leave orphaned logger processes behind.
set +e
wait -n "${PIDS[@]}"
STATUS=$?
set -e

if (( STATUS == 0 )); then
  echo "A node exited; stopping the remaining nodes."
else
  echo "A node failed with status $STATUS; stopping the remaining nodes." >&2
fi
exit "$STATUS"
