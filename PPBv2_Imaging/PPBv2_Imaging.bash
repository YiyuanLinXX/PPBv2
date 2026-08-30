#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Run all PPBv2 imaging programs: GPS publisher, GPS logger, and cameras
# -----------------------------------------------------------------------------


# =============================================================================
# USER SETTINGS
# Edit the values in this section when hardware or camera settings change.
# Everything below this section can normally be left unchanged.
# =============================================================================

# Folder containing this launcher and the built PPBv2 Imaging program.
# Override with PPBV2_IMAGING_WORKSPACE only if the launcher is stored elsewhere.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${PPBV2_IMAGING_WORKSPACE:-$SCRIPT_DIR}"

# Parent folder where each data-collection folder will be saved.
# USER normally contains the account running this script.
CURRENT_USER="${USER:-$(id -un)}"
DATA_ROOT="${PPBV2_DATA_ROOT:-/media/Data/$CURRENT_USER}"

# Arduino connection
ARDUINO_PORT="/dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_F412FA9CA7F8-if01"
ARDUINO_BAUD=9600

# Camera settings
EXPOSURE_TIME=250.0       # Microseconds
GAIN=5.0
WB_RED=1.34              # Red white balance
WB_BLUE=2.98             # Blue white balance

# Image file settings
# IMAGE_FORMAT can be "png", "jpg", or "pgm".
#
# REFERENCE PROFILES
# ------------------
# 2 cameras, 4096x3000, 2 FPS, RGB files (tested):
#   IMAGE_FORMAT=jpg
#   JPEG_QUALITY=95
#   JPEG_SUBSAMPLING=2
#   SAVE_QUEUE_DEPTH=4
#
# 4 cameras, 4096x3000, 2 FPS, safest current configuration:
#   IMAGE_FORMAT=pgm
#   SAVE_QUEUE_DEPTH=4
#   JPEG settings are ignored. PGM stores raw BayerRG8 and can be converted
#   to RGB offline. Camera detection is automatic; no camera-count setting is
#   required.
#
# 4 cameras, full-resolution RGB JPEG (experimental, not guaranteed at 2 FPS):
#   IMAGE_FORMAT=jpg
#   JPEG_QUALITY=85
#   JPEG_SUBSAMPLING=2
#   SAVE_QUEUE_DEPTH=4
#   Watch for "Image save queue is full". Increasing queue depth only delays
#   overload; it does not increase sustained encoding speed. Full-resolution
#   4-camera RGB at 2 FPS requires hardware JPEG encoding for reliable use.
#
# PNG is not suitable for real-time full-resolution multi-camera acquisition.
IMAGE_FORMAT="${IMAGE_FORMAT:-jpg}"
JPEG_QUALITY=95           # Range: 1 to 100
JPEG_SUBSAMPLING=2        # 0=4:4:4, 1=4:2:2, 2=4:2:0 (fastest)
PNG_COMPRESS_LEVEL=3      # Range: 0 to 9
SAVE_QUEUE_DEPTH=4        # Maximum batches waiting for background writers
TIMESTAMP_RECALIBRATION_SEC=60.0
CAMERA_MAX_CONSECUTIVE_FAILURES=5
CROSS_CAMERA_SYNC_TOLERANCE_MS=20.0
GPS_FAILURE_ABORT_SEC=3.0
STATUS_LOG_EVERY_N_FRAMES=20

# Keep this self-contained acquisition system off the LAN-wide DDS discovery
# domain. On this Jetson, external Fast DDS discovery caused multi-gigabyte
# transient allocations and eventually an out-of-memory kill.
ROS_LOCALHOST_ONLY_SETTING=1
ROS_DOMAIN_ID_SETTING=77

# Per-process address-space safety limit. A middleware/SDK allocation failure
# will stop that node and the supervisor will then stop the entire acquisition.
PROCESS_VIRTUAL_MEMORY_LIMIT_KIB=8388608  # 8 GiB per process

# GPS connection
GPS_PORT="/dev/serial/by-id/usb-FTDI_TTL-232R-5V-WE_ABB8YMCQ-if00-port0"
GPS_BAUD=115200

# UM982 dual-antenna settings
UM982_BASELINE_M=1.18                     # Measured distance between antennas, meters
UM982_BASELINE_TOLERANCE_M=0.1            # Allowed distance error, in meters
UM982_ANTENNA_BASELINE_ANGLE_DEG=-90.0    # ANT2 is left of ANT1; output is robot-forward heading
UM982_HEADING_OFFSET_DEG=0.0              # Heading calibration adjustment
UM982_PITCH_MULTIPLIER=1.0                # Use -1.0 if pitch direction is reversed
UM982_OUTPUT_PERIOD_SEC=0.1                # One GPS update every 0.1 seconds


# =============================================================================
# PROGRAM STARTS HERE
# =============================================================================

# 1. Load ROS 2 and the PPBv2 Imaging program
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble is not installed or cannot be found."
  exit 1
fi

if [[ ! -r "$WORKSPACE/install/setup.bash" ]]; then
  echo "ERROR: The PPBv2 Imaging program has not been built."
  echo "Expected file: $WORKSPACE/install/setup.bash"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

export ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY_SETTING"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_SETTING"

GPS_PUBLISHER_EXE="$WORKSPACE/install/multi_camera_trigger/lib/multi_camera_trigger/gps_publisher"
GPS_LOGGER_EXE="$WORKSPACE/install/multi_camera_trigger/lib/multi_camera_trigger/gps_logger"
CAMERA_EXE="$WORKSPACE/install/multi_camera_trigger/lib/multi_camera_trigger/multi_camera_trigger_node"

for REQUIRED_COMMAND in setsid prlimit; do
  if ! command -v "$REQUIRED_COMMAND" >/dev/null 2>&1; then
    echo "ERROR: Required command is unavailable: $REQUIRED_COMMAND"
    exit 1
  fi
done

for REQUIRED_EXE in "$GPS_PUBLISHER_EXE" "$GPS_LOGGER_EXE" "$CAMERA_EXE"; do
  if [[ ! -x "$REQUIRED_EXE" ]]; then
    echo "ERROR: Installed program is missing or not executable: $REQUIRED_EXE"
    exit 1
  fi
done

PROCESS_VIRTUAL_MEMORY_LIMIT_BYTES=$((PROCESS_VIRTUAL_MEMORY_LIMIT_KIB * 1024))


# 2. Check the selected image format
IMAGE_FORMAT="${IMAGE_FORMAT,,}"
if [[ "$IMAGE_FORMAT" == "jpeg" ]]; then
  IMAGE_FORMAT="jpg"
fi

if [[ "$IMAGE_FORMAT" != "png" && "$IMAGE_FORMAT" != "jpg" && "$IMAGE_FORMAT" != "pgm" ]]; then
  echo "ERROR: IMAGE_FORMAT must be png, jpg, or pgm."
  exit 1
fi

if [[ "$IMAGE_FORMAT" == "png" ]]; then
  echo "WARNING: Full-resolution PNG encoding cannot sustain the 2 FPS target"
  echo "on this system. Frames will accumulate in the bounded save queue and"
  echo "acquisition will slow down when the queue becomes full."
  if [[ "${ALLOW_SLOW_PNG:-0}" != "1" ]]; then
    read -r -p "Continue with PNG anyway? [y/N]: " PNG_CONFIRM
    if [[ "${PNG_CONFIRM,,}" != "y" && "${PNG_CONFIRM,,}" != "yes" ]]; then
      echo "Cancelled. Use the default JPEG format for 2 FPS acquisition."
      exit 1
    fi
  fi
fi


# 3. Ask for a new output-folder name
echo "Example folder name: Oblock_20260823_0914"
read -r -p "Enter a new output folder name: " USER_TAG

if [[ -z "$USER_TAG" ]]; then
  echo "ERROR: The folder name cannot be empty."
  exit 1
fi

if [[ ! "$USER_TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "ERROR: Use only letters, numbers, dots, underscores, and dashes."
  exit 1
fi

OUTPUT_DIR="$DATA_ROOT/$USER_TAG"
GPS_LOG="$OUTPUT_DIR/gps_log.csv"

if [[ -e "$OUTPUT_DIR" ]]; then
  echo "ERROR: This output folder already exists:"
  echo "$OUTPUT_DIR"
  echo "Please run the script again and choose a new folder name."
  exit 1
fi

mkdir -p "$DATA_ROOT"
mkdir "$OUTPUT_DIR"

echo
echo "Data will be saved in: $OUTPUT_DIR"
echo "Image format: $IMAGE_FORMAT"
echo


# 4. Remember each program so they can all be stopped with Ctrl+C
PIDS=()
declare -A PROCESS_NAMES=()
STOPPING_PROGRAMS=0

stop_all_programs() {
  if (( STOPPING_PROGRAMS )); then
    return
  fi
  STOPPING_PROGRAMS=1
  trap - INT TERM HUP
  echo
  echo "Stopping GPS, cameras, and Arduino trigger..."
  for PID in "${PIDS[@]}"; do
    kill -TERM -- "-$PID" 2>/dev/null
  done

  STOP_DEADLINE=$((SECONDS + 15))
  while (( SECONDS < STOP_DEADLINE )); do
    ANY_RUNNING=0
    for PID in "${PIDS[@]}"; do
      if kill -0 "$PID" 2>/dev/null; then
        ANY_RUNNING=1
        break
      fi
    done
    if (( ! ANY_RUNNING )); then
      break
    fi
    sleep 0.1
  done

  for PID in "${PIDS[@]}"; do
    if kill -0 "$PID" 2>/dev/null; then
      echo "WARNING: ${PROCESS_NAMES[$PID]} did not stop gracefully; forcing it down."
      kill -KILL -- "-$PID" 2>/dev/null
    fi
  done
  wait "${PIDS[@]}" 2>/dev/null
  echo "All programs stopped."
}

handle_ctrl_c() {
  stop_all_programs
  exit 130
}

trap handle_ctrl_c INT TERM HUP

start_program() {
  local program_name="$1"
  shift
  echo "Starting $program_name..."
  setsid prlimit --as="$PROCESS_VIRTUAL_MEMORY_LIMIT_BYTES" -- "$@" &
  local program_pid=$!
  PIDS+=("$program_pid")
  PROCESS_NAMES["$program_pid"]="$program_name"
}

# 5. Start the GPS publisher
start_program "GPS publisher" \
  "$GPS_PUBLISHER_EXE" \
  --ros-args \
    -p port:="$GPS_PORT" \
    -p baud:="$GPS_BAUD" \
    -p configure_receiver_on_start:=true \
    -p receiver_output_period_sec:="$UM982_OUTPUT_PERIOD_SEC" \
    -p baseline_m:="$UM982_BASELINE_M" \
    -p baseline_tolerance_m:="$UM982_BASELINE_TOLERANCE_M" \
    -p antenna_baseline_angle_deg:="$UM982_ANTENNA_BASELINE_ANGLE_DEG" \
    -p heading_offset_deg:="$UM982_HEADING_OFFSET_DEG" \
    -p pitch_multiplier:="$UM982_PITCH_MULTIPLIER"


# 6. Start the GPS logger
start_program "GPS logger" \
  "$GPS_LOGGER_EXE" \
  --ros-args \
    -p log_file:="$GPS_LOG" \
    -p overwrite_existing:=false


# 7. Start the cameras and Arduino trigger
start_program "cameras and Arduino trigger" \
  "$CAMERA_EXE" \
  --ros-args \
    -p output_dir:="$OUTPUT_DIR" \
    -p image_format:="$IMAGE_FORMAT" \
    -p jpeg_quality:="$JPEG_QUALITY" \
    -p jpeg_subsampling:="$JPEG_SUBSAMPLING" \
    -p png_compress_level:="$PNG_COMPRESS_LEVEL" \
    -p save_queue_depth:="$SAVE_QUEUE_DEPTH" \
    -p timestamp_recalibration_sec:="$TIMESTAMP_RECALIBRATION_SEC" \
    -p camera_max_consecutive_failures:="$CAMERA_MAX_CONSECUTIVE_FAILURES" \
    -p cross_camera_sync_tolerance_ms:="$CROSS_CAMERA_SYNC_TOLERANCE_MS" \
    -p gps_failure_abort_sec:="$GPS_FAILURE_ABORT_SEC" \
    -p status_log_every_n_frames:="$STATUS_LOG_EVERY_N_FRAMES" \
    -p overwrite_existing:=false \
    -p arduino_port:="$ARDUINO_PORT" \
    -p arduino_baud:="$ARDUINO_BAUD" \
    -p exposure_time:="$EXPOSURE_TIME" \
    -p gain:="$GAIN" \
    -p wb_red:="$WB_RED" \
    -p wb_blue:="$WB_BLUE"


# 8. Keep running until Ctrl+C is pressed or one program stops
echo
echo "All programs started successfully."
echo "Press Ctrl+C to stop data collection."

STOPPED_PID=''
wait -n -p STOPPED_PID "${PIDS[@]}"
PROGRAM_STATUS=$?
STOPPED_NAME="${PROCESS_NAMES[$STOPPED_PID]:-unknown program}"
if (( PROGRAM_STATUS == 0 )); then
  PROGRAM_STATUS=1
fi

stop_all_programs

echo "ERROR: $STOPPED_NAME stopped unexpectedly (status $PROGRAM_STATUS)."

exit "$PROGRAM_STATUS"
