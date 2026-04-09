# Amiga Navigation Package Guide

This document describes the current `amiga_navigation` package layout, usage, tuning workflow, debug workflow, and a short algorithm overview.

It does not replace any historical documentation. It is intended to match the current code in this repository.

## 1. Package Purpose

This package provides a lightweight GPS + IMU waypoint navigation stack for the Amiga robot.

The stack assumes:

- GPS provides global position
- IMU provides heading (`yaw`)
- Waypoints are recorded as latitude/longitude CSV files
- The robot is driven through `/cmd_vel_out`

The current navigation flow is:

1. RTK GPS serial data is parsed and published to `/gps/fix`
2. The first valid GPS fix is latched as `/gps/datum`
3. GPS position and IMU yaw are combined into `/robot/odom`
4. A waypoint follower consumes `/robot/odom` and publishes `/cmd_vel_nav`
5. `twist_mux` arbitrates stop/manual/navigation commands into `/cmd_vel_out`
6. A serial bridge forwards `/cmd_vel_out` to the Amiga MCU

## 2. Main Nodes

Current file names are chosen to reflect their responsibilities directly.

- `gnss_publisher.py`
  - Reads GGA messages from the RTK GNSS serial port
  - Publishes `/gps/fix`
  - Publishes `/gps/rtk_status_flag`
- `datum_publisher.py`
  - Stores the first `/gps/fix` as the local datum
  - Publishes `/gps/datum`
- `imu_publisher.py`
  - Reads the HWT905 IMU over serial
  - Publishes `/imu`
- `gnss2enu_odometry.py`
  - Converts GNSS to local ENU coordinates using `/gps/datum`
  - Combines ENU position with IMU yaw
  - Publishes `/robot/odom`
- `waypoint_follower.py`
  - Loads a waypoint CSV
  - Follows the current waypoint segment using the selected controller
  - Publishes `/cmd_vel_nav`
- `rtk_monitor.py`
  - Monitors RTK FIX status
  - Publishes `/cmd_vel_stop` if RTK is missing, stale, or not FIX
- `amiga_serial_bridge.py`
  - Subscribes to `/cmd_vel_out`
  - Forwards commands to the Amiga MCU over serial
- `gnss_waypoint_keyboard_logger.py`
  - Records current GPS positions to CSV when the user presses the space bar
- `localization_logger.py`
  - Logs `/robot/odom` and `/imu` to CSV for offline analysis
- `robot_odom_logger.py`
  - Logs `/robot/odom` only

## 3. Executable Names

The package now exposes clearer executable names:

- `gnss_publisher`
- `datum_publisher`
- `imu_publisher`
- `gnss2enu_odometry`
- `waypoint_follower`
- `rtk_monitor`
- `amiga_serial_bridge`
- `gnss_waypoint_keyboard_logger`
- `localization_logger`
- `robot_odom_logger`

Legacy executable aliases are still kept for compatibility:

- `gps_publisher`
- `datum_publisher_node`
- `imu_publisher`
- `gps_to_enu_odometry_node`
- `logged_waypoint_follower`
- `rtk_monitor_node`
- `amiga_com`
- `gps_waypoint_logger_keyboard`
- `odometry_logger_node`
- `robot_odom_node`

## 4. Quick Start

Build and source:

```bash
cd /home/cairlab/PPBv2_Navigation
colcon build --packages-select amiga_navigation
source install/setup.bash
```

Start the base stack:

```bash
ros2 launch amiga_navigation basic_bringup.launch.py
```

This launch starts:

- `gnss_publisher`
- `datum_publisher`
- `imu_publisher`
- `gnss2enu_odometry`
- `rtk_monitor`
- `amiga_serial_bridge`
- `twist_mux`

Then start waypoint following in a second terminal:

```bash
source /home/cairlab/PPBv2_Navigation/install/setup.bash
ros2 run amiga_navigation waypoint_follower
```

If you want a different waypoint file:

```bash
ros2 run amiga_navigation waypoint_follower \
  --waypoints /home/cairlab/navigation_waypoints/your_waypoints.csv
```

If an unfinished navigation snapshot exists, the node can ask whether to resume it.

## 5. Recording Waypoints

To record waypoints manually from GPS:

```bash
ros2 launch amiga_navigation waypoints_logger.launch.py
```

Or run the recorder directly:

```bash
ros2 run amiga_navigation gnss_waypoint_keyboard_logger \
  --output_dir /home/cairlab/navigation_waypoints
```

Recorder behavior:

- Press `SPACE` to save the current GPS position
- Press `q` to quit
- `latest_waypoints.csv` is always updated
- A snapshot CSV is also maintained
- On startup, the recorder can ask whether you want to manually name the snapshot file

## 6. Waypoint Follower Parameters

Main parameters live in:

- `config/waypoint_follower_params.yaml`

Important note:

- The ROS node name is `waypoint_follower`
- Therefore the YAML root key is `/waypoint_follower:`

### Most Important Parameters

- `controller_type`
  - Selects the controller
  - Options: `pid_line`, `pure_pursuit`, `mpc_rollout`, `mpc_formal`, `row_hybrid`
- `target_speed`
  - Nominal forward speed
- `pid_kp`, `pid_ki`, `pid_kd`
  - PID gains for line tracking
- `heading_gain`
  - Additional turning correction during tracking
- `max_angular_speed`
  - Angular velocity limit
- `goal_threshold`
  - Distance threshold for waypoint reached
- `alignment_threshold`
  - Heading threshold before tracking starts
- `publish_debug`
  - Enables `/nav/controller_debug`
- `enable_csv_logging`
  - Enables controller CSV logging

### Recommended Low-Resource Default

For Raspberry Pi deployment, the lowest-resource option is:

```yaml
controller_type: pid_line
publish_debug: false
enable_csv_logging: false
```

This avoids the heavier MPC controllers during normal operation.

## 7. Controller Overview

All controllers operate on the current waypoint segment:

- segment start point
- segment end point
- current pose `[x, y, yaw]`

The follower first performs an alignment turn, then enters segment tracking.

### `pid_line`

This is the lightest controller and usually the best starting point for deployment.

Idea:

- Project the robot onto the current segment
- Compute cross-track error
- Compute heading error
- Use PID on cross-track error
- Add heading correction
- Reduce speed when turning sharply or approaching the goal

Expected behavior:

- Good on long vineyard rows
- Lightweight
- Sensitive to tuning when GPS is noisy or low-rate

### `pure_pursuit`

Idea:

- Choose a lookahead point on the current segment
- Steer toward that point

Expected behavior:

- Smoother than PID
- Often less aggressive
- Can cut corners on very short connector segments

### `mpc_rollout`

Idea:

- Sample candidate commands
- Simulate a few steps forward
- Pick the lowest-cost candidate

Expected behavior:

- More flexible than PID on short connectors
- Higher CPU cost than PID and pure pursuit

### `mpc_formal`

Idea:

- Solve a receding-horizon optimization problem over a sequence of controls
- Execute only the first control
- Repeat at the next cycle

Expected behavior:

- Best suited for short connector segments where local maneuvering matters
- Highest CPU cost in this package

### `row_hybrid`

This is the controller designed for vineyard-style waypoint layouts.

Typical layout:

```text
1 ------------------------------ 2
4 ------------------------------ 3
```

The intent is:

- long row segments use `pid_line`
- short row-to-row connectors use `mpc_formal`
- intermediate segments can use `pure_pursuit`

This makes the package better suited for:

- long straight rows
- short transitions between neighboring rows

## 8. Practical Tuning Workflow

Start simple. Do not tune everything at once.

### Step 1: Use `pid_line`

Set:

```yaml
controller_type: pid_line
pid_ki: 0.0
publish_debug: false
enable_csv_logging: false
```

### Step 2: Start conservatively

Example conservative starting point:

```yaml
target_speed: 0.5
pid_kp: 0.8
pid_ki: 0.0
pid_kd: 0.25
heading_gain: 1.8
max_angular_speed: 1.2
goal_threshold: 0.4
alignment_threshold: 0.12
dist_start_threshold: 2.0
dist_stop_threshold: 1.5
```

### Step 3: Tune by observed behavior

If the robot takes too long to converge back to the row:

- decrease `target_speed`
- increase `pid_kp`
- increase `heading_gain`
- increase `max_angular_speed`

If the robot oscillates left and right:

- decrease `pid_kp`
- increase `pid_kd`
- decrease `heading_gain`

If the robot overshoots near the end of a row:

- increase `dist_stop_threshold`
- increase `goal_threshold`
- decrease `target_speed`

If the robot starts too aggressively:

- decrease `initial_speed_ratio`
- decrease `target_speed`

### Step 4: Only then test other controllers

After a stable PID baseline is established, compare:

- `pure_pursuit`
- `row_hybrid`
- `mpc_formal`

## 9. Debug Workflow

### Basic health checks

Check that the base topics are alive:

```bash
ros2 topic hz /gps/fix
ros2 topic hz /imu
ros2 topic hz /robot/odom
ros2 topic echo /gps/rtk_status_flag
```

### Check command chain

```bash
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_out
```

### Enable follower debug temporarily

Set in the parameter file:

```yaml
publish_debug: true
enable_csv_logging: true
```

Then run:

```bash
ros2 topic echo /nav/controller_debug
```

The debug topic is useful for:

- current phase
- active controller
- waypoint index
- command values
- tracking errors

### Debug launches

The package includes one dedicated debug launch:

- `nav_debug.launch.py`

It starts the base stack plus the debug loggers, but still leaves
`waypoint_follower` to be started manually in another terminal.

## 10. Common Failure Modes

### No `/robot/odom`

Check:

- `/gps/fix` exists
- `/imu` exists
- `/gps/datum` is published
- `gnss2enu_odometry` is running

### RTK stop is always active

Check:

- `/gps/rtk_status_flag`
- GPS is actually outputting FIX-quality GGA data

### Robot does not move even though follower is running

Check:

- `/cmd_vel_nav`
- `/cmd_vel_out`
- `twist_mux`
- `rtk_monitor`
- `amiga_serial_bridge`

### Robot converges too slowly to the row

Likely causes:

- GPS position updates are too low-rate
- `target_speed` is too high
- PID gains are too soft
- heading correction is too weak

This is usually not just a ROS software delay issue.

## 11. Resource Usage Recommendations

If the priority is low CPU and simple deployment:

- prefer `pid_line`
- disable debug publishing
- disable CSV logging
- avoid `mpc_formal` for normal field operation unless needed

Suggested production settings:

```yaml
controller_type: pid_line
publish_debug: false
enable_csv_logging: false
```

## 12. Suggested Command Set

Base bringup:

```bash
ros2 launch amiga_navigation basic_bringup.launch.py
```

Waypoint recording:

```bash
ros2 run amiga_navigation gnss_waypoint_keyboard_logger
```

Waypoint following:

```bash
ros2 run amiga_navigation waypoint_follower
```

Waypoint following with a specific controller:

```bash
ros2 run amiga_navigation waypoint_follower --controller pid_line
```

Waypoint following with a specific file:

```bash
ros2 run amiga_navigation waypoint_follower \
  --waypoints /home/cairlab/navigation_waypoints/latest_waypoints.csv
```

## 13. Notes

- The package still keeps compatibility aliases for older executable names.
- The current code is intentionally lightweight and oriented toward outdoor GPS + IMU navigation.
- For Raspberry Pi deployment, simpler controllers are generally more reliable than heavier optimizers unless short connector maneuvering clearly requires MPC.
