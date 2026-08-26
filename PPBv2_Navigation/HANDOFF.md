# Dual-antenna UM982 navigation handoff

This document is the technical handoff for maintainers and coding agents
working on this repository. Read it together with `README.md` before changing
the navigation or GNSS pipeline.

## 1. Project goal and verified state

This is a ROS 2 waypoint-navigation stack for a Farm-ng Amiga using:

- Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy in the deployed system;
- one UM982 dual-antenna GNSS receiver;
- two GNSS antennas connected as ANT1 and ANT2;
- RTK corrections received by NTRIP and forwarded to the UM982;
- an Adafruit Feather M4 CAN board as the velocity-command bridge.

The deployed stack has been tested successfully with both:

- an Internet fixed-base NTRIP mountpoint;
- an Emlid Reach RS3 Local NTRIP caster on the same LAN without Internet.

The repository intentionally contains no separate IMU navigation path. Old
single-GPS, Witmotion IMU, and GPS+IMU odometry nodes were removed. Do not
reintroduce them unless the architecture is deliberately being expanded.

The current UM982 protocol/geometry test suite has 11 passing tests.

## 2. Runtime data flow

```text
Internet NTRIP or Reach RS3 Local NTRIP
                  |
                  | RTCM3
                  v
            um982_driver
                  |
                  | full-duplex USB serial
                  v
                UM982
                  |
                  | GGA + UNIHEADINGA
                  v
     /gps/fix + /robot/odom + heading/pitch
                  |
                  v
          waypoint_follower
                  |
             /cmd_vel_nav
                  |
                  v
              twist_mux
                  |
             /cmd_vel_out
                  |
                  v
        amiga_serial_bridge
                  |
                  v
         Feather M4 CAN -> Amiga
```

The safety path is:

```text
um982_driver -> /gps/rtk_status_flag -> rtk_monitor
                                         |
                                    /cmd_vel_stop
                                         |
                              highest twist_mux priority
```

Important semantic detail: `/gps/rtk_status_flag` is historically inverted.
`False` means the complete navigation solution is valid; `True` means stop
because RTK, heading, synchronization, NTRIP connection, or correction data is
invalid/stale. `rtk_monitor.py` performs the inversion explicitly.

## 3. Main source files

### GNSS and localization

- `amiga_navigation/um982_driver.py`
  - is the only process allowed to own the UM982 serial port;
  - configures the receiver as a rover;
  - reads GGA and UNIHEADINGA;
  - connects to NTRIP and writes raw RTCM3 to the same full-duplex serial port;
  - validates RTK fix, differential age, heading quality, baseline, satellite
    count, and message synchronization;
  - publishes midpoint `NavSatFix`, robot-forward heading, pitch, odometry, and
    the safety flag.
- `amiga_navigation/um982_protocol.py`
  - contains pure parsing and geometry helpers;
  - should remain independent of ROS so it can be unit tested.
- `amiga_navigation/datum_publisher.py`
  - latches the first published valid `/gps/fix` as `/gps/datum`;
  - the UM982 driver uses it to create the local transverse-Mercator frame;
  - the waypoint follower uses the same datum.

### Navigation and control

- `amiga_navigation/waypoint_follower.py`
  - consumes `/robot/odom` and `/gps/datum`;
  - publishes `/cmd_vel_nav`;
  - supports PID line tracking, pure pursuit, rollout MPC, formal MPC, and a
    row-hybrid controller.
- `amiga_navigation/utils/`
  - contains the controller and shared tracking geometry implementations.
- `amiga_navigation/rtk_monitor.py`
  - publishes a zero command on `/cmd_vel_stop` whenever the GNSS solution is
    invalid or its status becomes stale.
- `amiga_navigation/amiga_serial_bridge.py`
  - forwards `/cmd_vel_out` to the Feather M4;
  - has its own command watchdog and sends a final stop during shutdown.
- `FeatherM4_MCU/code.py`
  - is deployed separately to the Feather M4 CAN board.

### Utilities

- `gnss_waypoint_keyboard_logger.py`: interactively records RTK waypoints.
- `nav_topic_debug_logger.py`: records the current dual-GPS navigation topics.
- `robot_odom_logger.py`: records `/robot/odom`.

## 4. Launch structure

`basic_bringup.launch.py` starts:

1. `twist_mux`;
2. `datum_publisher`;
3. `um982_driver`;
4. `rtk_monitor`;
5. `amiga_serial_bridge`.

It deliberately does not start `waypoint_follower`, because the waypoint file
and controller are normally selected when starting that node separately.

`nav_debug.launch.py` includes the basic stack and adds
`nav_topic_debug_logger`.

Never start two `um982_driver` processes, and never run another NTRIP-to-serial
program such as `str2str` against the same UM982 port at the same time.

## 5. Coordinate and heading conventions

### GNSS antennas

- ANT1 is the master position antenna; GGA represents ANT1.
- ANT2 is the heading/slave antenna.
- UM982 `UNIHEADINGA.heading` is treated as true heading from ANT1 toward
  ANT2, clockwise from north.
- Published `/gps/fix` is the 3-D midpoint of ANT1 and ANT2.
- Published `/robot/odom` orientation is the robot's forward direction, not
  necessarily the ANT1-to-ANT2 direction.

### Antenna layout parameter

`antenna_baseline_angle_deg` is the direction from ANT1 to ANT2 relative to
robot forward, viewed from above. Positive angles are clockwise toward the
robot's right.

```text
ANT1 rear,  ANT2 front:  antenna_baseline_angle_deg =   0
ANT1 front, ANT2 rear:   antenna_baseline_angle_deg = 180
ANT2 right of ANT1:      antenna_baseline_angle_deg =  90
ANT2 left of ANT1:       antenna_baseline_angle_deg = -90
```

The conversion is:

```text
robot true heading =
    UM982 ANT1-to-ANT2 heading
    - antenna_baseline_angle_deg
    + heading_offset_deg
```

`heading_offset_deg` is only a small optional calibration after layout
compensation. It is not the primary hardware-layout parameter.

Do not use robot heading to calculate the antenna midpoint. For side-mounted
or reversed antennas, midpoint displacement must use the raw ANT1-to-ANT2
heading. This separation is implemented intentionally in `_process_fix()`.

True heading is converted to ROS ENU yaw as:

```text
ENU yaw = 90 degrees - true heading
```

The odometry frames are:

```text
header.frame_id = map
child_frame_id  = base_link
```

## 6. Valid-solution requirements

The driver publishes a navigation pose only if all of these pass:

- GGA quality is `4` (RTK Fixed);
- differential age exists and is within `max_differential_age_sec`;
- UNIHEADINGA solution status is `SOL_COMPUTED`;
- heading position type is `NARROW_INT`;
- reported baseline matches `baseline_m` within `baseline_tolerance_m`;
- heading standard deviation is below its configured maximum;
- sufficient heading satellites are used;
- GGA and heading receive times are sufficiently synchronized.

The safety flag additionally requires:

- a recent complete valid solution;
- an active NTRIP connection when NTRIP is enabled;
- recent NTRIP data within `ntrip_data_timeout_sec`.

Expected GGA quality progression is generally:

```text
1 = standalone
5 = RTK float
4 = RTK fixed
```

The navigation stack intentionally accepts only quality `4`.

## 7. NTRIP implementation and compatibility traps

The NTRIP worker supports:

- HTTP/1.1 / NTRIP v2 responses;
- HTTP/1.0 / legacy NTRIP v1;
- `HTTP 200` responses;
- `ICY 200 OK` responses;
- legacy casters that begin streaming RTCM3 directly, possibly after extra
  non-HTTP prefix bytes;
- automatic switching between HTTP/1.1 and HTTP/1.0 after connection or
  protocol errors;
- plain TCP or TLS selected explicitly by configuration.

Reach RS3 Local NTRIP exposed an important parser issue during deployment:

- network connectivity and authentication were valid;
- `curl --http0.9` received a continuous stream containing RTCM3 `0xD3`
  preambles;
- the old driver waited for a conventional HTTP header and eventually raised
  `NTRIP response header is too large`;
- the fix searches received prefix data for the first RTCM3 `0xD3` and treats
  it as the beginning of a valid headerless correction stream;
- a genuine HTTP/1.0 request does not send `Ntrip-Version: Ntrip/2.0`.

Do not simplify `_read_response_header()` back to conventional HTTP-only
parsing. That will break Reach Local NTRIP.

TLS selection must remain explicit. Do not probe TLS by first sending
credentials over plaintext. Use:

```yaml
ntrip_use_tls: false  # legacy/plain TCP
ntrip_use_tls: true   # caster explicitly requires TLS
```

The current client does not send rover GGA back to the caster. It is verified
with fixed-base mountpoints. VRS, NEAR, MAC, or other network-RTK services that
require periodic rover GGA need a deliberate new implementation.

## 8. Configuration

Primary configuration:

```text
src/amiga_navigation/config/bringup.yaml
```

Important groups:

- serial port and baud rate;
- physical baseline length and tolerance;
- antenna layout and heading/pitch conventions;
- quality and age limits;
- receiver startup configuration;
- NTRIP endpoint, credentials, TLS, and timeouts.

Normal hardware-layout changes should require only:

```yaml
baseline_m: ...
baseline_tolerance_m: ...
antenna_baseline_angle_deg: ...
heading_offset_deg: 0.0
pitch_multiplier: 1.0
```

For switching between Internet and Local NTRIP, prefer two private YAML files
with identical non-NTRIP settings. Select one with:

```bash
ros2 run amiga_navigation um982_driver --ros-args \
  --params-file /absolute/path/to/selected_bringup.yaml
```

The checked-in YAML currently contains deployment credentials because it is
also used locally. Before publishing or sharing the repository, replace all
real hosts, usernames, and passwords—including commented credentials—with
placeholders and rotate any exposed secrets.

## 9. Build and run

From the repository root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Use `--symlink-install` during development so edits to Python and configuration
are reflected predictably.

Start the base stack:

```bash
ros2 launch amiga_navigation basic_bringup.launch.py \
  bringup_config:="$PWD/src/amiga_navigation/config/bringup.yaml"
```

Or test the UM982 driver alone:

```bash
ros2 run amiga_navigation um982_driver --ros-args \
  --params-file "$PWD/src/amiga_navigation/config/bringup.yaml"
```

If a normal non-symlink build was previously used, an installed YAML copy may
be stale. Either rebuild or pass the source YAML explicitly. To verify the
active parameters:

```bash
ros2 param get /um982_driver ntrip_host
ros2 param get /um982_driver ntrip_port
ros2 param get /um982_driver ntrip_mountpoint
ros2 param get /um982_driver ntrip_use_tls
```

## 10. Runtime verification

Useful checks:

```bash
ros2 topic echo /gps/fix
ros2 topic echo /gps/rtk_status_flag
ros2 topic echo /um982/heading_deg
ros2 topic echo /um982/pitch_deg
ros2 topic echo /robot/odom
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_out
```

A healthy startup should include an NTRIP connection log and should not
continuously report rejected UM982 solutions. The output heading is already
robot-forward heading after layout compensation.

For Reach Local NTRIP diagnostics:

```bash
ping -c 4 REACH_IP
nc -vz -w 3 REACH_IP NTRIP_PORT
curl --http1.0 --http0.9 -v --max-time 10 \
  -u USERNAME http://REACH_IP:NTRIP_PORT/MOUNTPOINT \
  --output /tmp/reach.rtcm
xxd -l 128 /tmp/reach.rtcm
```

A curl timeout after receiving bytes is normal for a continuous stream.
Successful legacy output commonly begins with `ICY 200 OK` followed by binary
RTCM3 frames containing repeated `d3` preambles.

## 11. Tests

Run:

```bash
colcon build --symlink-install --packages-select amiga_navigation
colcon test --packages-select amiga_navigation
colcon test-result --verbose
```

The pure tests cover:

- RTK Fixed GGA parsing;
- UNIHEADINGA parsing and CRC rejection;
- 3-D antenna midpoint geometry;
- true-heading to ENU-yaw conversion;
- forward, reversed, left-side, and right-side antenna layouts;
- post-layout heading calibration.

After validation, `build/`, `install/`, `log/`, `__pycache__/`, and pytest
caches are generated artifacts and must not be committed. `.gitignore`
already excludes them.

## 12. Known limitations and recommended extensions

Known limitations:

- no rover-GGA return channel for VRS/NEAR services;
- no RTCM frame CRC/type counters in ROS diagnostics;
- NTRIP credentials are plain ROS parameters;
- fixed hard-coded frame names (`map`, `base_link`);
- datum is the first valid published fix and is not persisted between runs;
- no separate IMU fusion for motion during GNSS heading outages;
- the driver publishes fixed covariance values for position.

Good future improvements:

1. Add optional GGA return with configurable interval.
2. Add RTCM received bytes, valid-frame counts, message-type counts, and serial
   write counters.
3. Publish standard `diagnostic_msgs/DiagnosticArray`.
4. Make frame IDs and covariance configurable.
5. Support a configured/persisted datum.
6. Move secrets to a private untracked configuration or environment-backed
   mechanism.
7. Add pseudo-terminal and simulated-caster integration tests.

Any IMU fusion should be introduced as a separate, explicit localization
layer. Do not replace the reliable UM982 dual-antenna heading path with the old
Witmotion implementation.

## 13. Safety and change checklist

Before field testing a change:

1. Confirm only one process owns the UM982 serial port.
2. Confirm antenna cables and ANT1/ANT2 roles.
3. Measure the physical baseline and set its tolerance.
4. Set `antenna_baseline_angle_deg` for the actual layout.
5. Confirm `/um982/heading_deg` follows robot forward, not merely ANT1→ANT2.
6. Confirm `/gps/fix` is the antenna midpoint.
7. Confirm NTRIP data stays active and GGA reaches quality `4`.
8. Confirm loss of NTRIP or RTK causes `/cmd_vel_stop` to override navigation.
9. Confirm the Feather watchdog stops the robot if commands cease.
10. Start with low speed and maintain a physical emergency-stop path.

When changing geometry, heading conventions, safety status, or twist_mux
priority, update this handoff, the README, YAML comments, and tests together.
