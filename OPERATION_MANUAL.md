# PPBv2 Public Operation Manual

Updated August 29, 2026

This guide explains how to operate PPBv2 after the hardware, operating systems, ROS 2 workspaces, serial ports, NTRIP profile, camera settings, and robot interface have been configured by a system maintainer. For installation and configuration, use the [Navigation README](PPBv2_Navigation/README.md) and [Imaging README](PPBv2_Imaging/README.md).

## Safety

> [!WARNING]
> PPBv2 is a moving robotic system. Only trained operators should run it. Keep the physical emergency stop accessible, maintain direct visual supervision, and keep people and obstacles outside the operating area. Software controls and keyboard commands do not replace the physical emergency stop.

Before every run:

1. Complete the robot manufacturer's pre-operation inspection.
2. Check power, antennas, cables, cameras, illumination, and the operating area.
3. Confirm that the RTK base station and NTRIP service are available.
4. Confirm that the selected waypoint route matches the current field and base-station reference.
5. Confirm sufficient data storage before imaging.
6. Begin at a conservative speed after any hardware, configuration, or route change.

## Choose an operating mode

| Goal | RPi Navigation Basic Bring-Up | Waypoint Follower | Orin Imaging |
| --- | :---: | :---: | :---: |
| Image and GNSS collection only | Required for RTK corrections | No | Yes |
| Autonomous navigation only | Yes | Yes | No |
| Autonomous navigation with imaging | Yes | Yes | Yes |

The Navigation Basic Bring-Up is required whenever RTK-quality position is needed. The Raspberry Pi receives RTCM correction data from the NTRIP caster and sends it to the UM982. The UM982 performs the RTK solution internally. The Orin reads the resulting position and heading from another receiver port; it does not establish its own NTRIP connection.

## Terminal conventions

The examples below assume that ROS 2 and each workspace are sourced automatically on their respective computer. Replace all angle-bracket placeholders with values from your deployment.

- `<rpi-user>@<rpi-host>`: Raspberry Pi login and hostname or address
- `<orin-user>@<orin-host>`: Jetson Orin login and hostname or address
- `<route.csv>`: absolute path to an approved waypoint file

Run each long-lived program in a named GNU `screen` session. Start one with `screen -S SESSION_NAME`; detach without stopping it by pressing `Ctrl + A`, releasing both keys, and then pressing `D`. After reconnecting with SSH, use `screen -ls` to list sessions and `screen -d -r SESSION_NAME` to reopen one. `Ctrl + C` stops the program in the active screen; after it stops, type `exit` to close that session.

> [!WARNING]
> Losing SSH or Wi-Fi does not stop a screen session or the robot. Use the physical emergency stop if motion is unsafe, then reconnect to the existing session. Do not start a second copy of a program merely because its terminal disappeared.

## Start the common RTK and robot services

Connect to the Raspberry Pi:

```bash
ssh -t <rpi-user>@<rpi-host>
```

Create its screen session:

```bash
screen -S nav_bringup
```

Start Navigation Basic Bring-Up inside that screen:

```bash
ros2 launch amiga_navigation basic_bringup.launch.py
```

Wait for an NTRIP connection and a healthy GPS status before navigation or RTK-quality collection. A brief non-FIX state during initialization can be normal; a continuing `RTK not FIX` or `NTRIP disconnected` message is not.

## Mode A: Image and GNSS collection only

Leave Navigation Basic Bring-Up running on the Raspberry Pi. Do not start the Waypoint Follower.

In another terminal, connect to the Orin:

```bash
ssh -t <orin-user>@<orin-host>
```

Create the acquisition screen:

```bash
screen -S daq
```

Start acquisition inside that screen:

```bash
cd /path/to/PPBv2/PPBv2_Imaging
bash PPBv2_Imaging.bash
```

Enter a unique dataset folder name when prompted. Wait for `All programs started successfully`, then confirm that frames are being saved without repeated GPS, camera, Arduino, storage, or writer errors.

To stop, press `Ctrl + C` in the Imaging terminal and wait for `All programs stopped.` Stop Navigation Basic Bring-Up only after acquisition and any robot motion have stopped.

## Mode B: Autonomous navigation only

After Navigation Basic Bring-Up reports healthy RTK, open a second terminal, connect to the Raspberry Pi, and create a follower screen:

```bash
ssh -t <rpi-user>@<rpi-host>
```

After SSH connects, create the screen:

```bash
screen -S waypoint_follower
```

Run the follower inside that screen:

```bash
ros2 run amiga_navigation waypoint_follower --waypoints <route.csv>
```

Confirm that the terminal reports the intended route before allowing motion. If an unfinished route is detected, resume it only after confirming that the robot position, direction, field layout, and base-station reference are appropriate. Otherwise start the requested route from the beginning.

Monitor the robot, the RTK terminal, and the follower terminal continuously. For normal shutdown, stop the Waypoint Follower first and Basic Bring-Up second.

## Mode C: Autonomous navigation with imaging

Use this startup order:

1. Start Navigation Basic Bring-Up on the Raspberry Pi.
2. Wait for healthy NTRIP and RTK status.
3. Start Imaging on the Orin and confirm that frames are saving.
4. Start the Waypoint Follower on the Raspberry Pi.
5. Monitor all terminals and the robot throughout the run.

Use this normal shutdown order:

1. Stop the Waypoint Follower and confirm the robot has stopped.
2. Stop Imaging and wait for `All programs stopped.`
3. Stop Navigation Basic Bring-Up.
4. Inspect and back up the dataset.

## Optional supervised keyboard control

For testing or deliberate operator takeover, connect in another terminal and create a separate Raspberry Pi screen:

```bash
ssh -t <rpi-user>@<rpi-host>
```

After SSH connects, create the screen:

```bash
screen -S keyboard_control
```

Then start keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_key
```

Keyboard commands have higher software priority than waypoint commands in the supplied configuration, but they work only while the terminal is active. Use the physical emergency stop for emergencies.

## Recognizing valid data

The Imaging dataset contains a root `gps_log.csv`, one directory per camera, images, and a `Timestamp_GPS.csv` file in each camera directory. For high-confidence RTK position and heading, select rows in `gps_log.csv` where:

```text
Fix Quality = 4
Dual Solution Valid = 1
```

The acquisition supervisor stops the complete imaging session if fresh GPS data is unavailable for the configured timeout or if a required process fails. Correct the cause and use a new dataset folder for the next run.

## Common situations

| Situation | Response |
| --- | --- |
| RTK does not become valid | Do not start motion or final data collection; check the base station, NTRIP path, network, antennas, and receiver |
| The robot does not move | Check robot AUTO state, emergency-stop state, RTK status, and terminal safety messages |
| A latched navigation safety stop appears | Keep the robot stopped and inspect the reported cause before restarting the follower |
| The selected dataset folder already exists | Restart Imaging and choose a new unique name |
| Imaging stops unexpectedly | Stop navigation, record the first error, correct the cause, and restart with a new dataset folder |
| The wrong route is selected | Stop the Waypoint Follower immediately and restart with the approved route |
| Behavior is unsafe or unexpected | Use the physical emergency stop first, then stop the software |
| A terminal or network connection is lost | The programs and robot may continue; use the emergency stop if needed, reconnect, run `screen -ls`, and reopen the existing session with `screen -d -r SESSION_NAME` |

## Further documentation

- [PPBv2 repository overview](README.md)
- [Navigation installation, configuration, and controller reference](PPBv2_Navigation/README.md)
- [Imaging installation, formats, parameters, and output reference](PPBv2_Imaging/README.md)
- [Navigation maintainer handoff](PPBv2_Navigation/HANDOFF.md)
