# PPBv2 Imaging

Last updated by [Yiyuan Lin](yl3663@cornell.edu) on Aug 10, 2026



ROS 2 Humble package for synchronized multi-camera triggering, RGB/raw image recording, GNSS logging, and external strobe control. The deployment target is an NVIDIA Jetson AGX Orin running Ubuntu 22.04.

<br>

## Hardware

- [NVIDIA Jetson AGX Orin 64 GB](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
- [Arduino UNO R4 WiFi](https://store.arduino.cc/products/uno-r4-wifi)
- PhenoStereo illumination system
- [FLIR Blackfly S BFS-U3-123S6C-C](https://www.teledynevisionsolutions.com/en-gb/products/blackfly-s-usb3/?model=BFS-U3-123S6C-C&vertical=machine+vision&segment=iis) cameras **x2**
- [UM982 dual-antenna GNSS receiver](https://www.amazon.com/dp/B0FCFZXDDJ?lv=shuf&rsd=D5eDWMLU%2BlZo6H0ytWeBpEDHKXB%2FOcsDmcQZWwHV38CM3ISNZMfZLK1%2FrAAe8%2Fnf8sC3sOUDDCqGa9AzsXbzcDa01EuH3%2FG1VnOwXIxYSYYF&edk=AQIDAHi1lw%2FM8UbbSMD9ScOOFEmBMHMthHeEhqDaQYPJUAX3jQFkG1KajJRK0UpRjdW%2FOK8dAAAAfjB8BgkqhkiG9w0BBwagbzBtAgEAMGgGCSqGSIb3DQEHATAeBglghkgBZQMEAS4wEQQMNue5pNQTD4syVtUoAgEQgDu7Fq4A7aVxOrC%2BURE4feV3vhHwf5frlgX6dqASVvksRvEaQAYA46izYui3WygErr4Sb%2BhMZgkKwi79wQ%3D%3D&social_share=cm_sw_r_apin_dp_TP36CY53FWREGK9BPECT&channelId=704&ref_=cm_sw_r_apin_dp_TP36CY53FWREGK9BPECT&plpRedirect=mhFallback&th=1)
- [GNSS Multi-Band L1/L2/L5 Surveying Antenna - TNC (SPK6618H)](https://www.sparkfun.com/gnss-multi-band-l1-l2-l5-surveying-antenna-tnc-spk6618h.html?gad_source=1&gad_campaignid=21251727806&gbraid=0AAAAADsj4ESEiGaW3DcX3fRbrfV9ID-rR) **x2**

<br>

## Dependencies

1. Install ROS 2 Humble.
2. Install FLIR Spinnaker SDK and PySpin `4.2.0.88` for Python 3.10.
3. Install the system Python dependencies:

   ```bash
   sudo apt install python3-pil python3-pyproj python3-serial
   ```

4. The optional offline `convert_pgm2png.py` tool additionally requires:

   ```bash
   pip install pillow numpy opencv-python tqdm
   ```

<br>

## Build and run

0. Upload the current `Arduino/Strobe_Light_Serial_Trigger_Init.ino` sketch to Arduino before running. The sketch contains a safety watchdog: after acquisition starts, it stops the strobe and camera trigger if the Jetson heartbeat is absent for two seconds.

1. Build the workspace:

   ```bash
   cd PPBv2_Imaging
   colcon build --symlink-install
   source install/setup.bash
   ```

2. Edit the `PPBv2_Imaging.bash` file to configure your serial port, white balance parameters and GNSS receiver setup.

3. Launch the data acquisition by running:

   ```bash
   x bash PPBv2_Imaging.bash
   ```

   The script prompts for a new output folder and an image format. It refuses to reuse an existing output folder. Press `Ctrl+C` to stop all nodes.

   The data root defaults to `/media/Data/cairlab`. It can be changed without editing the script:

   ```bash
   PPBV2_DATA_ROOT=/path/to/data bash PPBv2_Imaging.bash
   ```

   The format can also be supplied non-interactively:

   ```bash
   IMAGE_FORMAT=jpg bash PPBv2_Imaging.bash
   ```

<br>

## Image formats

| Value | Pixel data | Compression | Intended use |
| --- | --- | --- | --- |
| `png` | RGB | Lossless | Default; analysis-ready images |
| `jpg` or `jpeg` | RGB | Lossy | Smaller files and faster transfer |
| `pgm` | BayerRG8 (RGGB) | Uncompressed | Original raw sensor mosaic |

For PNG and JPEG, PySpin performs BayerRG8-to-RGB conversion before the image is written. Frames from different cameras are compressed in parallel, and image files are committed with an atomic rename, reducing the chance that an interrupted run leaves a partial file with a normal filename.

<br>

## Main parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `output_dir` | string | `/tmp` | Root directory for images and CSV files |
| `image_format` | string | `png` | `png`, `jpg`/`jpeg`, or `pgm` |
| `jpeg_quality` | int | `95` | JPEG quality, 1–100 |
| `png_compress_level` | int | `3` | PNG compression, 0–9 |
| `overwrite_existing` | bool | `false` | Explicitly allow existing camera output |
| `arduino_port` | string | `/dev/ttyACM0` | Arduino serial port |
| `arduino_baud` | int | `9600` | Arduino baud rate |
| `arduino_startup_delay_sec` | float | `2.0` | Wait for Arduino reset after opening serial |
| `arduino_heartbeat_period_sec` | float | `0.5` | Safety heartbeat interval |
| `exposure_time` | float | `400.0` | Exposure time in microseconds |
| `gain` | float | `5.0` | Manual gain in dB |
| `wb_red` | float | `1.34` | Manual red white-balance ratio |
| `wb_blue` | float | `2.98` | Manual blue white-balance ratio |
| `gps_match_max_age_sec` | float | `0.25` | Maximum frame-to-GPS ROS-time delta |
| `camera_timeout_ms` | int | `1000` | Per-camera frame timeout |
| `resync_max_attempts` | int | `3` | Maximum cross-camera resync passes |
| `resync_max_drop_frames` | int | `10` | Maximum frames discarded in one resync |
| `resync_timeout_sec` | float | `4.0` | Resync wall-clock time limit |
| `baseline_m` | float | `1.28` | Measured UM982 ANT1-to-ANT2 distance |
| `baseline_tolerance_m` | float | `0.1` | Allowed reported-baseline error |
| `antenna_baseline_angle_deg` | float | `0.0` | ANT1-to-ANT2 angle relative to robot forward |
| `heading_offset_deg` | float | `0.0` | Fine robot-heading calibration |
| `receiver_output_period_sec` | float | `0.1` | UM982 GGA/UNIHEADINGA output period |
| `max_heading_stddev_deg` | float | `1.0` | Maximum valid heading uncertainty |
| `minimum_heading_satellites` | int | `6` | Minimum satellites used for heading |

Camera parameters are clamped to the ranges reported by each camera. Required trigger/chunk settings are checked before acquisition starts.

<br>

## UM982 dual-antenna heading

The imaging GPS publisher configures `GPGGA` and `UNIHEADINGA` on its selected UM982 serial port. It can run alongside Navigation when the two processes use different physical UM982 ports. Configure distinct stable `/dev/serial/by-id/` paths and keep their baseline/layout parameters consistent.

The Orin Imaging code does **NOT** connect to an NTRIP caster or send RTCM correction data to the UM982. RTK corrections are provided by `um982_driver` in `PPBv2_Navigation` running on the Raspberry Pi. That driver receives RTCM from the base station through NTRIP and writes it to one serial port of the same UM982. The UM982 performs the RTK solution internally, so the Orin can read the corrected GGA and heading solution from another UM982 serial port. Merely running the Imaging code does not guarantee RTK Fixed; the Raspberry Pi Navigation driver and its NTRIP connection must also be running. In the CSV files, use `Fix Quality = 4` and `Dual Solution Valid = 1` to identify valid RTK Fixed position-and-heading samples.

Select the dedicated Imaging port without editing the script:

```bash
PPBV2_UM982_IMAGING_PORT=/dev/serial/by-id/<imaging-port> \
  bash PPBv2_Imaging.bash
```

UM982 GGA is the ANT1 position. The imaging node uses the raw ANT1-to-ANT2 heading, corrected pitch, configured baseline, and WGS84 geodesics to calculate the three-dimensional antenna midpoint. `/imaging/gps/fix` remains ANT1 for backward compatibility. `gps_log.csv` stores both ANT1 and midpoint positions; each camera's `Timestamp_GPS.csv` stores the midpoint position.

UM982 reports a true heading from ANT1 toward ANT2, clockwise from north. The CSV records this as `Baseline Heading(deg)`. It also records robot-forward true heading using the same convention as `PPBv2_Navigation`:

```text
Robot Heading = Baseline Heading - antenna_baseline_angle_deg
                + heading_offset_deg
```

Set `antenna_baseline_angle_deg` for the physical installation: `0` when ANT2 is in front of ANT1, `180` when behind, `90` when on the robot's right, and `-90` when on the left. All headings are normalized to `[0, 360)` degrees.

`Heading Valid` checks message age, `SOL_COMPUTED/NARROW_INT`, baseline length, heading standard deviation, and satellite count. `Dual Solution Valid` also requires RTK Fixed GGA quality, acceptable differential age, and sufficiently synchronized GGA/heading receive times. Raw angle fields remain in the CSV even when a validity flag is zero, so invalid solutions can be diagnosed but should not be used as ground truth.

<br>

## Output layout

```text
output_dir/
├── <camera_1_serial>/
│   ├── 000001.png
│   ├── 000002.png
│   └── Timestamp_GPS.csv
├── <camera_2_serial>/
│   ├── 000001.png
│   ├── 000002.png
│   └── Timestamp_GPS.csv
└── gps_log.csv
```

The extension follows `image_format`. Each per-camera CSV contains:

```text
Frame ID, Image File, Computer Time, ROS Time Stamp(s.ns), Chunk Frame ID,
Chunk Time(ns), Satellite UTC, GPS ROS Time(s.ns), GPS Match Delta(s),
Midpoint Latitude, Midpoint Longitude, Midpoint Altitude, Midpoint Computed,
FixQuality, Differential Age(s),
Baseline Heading(deg), Robot Heading(deg), Pitch(deg), Heading StdDev(deg),
Pitch StdDev(deg), Heading Baseline(m), Heading Satellites Tracked,
Heading Satellites Used, Heading Solution Status, Heading Position Type,
Heading ROS Time(s.ns), GGA-Heading Delta(s), Frame-Heading Delta(s),
Heading Valid, Dual Solution Valid, Dual Solution Reason, CrossCameraSyncOk
```

`CrossCameraSyncOk` is `1` when all saved frames in the batch had the same camera Chunk Frame ID. GPS fields are blank when no sample falls within `gps_match_max_age_sec`.

The root `gps_log.csv` records the same UM982 diagnostics but includes both `ANT1 Latitude/Longitude/Altitude` and `Midpoint Latitude/Longitude/Altitude`. `Midpoint Computed` distinguishes a computed coordinate from a blank row; use `Dual Solution Valid=1` when selecting high-confidence RTK position-and-heading samples.

The GPS reader validates GGA checksums and UNIHEADINGA CRC values and reconnects after serial failures. Valid robot heading and pitch are also published on `/imaging/gps/heading_deg` and `/imaging/gps/pitch_deg`.

<br>

## Converting PGM data

Raw BayerRG8 PGM folders can still be converted offline:

```bash
python convert_pgm2png.py <path/to/pgm/folder> <path/to/png/folder>
```

<br>

## Maintenance

For questions, contact Yiyuan Lin at yl3663@cornell.edu.
