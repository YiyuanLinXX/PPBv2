# PPBv2 Imaging

Last updated by [Yiyuan Lin](yl3663@cornell.edu) on July 10, 2025

---



This is the ROS 2 package for synchronized multi-camera image triggering, GPS data logging, and external strobe light control for high-throughput  phenotyping. The codebase is deployed on NVIDIA Jetson AGX Orin with ROS2 Humble. 



## Hardware

1. NVIDIA Jetson AGX Orin 64GB
2. Arduino UNO M4 WiFi
3. PhenoStereo illumination system (FieldRobo, LLC, Ames, IA)
4. FLIR Blackfly S BFS-U3-123S6C-C RGB cameras (stereo setup)



## Dependencies

1. Install ROS 2 Humble following the official [instruction](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

2. Install FLIR Spinnaker `4.2.0.88` SDK for UBuntu 22.04 and the same version of Spinnaker SDK for Python 3.10 (PySpin) following the FLIR official [instruction](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis).

3. Install necessary Python packages by running

   ```bash
   pip install pyserial pillow numpy opencv-python tqdm argparse pandas matplotlib
   ```



## Getting Start

1. Upload `Arduino/Strobe_Light_Serial_Trigger_Init.ino` using Arduino IDE to an Arduino board. The Arduino should be connected to Orin via USB and will receive `'s'` (start) and `'e'` (end) trigger signal over serial. **Please refer to the comment in the `.ino` file to check the pinouts and make sure you connect the wires for hardware correctly.**



2. Clone the codebase and build the ROS2 package on Orin.

   ```bash
   git clone https://github.com/YiyuanLinXX/PPBv2.git
   cd PPBv2_Imaging
   colcon build
   source install/setup.bash
   ```



3. Edit the configuration (e.g. GPS port, Arduino port, exposure, etc.) in the bash file and run it.

   ```bash
   bash PPBv2_Imaging.bash
   ```

   To stop data acquisition at any time (e.g. when complete or in an emergency), press `CTRL+C` in the terminal that is running the DAQ command.

| Parameter       | Type   | Default        | Description                     |
| --------------- | ------ | -------------- | ------------------------------- |
| `output_dir`    | string | `/tmp`         | Where images and CSVs are saved |
| `arduino_port`  | string | `/dev/ttyACM0` | Serial port for Arduino         |
| `arduino_baud`  | int    | `9600`         | Baudrate for Arduino            |
| `exposure_time` | float  | `400.0`        | Camera exposure in µs           |
| `gain`          | float  | `5.0`          | Manual gain (dB)                |
| `wb_red`        | float  | `1.34`         | White balance red ratio         |
| `wb_blue`       | float  | `2.98`         | White balance blue ratio        |
| `gps_port`      | string | `/dev/ttyUSB0` | Serial port for GPS receiver    |
| `gps_baud`      | int    | `115200`       | Baudrate for GPS receiver       |



## Output Format

- Each camera will produce a folder:

```bash
output_dir/
└── <camera_1_serial>/
    ├── 000001.pgm
    ├── 000002.pgm
    └── Timestamp_GPS.csv  # Per-frame log
└── <camera_1_serial>/
    ├── 000001.pgm
    ├── 000002.pgm
    └── Timestamp_GPS.csv
└── gps_log.csv
```

- Timestamp_GPS.csv columns

  ```mathematica
  Frame ID, Computer Time, ROS Timestamp, Chunk Frame ID, Chunk Time, Latitude, Longitude, Fix Quality
  ```

  - Frame ID: ID of the **saved** images

  - Computer time: less accurate time stamp

  - ROS time stamp: less accurate time stamp

  - Chunk frame ID : ID of the **captured** images (may not be saved)

  - Chunk time: very accurate time stamp, in nanosecond (ns) level

  - Latitude: Synchronized GPS coordinate

  - Longitude: Synchronized GPS coordinate
  - FIX Quality: RTK status from NMEA message



## Post Process

1. The images are collected in PGM (portable gray map) with Bayer RG pattern. To convert them into RGB images. Run

   ```bash
   python convert_pgm2png.py <path/to/pgm/folder> <path/to/png/folder>
   ```

   Note: we always transfer the data from the Orin to a local machine (the other device for analysis) and convert the images in the local machine.



## Maintenance

For any questions or uncertainty, please contact Yiyuan Lin (yl3663@cornell.edu).



## Supplementary

### Data Acquisition Example

```bash
cairlab@ubuntu:~$ bash PPBv2_Imaging.bash 
Enter output folder name in the format Vineyard_YYYYMMDD_HHMM(e.g., Oblock_20250617_0914): Demo_20250630_1917 # Enter your desired folder name for data collection here



Save Directory: /media/Data/cairlab/Demo_20250630_1917
Launching gps_publisher...
Launching gps_logger...
Launching multi_camera_trigger_node...
[INFO] [1751325479.646477906] [gps_publisher]: Opened GPS serial on /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B001F6BO-if00-port0 @ 115200
[INFO] [1751325481.795106393] [multi_camera_trigger]: Set Device Link Throughput Limit to 100000000 Bytes/sec
[INFO] [1751325481.799805944] [multi_camera_trigger]: ExposureAuto set to Off
[INFO] [1751325481.802941506] [multi_camera_trigger]: ExposureMode set to Timed
[INFO] [1751325481.806409255] [multi_camera_trigger]: Set ExposureTime to 253.0 µs (max 30000002.0 µs)
[INFO] [1751325481.813976929] [multi_camera_trigger]: Set Gain to 4.999405628440247 dB
[INFO] [1751325481.827597768] [multi_camera_trigger]: Set WB ratios: Red=1.34, Blue=2.98
[INFO] [1751325481.851256990] [multi_camera_trigger]: TriggerActivation set to FallingEdge
[INFO] [1751325482.300246983] [multi_camera_trigger]: Set Device Link Throughput Limit to 100000000 Bytes/sec
[INFO] [1751325482.304997759] [multi_camera_trigger]: ExposureAuto set to Off
[INFO] [1751325482.308444117] [multi_camera_trigger]: ExposureMode set to Timed
[INFO] [1751325482.312052776] [multi_camera_trigger]: Set ExposureTime to 253.0 µs (max 30000002.0 µs)
[INFO] [1751325482.319636716] [multi_camera_trigger]: Set Gain to 4.999405628440247 dB
[INFO] [1751325482.333646603] [multi_camera_trigger]: Set WB ratios: Red=1.34, Blue=2.98
[INFO] [1751325482.357497846] [multi_camera_trigger]: TriggerActivation set to FallingEdge
[INFO] [1751325482.374574345] [multi_camera_trigger]: Starting acquisition loop...
[INFO] [1751325482.377842848] [multi_camera_trigger]: Sent 's' to Arduino on /dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_F412FA67A978-if01
[INFO] [1751325482.474958709] [multi_camera_trigger]: camera 24023187 frame 000001 saved in 0.030s
[INFO] [1751325482.505394038] [multi_camera_trigger]: camera 24071779 frame 000001 saved in 0.028s
[INFO] [1751325482.972552544] [multi_camera_trigger]: camera 24023187 frame 000002 saved in 0.022s
[INFO] [1751325482.996355746] [multi_camera_trigger]: camera 24071779 frame 000002 saved in 0.022s
[INFO] [1751325483.474593913] [multi_camera_trigger]: camera 24023187 frame 000003 saved in 0.019s
[INFO] [1751325483.496084039] [multi_camera_trigger]: camera 24071779 frame 000003 saved in 0.019s
[INFO] [1751325483.979531064] [multi_camera_trigger]: camera 24023187 frame 000004 saved in 0.019s
...
```



