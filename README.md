# PPBv2

Last updated by [Yiyuan Lin](mailto:yl3663@cornell.edu) on July 26, 2026

---



This repository includes the codebase for our phenotyping robot PhytoPatholoBot version 2 (PPBv2). PPBv2 is a mobile robotic platform designed for high-throughput phenotyping in field environments and optimized for tasks such as disease phenotyping, supporting precise spatiotemporal mapping of phenotypic traits. Equipped with an active illumination system and stereo imaging, PPBv2 enables robust and consistent visual data acquisition under natural lighting variability. The robot is powered by a Raspberry Pi 5 for navigation control and integrates with external triggers and logging systems for synchronized image capture and georeferencing.

<img src="assets/ppbv2_in_field.gif" width="100%" />

## PPBv2 Modular Design
<img src="assets/PPBv2_System_Modular_Design_20260726_horizontal.png" width="100%" />



## PPBv2 Navigation

This repository provides two navigation configurations for GNSS waypoint-based navigation on the Farm-ng Amiga robot:

- **`main` (recommended):** Dual GPS navigation using a dual-antenna UM982 GNSS receiver. It provides RTK position and true heading without requiring a separate IMU.
- **`single-gps-imu`:** The legacy navigation implementation based on a single GPS receiver and an IMU.

The navigation stack is deployed on a Raspberry Pi 5 running Ubuntu 24.04 and ROS 2 Jazzy. For Dual GPS setup and implementation details, see the
[PPBv2 Navigation README](PPBv2_Navigation/README.md).

To use the legacy Single GPS + IMU implementation, switch branches:

```bash
git switch single-gps-imu
```



## PPBv2 Imaging

This is the ROS 2 package for synchronized multi-camera image triggering, GPS data logging, and external strobe light control for high-throughput  phenotyping. The codebase is deployed on NVIDIA Jetson AGX Orin with ROS2 Humble.

For implementation details, please refer to: [PPBv2 Imaging README](PPBv2_Imaging/README.md).



## Maintenance

For any questions or uncertainty, please contact Yiyuan Lin ([yl3663@cornell.edu](mailto:yl3663@cornell.edu)).
