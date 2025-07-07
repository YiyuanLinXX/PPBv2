# PPBv2

Last updated by [Yiyuan Lin](yl3663@cornell.edu) on July 07, 2025

---



This repository includes the codebase for our phenotyping robot PhytoPatholoBot version 2 (PPBv2). PPBv2 is a mobile robotic platform designed for high-throughput phenotyping in field environments and optimized for tasks such as disease phenotyping, supporting precise spatiotemporal mapping of phenotypic traits. Equipped with an active illumination system and stereo imaging, PPBv2 enables robust and consistent visual data acquisition under natural lighting variability. The robot is powered by a Raspberry Pi 5 for navigation control and integrates with external triggers and logging systems for synchronized image capture and georeferencing.



## PPBv2 Navigation

This is the ROS 2 package for GPS waypoints based robot navigation on Farm-ng Amiga robot. The codebase is deployed on Raspberry Pi 5 with ROS2 Jazzy. Please refer to [PPBv2 Navigation README](PPBv2_Navigation/README.md) for detailed information of implementation.



## PPBv2 Imaging

This is the ROS 2 package for synchronized multi-camera image triggering, GPS data logging, and external strobe light control for high-throughput  phenotyping. The codebase is deployed on NVIDIA Jetson AGX Orin with ROS2 Humble. Please refer to [PPBv2 Imaging README](PPBv2_Imaging/README.md) for detailed information of implementation.



## Maintenance

For any questions or uncertainty, please contact Yiyuan Lin ([yl3663@cornell.edu](mailto:yl3663@cornell.edu)).
