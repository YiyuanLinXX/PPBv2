# PPBv2

Last updated by [Yiyuan Lin](yl3663@cornell.edu) on June 30, 2025

---



This repository includes the codebase for our phenotyping robot PPBv2.



## Dependencies

1. Install ROS 2 Humble following the official [instruction](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

2. Install FLIR Spinnaker 4.2.0.88 SDK for UBuntu 22.04 and Spinnaker SDK for Python 3.10 (PySpin) following the FLIR official [instruction](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis)

3. Install necessary Python packages by running

   ```bash
   pip install pyserial
   ```



## PPBv2 Imaging

This is the ROS 2 package for synchronized multi-camera image triggering, GPS data logging, and external strobe light control for high-throughput  phenotyping. The codebase is deployed on NVIDIA Jetson AGX Orin with ROS2 Humble. Please refer to [PPBv2 Imaging README](PPBv2_Imaging/README.md) for detailed information of implementation.

