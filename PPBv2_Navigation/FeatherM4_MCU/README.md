Please follow the Farm-ng Amiga documentation about using the micro-controller and uploading script to it.

https://amiga.farm-ng.com/docs/mcu_kit/



Upload the `code.py` to your Feather M4 CAN micro controller

The script expects newline-terminated serial velocity commands from the ROS-side bridge:

```text
linear_velocity,angular_velocity
```

Example:

```text
0.500000,0.100000
```

The Feather sends `AmigaRpdo1` CAN commands at 20 Hz. If no valid serial command is received for `0.75` seconds, the Feather-side watchdog sets speed and angular rate to zero. This is separate from the ROS-side watchdog in `amiga_serial_bridge.py`.

The Feather writes back compact status feedback on the USB serial console:

```text
amiga_state,measured_speed,measured_angular_rate
```
