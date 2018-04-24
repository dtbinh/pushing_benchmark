# APC Sensor

A standalone C++ ROS package for streaming and capturing data (RGB-D frames and 3D point cloud) from the Intel® RealSense™ F200 Camera in real-time (up to 30 FPS)

## Hardware Dependencies

* Intel® RealSense™ F200 Camera

## Software Dependencies (Instructions)
1. Install [librealsense](https://github.com/IntelRealSense/librealsense) (instructions can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md))
 * Note: install with the Video4Linux backend

## ROS Usage
* Clone ```apc_sensor``` into the catkin workspace source directory (e.g., ```catkin_ws/src```)
* In your catkin workspace, compile the package with `catkin_make` 
* Source `devel/setup.sh`
* Start `roscore`
* Run `rosrun apc_sensor capture` on the brix computer to stream data from the sensor and start the data capture service:
  * `/apc_sensor` returns data from the sensor (response data format described in `apc_vision/srv/StreamSensor.srv`)
  * if you need the GL window to see the data do `rosrun apc_sensor capture _useGL:=True`
 
