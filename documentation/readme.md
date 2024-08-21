# AUV Toolkit - Project Developed by Robert Fudge, 2024

# Dependencies
Dependencies can be installed via ./ACC.sh -i. For system configuration (not tested anymore but should mostly work), specify architecture. If self-configuring hardware, use no-arch to get github dependencies

Nvidia CUDA Toolkit v12.5 or v12.2

Docker version 27.1.1

NVIDIA Container Runtime Hook version 1.14.2

ZED SDK V1.14

ROS2 Humble Hawksbill

ZED ROS2 Wrapper

NMEA Navsat ROS2 Driver

LibMavConn

MAVROS ROS2 Driver

# To Start

## 1. Run ./ACC.sh -i [arch]
Note: In order to run -i. -b, or -s, you must specify which architecture the code is being built on

## 2. Run ./ACC.sh -b [arch]
This will create a docker container using the specified architetcure, while grabbing all dependencies and prerequisite software

## 3. Run ./ACC.sh -s [arch]
This starts the container and will generate a terminal instance for the user to interact with the system. Note that additional terminal insatances may be started after this point by using the -n flag.

## 4. Source ROS2 Tools
source /opt/ros/humble/setup.bash
source install/setup.bash

## 5. Build Modules - MAVROS package is fixed, removed tests because they wouldn't compile
colcon build --parallel-workers $(nproc) --symlink-install --event-handlers console_direct+ --base-paths src --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' ' --no-warn-unused-cli' \
  --packages-select nmea_navsat_driver zed_ros2 zed_interfaces zed_components zed_wrapper libmavconn mavros_msgs mavros && source install/setup.bash

## 5. Launch ROS2 Services
### Foxglove ROS2 Bridge - https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge/
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

### ZED2 ROS2 Wrapper - https://github.com/stereolabs/zed-ros2-wrapper
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

### NMEA Navsat Driver - https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py

## 6. Launch Nvidia ROS Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py num_cameras:=2 enable_localization_n_mapping:=true enable_imu_fusion:=true base_frame:=sub_centre imu_frame:=imu enable_slam_visualization:=true enable_observations_view:=true enable_landmarks_view:=true

# Changelog
The following versions have been deemed stable enough to consider signifigant milestones, and starting June 17th, 2024

## V6.0 - August 18th, 2024
Started looking into controlling boards power capabilities
/sys/devices/platform/17000000.gpu/devfreq/17000000.gpu - For GPU control
/sys/devices/system/cpu/cpu0/cpufreq - For CPU control

## V5.0 - July XXth, 2024
Created and deployed temporary fixes

## V4.0 - June 17th, 2024
Initial commit to github

Updated container to work with ISSAC ROS 3.1, integrated configuration of drivers from outside of the container

Configured ZED2i to provide high quality depth maps, enabled postional tracking

Created basic GUI in Foxglove Studio to visualize data

## V3.0 - June Xth, 2024
Updated script to work with ISSAC ROS 3.0

Updated container to work with new ISSAC ROS build system, resulted in major refactoring of both scripts and containers

Added ability to launch additional terminals with -n flag, don't need to specify architecture

## V2.0 - May XXth, 2024
Changed base container to NVIDIA ISSAC ROS 2.1

Created container for aarch64 build, integrated scripts into one cohesive ACC.sh

Added .status folder to track container controller status

## V1.0 - May Xth, 2024
Initial stable build, utilizing ROS2 Humble, ZED ROS2 Wrapper, and NMEA Navsat Driver


# TODO
Design ROS2 system to act as system moderator, make decisions, provide high level control of systems in a unified interface - Created shared volume for mounting

NOT NEEDED Get ARDUSUB Driver working and configured

Get gaskets for end cap

Source waterproof, pressure resistant silcone

# DONE
Integrate T200 Thrusters, ESC, and PiwHawk 1
Get approval on purchase request, pick up items from Carl Thibault, P. Eng
Fix broken MAVROS package, discovered error in packages, use sed to modify CMakeLists.txt and remove the tests from the build
Meet with Client again, temper expectations - Reached out but he hasn't responded