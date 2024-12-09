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

Foxglove Bridge

# Contents
AUV Container Controller (ACC.sh)

x86_64 AUV Container, Nvidia GPU required

aarch64 AUV Container, Nvidia Jetson Orin NX 8GB minimum

# To setup

## x86_64 (Development Toolchain)
### 1. Install Ubuntu 22.04 (Dual-Boot is fine)
The system must have an Nvidia CUDA-capable GPU

### 2. Install Nvidia CUDA Toolkit
https://developer.nvidia.com/cuda-downloads

### 3. Install Docker Engine for Linux
https://docs.docker.com/engine/install/ubuntu/

### 4. Follow Post-install Steps for Linux (Docker Engine)
https://docs.docker.com/engine/install/linux-postinstall/

### 5. Install Nvidia Container Toolkit
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

### 6. Link Nvidia Container Toolkit to Docker Engine
On above webpage

### 7. Install the ZED SDK
https://www.stereolabs.com/en-ca/developers/release#82af3640d775

Suggest running AI optimization while installing (will take a while)

### 8 (1 if production environment). Install Nvidia SDK Manager
https://developer.nvidia.com/sdk-manager

You will need to create an Nvidia Developer Account

## 9. Run "./ACC.sh -b x86_64" to build the container

## aarch64
### 1 (On x86_64). Flash the Target Device
During this step you should aim to install all dependency libraries and Software development kits

### 2 (On aarch64). Ensure Python3, Python3-pip, and Zstd are Installed
Will be needed for next steps

### 3. Run pip3 install onnx
Otherwise the ZED SDK will encounter errors

### 4. Install the ZED SDK
https://www.stereolabs.com/en-ca/developers/release#82af3640d775

Suggest running AI optimization while installing (will take a long time)

## 5. Ensure all devices are connected to the Jetson

## 6. Configure Network Connections (TODO)

## 6. Run the ZED Camera to ensure default calibration file exists

## 7. Run "./ACC.sh -b aarch64" to build the container
Note, you must have a Personal Access Token (PAT) to build the project, located in the .tokens directory.



# To Start

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

# Changelog
The following versions have been deemed stable enough to consider signifigant milestones, and starting June 17th, 2024 will be uploaded to github

## V7.0 - August 24th, 2024
Added AUV ROS2 controller project into appropriate dockerfiles

## V6.0 - August 18th, 2024
Started looking into controlling boards power capabilities

/sys/devices/platform/17000000.gpu/devfreq/17000000.gpu - For GPU control

/sys/devices/system/cpu/cpu0/cpufreq - For CPU control

Starting work on ROS2 Controller module

## V5.0 - July XXth, 2024
Created and deployed temporary fixes for ISSAC ROS V3.0.1, made signifigant headway connecting FCU to computer (via companion computer)

Working on debugging Ardusub ROS2 Driver - Seems to be dead end

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
