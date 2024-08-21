#!/bin/bash

#AUV Toolkit Container Entrypoint - Program developed by Robert Fudge
GNSS_DIR="$@"

set -e

source ${HOME}/.bashrc

sed -i "3 c\    port: \"${GNSS_DIR}\" " ${HOME}/ros_ws/src/nmea_navsat_driver/config/nmea_serial_driver.yaml
cd ~/ros_ws


exec bash -i