#!/bin/bash
# Entrypoint — source ROS and exec command
source /opt/ros/jazzy/setup.bash
source /home/robot/hospital_ws/install/setup.bash
exec "$@"
