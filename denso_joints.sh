#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/dung/denso_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic pub --once /denso/cmd std_msgs/msg/String 'data: demo' >/dev/null 2>&1
echo "[INFO] Da bat dau chu trinh chuyen dong Link 1 -> Link 5 (2.0s / link) tren RViz!"
